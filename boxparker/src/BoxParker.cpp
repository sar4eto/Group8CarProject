/**
 * boxparker - Sample application for realizing a box parking car.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <iostream>
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "BoxParker.h"

namespace automotive {
    namespace miniature {
        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace automotive;

        BoxParker::BoxParker(const int32_t &argc, char **argv) :   
            TimeTriggeredConferenceClientModule(argc, argv, "BoxParker"),
            m_foundGaps() {}

            BoxParker::~BoxParker() {}

            void BoxParker::setUp() {}
            void BoxParker::tearDown() {}

        vector<double> BoxParker::getFoundGaps() const {
            return m_foundGaps;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode BoxParker::body() {
            double absPathStart = 0;
            double absPathEnd = 0;
            int stageMoving = 0;
            int stageMeasuring = 0;

        while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data describing virtual sensor data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                // Create vehicle control data.
                VehicleControl vc;

                //Initalize the back and front sensors          
                //double BACK_SENSOR = sbd.getValueForKey_MapOfDistances(2);
                double FRONT_SENSOR = sbd.getValueForKey_MapOfDistances(3); 

                //IR sensor
                double RIGHT_SENSOR = sbd.getValueForKey_MapOfDistances(0);
                double DISTANCE_CAR = vd.getAbsTraveledPath(); 

                // Moving state machine. 
                //cout << "Back Sensor = " << BACK_SENSOR << endl;
                cout << "Front Sensor = " << FRONT_SENSOR << endl;   
                cout << "Right Front Sensor = " << RIGHT_SENSOR << endl; 
                cout << "distance = " << DISTANCE_CAR << endl; 
       
        if (stageMoving == 0) { 
                    //Go forward.
                    cout << "moving forward " << endl;
                    //vc.setSteeringWheelAngle(0);
                    vc.setSpeed(1);

                }
                if ((stageMoving > 0) && (stageMoving < 45)) {
                    // Backwards, steering wheel to the right.
                    cout << "Reversing - RIGHT TURN - INTO PARKING. " << endl;
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(30);
                    stageMoving++;
                }
               
                if ((stageMoving >= 45) && (stageMoving < 70))  {
                    // Backwards, steering wheel to the left.
                    cout << "Reversing - LEFT TURN - BACK TO FIRST BOX " << endl;
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-30);
                    stageMoving++;
                }
   if ((stageMoving >= 70) && (stageMoving < 90))  {
                    // Backwards, steering wheel to the left.
                    cout << "Reversing - LEFT TURN - BACK TO FIRST BOX " << endl;
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(30);
                    stageMoving++;
                }
                if (stageMoving >= 90) {
                    // End component.
                    cout << "done - PARKED " << endl;
                    vc.setSpeed(0);
                    stageMoving++;
                    //break;
                } 

                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            stageMeasuring++;
                        }
                    break;

                    case 1:
                        {
                            if ((sbd.getValueForKey_MapOfDistances(0) < 280 )) {
                            // Found distance sequence +, -.
                                cout << "started measuring the distance = " << endl;
                                stageMeasuring = 2;
                                absPathStart = vd.getAbsTraveledPath();
                            }   
                        }
                    break;

                    case 2:
                        {
                            if ((sbd.getValueForKey_MapOfDistances(0) > 280)) {
                                // Found distance sequence -, +.
                                cout << "found gap measuring the distance" << endl;
                                stageMeasuring = 1;
                                absPathEnd = vd.getAbsTraveledPath();

                                const double GAP_SIZE = (absPathEnd - absPathStart);
                                cout << "Gap Size = " << GAP_SIZE << endl;   
                                cerr << "Size = " << GAP_SIZE << endl;
                                m_foundGaps.push_back(GAP_SIZE);

                                if ((stageMoving < 1) && (GAP_SIZE > 20) ) {
                                    vc.setSpeed(-1);
                                    stageMoving = 1;
                                }
                            }   
                        }
                        break;
                    }
                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    } // miniature
} // automotive
