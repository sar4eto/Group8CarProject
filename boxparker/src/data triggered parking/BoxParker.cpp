

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

                const double GAP_SIZE = (absPathEnd - absPathStart);


                if (stageMoving == 0) {
                    //Accelerate
                    cout << "moving forward " << endl;
                    cout << "Gap Size = " << GAP_SIZE << endl;
                    vc.setSpeed(1);
                }

                if ((stageMoving ==1) && (GAP_SIZE > 10)) {
                    //Reverse right turn
                    cout << "Reversing - RIGHT TURN - INTO PARKING. " << endl;
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(30);
                    absPathEnd = vd.getAbsTraveledPath();
                    m_foundGaps.push_back(GAP_SIZE);
                }

                if ((stageMoving ==1) && (GAP_SIZE <= 10) && (GAP_SIZE > 15))  {
                    //Reverse left turn
                    cout << "Reversing - LEFT TURN - BACK TO FIRST BOX " << endl;
                    vc.setSpeed(-1);
                    vc.setSteeringWheelAngle(-30);
                    absPathEnd = vd.getAbsTraveledPath();
                    m_foundGaps.push_back(GAP_SIZE);
                }

                if ((stageMoving ==1) && (GAP_SIZE <= 15) && (GAP_SIZE > 20))  {
                    //Accelerate right turn
                    cout << "Accelerate - RIGHT TURN - BACK TO FIRST BOX " << endl;
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(30);
                    absPathEnd = vd.getAbsTraveledPath();
                    m_foundGaps.push_back(GAP_SIZE);
                }


                if ((stageMoving == 1)&&(GAP_SIZE >= 20)) {
                    //Parking Done
                    cout << "done - PARKED " << endl;
                    vc.setSpeed(0);

                }

                switch (stageMeasuring) {
                    case 0:
                    {
                        stageMeasuring++;
                    }
                        break;

                    case 1:
                    {
                        if ((sbd.getValueForKey_MapOfDistances(0) < 280 )) {
                            //Found distance sequence +, -.
                            cout << "started measuring the distance = " << endl;
                            stageMeasuring = 2;
                            absPathStart = vd.getAbsTraveledPath();
                        }
                    }
                        break;

                    case 2:
                    {
                        if ((sbd.getValueForKey_MapOfDistances(0) > 180)) {
                            // Found distance sequence -, +.
                            cout << "found gap measuring the distance" << endl;
                            stageMeasuring = 1;
                            absPathEnd = vd.getAbsTraveledPath();

                            //const double GAP_SIZE = (absPathEnd - absPathStart);
                            cout << "Gap Size = " << GAP_SIZE << endl;
                            cerr << "Size = " << GAP_SIZE << endl;
                            m_foundGaps.push_back(GAP_SIZE);

                            //Detect GAP size to start reverse.
                            if ((stageMoving < 1) && (GAP_SIZE > 24 ) ) {
                                vc.setSpeed(-1);
                                stageMoving = 1;
                                absPathStart = vd.getAbsTraveledPath();
                                m_foundGaps.push_back(GAP_SIZE);
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


