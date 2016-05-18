#include <cstdio>
#include <cmath>
#include <iostream>
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "Overtaker.h"

#define Infrared_RightForward 0
#define Infrared_RightBack 1
#define UltraSonic_Front 3

namespace automotive{
    namespace miniature{
        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        Overtaker::Overtaker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "overtaker") {
        }
        Overtaker::~Overtaker(){}
        void Overtaker::setUp(){}
        void Overtaker::tearDown(){}

        bool cmpf(float A, float B, float epsilon = 0.005f) {       //compare 2 float values, if the difference is less than the epsilon they are equal
            return (fabs(A - B) < epsilon);
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Overtaker::body() {

            enum overtakeState { DETECTED_OBSTACLE, POSITIONING_LEFT, POSITIONING_RIGHT, OVERTAKING, RE_POSITIONING_RIGHT, RE_POSITIONING_LEFT };
            overtakeState overtakeState = DETECTED_OBSTACLE;    // We are making the assumption that that this module starts with the obstacle detected

            double startPosLeft = 0;    // variables to calculate distance travelled in the turning states
            double midPos = 0;
            double rePosRight = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                Container containerVehicleData = getKeyValueDataStore().get(VehicleData::ID());     // Get the most recent vehicleData for the wheel encoder
                VehicleData vd = containerVehicleData.getData<VehicleData> ();
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());   // Get most recent sensor board data:
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                SteeringData steeringControl;
                steeringControl.setExampleData(0);

                double irRightBack = sbd.getValueForKey_MapOfDistances(Infrared_RightBack);         //get the latest ir values for the right back IR sensor
                double irRightForward = sbd.getValueForKey_MapOfDistances(Infrared_RightForward);
                double usFront = sbd.getValueForKey_MapOfDistances(UltraSonic_Front);       // ultrasonic sensor
                double distTravelled = vd.getAbsTraveledPath();     // get the distance travelled from the wheel encoder vehicle data

                if(overtakeState == DETECTED_OBSTACLE){     // initial state, waiting for an obstacle to come within the set range
                	steeringControl.setExampleData(0);
                    if(usFront <= 55  && usFront > 1) overtakeState = POSITIONING_LEFT;       // if the obstacle is close enough change state to turn left
                }

                if(overtakeState == POSITIONING_LEFT){      // US sensors detects the obstacle close enough and starts turning left
                    steeringControl.setExampleData(-30);
                    if(cmpf(startPosLeft, 0)) startPosLeft = distTravelled;     // if startPosLeft is 0 - set it to the current distance travelled 
                    if(irRightForward > 90 && distTravelled - startPosLeft > 10){
                        midPos = distTravelled;         // set midPos (also start of right turn) to the new distTravelled  
                        overtakeState = POSITIONING_RIGHT;
                    }
                }

                if(overtakeState == POSITIONING_RIGHT){     // turning right to position in left lane
                    steeringControl.setExampleData(30);
                    if((fabs(irRightBack - irRightForward) < 30) && distTravelled - midPos > 10) overtakeState = OVERTAKING;  // check difference between ir right front & right back, also check if we've turned enough
                }

                if(overtakeState == OVERTAKING){        // overtake obstacle in left lane
                    steeringControl.setExampleData(0); 	//when steeringControl is 0 - lanefollower has the control 
                    if(irRightForward < 50) overtakeState = RE_POSITIONING_RIGHT;       // forward IR doesn't detect the obstacle anymore
                }

                if(overtakeState == RE_POSITIONING_RIGHT){      // when front IR is not sensing any values , turn left back in to right lane
                    steeringControl.setExampleData(30);
                    if(cmpf(rePosRight, 0)) rePosRight = distTravelled;         // if not set, rePosRight = the total distance travelled
                    if(irRightBack < 50 && distTravelled - rePosRight > 10) overtakeState = DETECTED_OBSTACLE;      // when IR back doesn't detect anymore
                }

                Container steeringControlContainer(steeringControl);
                getConference().send(steeringControlContainer);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
}
