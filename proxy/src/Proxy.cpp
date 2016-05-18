/** 
Code based on initial skeleton found at https://github.com/se-research/OpenDaVINCI/blob/master/automotive/miniature/proxy/src/Proxy.cpp **/


#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <vector>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

#include "opendavinci/odcore/wrapper/SerialPort.h"
#include "opendavinci/odcore/wrapper/SerialPortFactory.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "OpenCVCamera.h"

#ifdef HAVE_UEYE
    #include "uEyeCamera.h"
#endif

#include "Proxy.h"
#include "SerialReceiveBytes.hpp"

namespace automotive {
    namespace miniature {
        using namespace std;
        using namespace odcore::wrapper;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;

        array<vector<double>, 5> medianBuffers; // Moving median buffer for sensor values

        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera(){
            }

        Proxy::~Proxy() {
        }

        void Proxy::setUp() {
            // Pre-made camera set-up code, found at https://github.com/se-research/OpenDaVINCI/blob/master/automotive/miniature/proxy/src/Proxy.cpp

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Create built-in recorder.
            const bool useRecorder = kv.getValue<uint32_t>("proxy.useRecorder") == 1;
            if (useRecorder) {
                // URL for storing containers.
                stringstream recordingURL;
                recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS() << ".rec";
                // Size of memory segments.
                const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.memorySegmentSize");
                // Number of memory segments.
                const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.numberOfMemorySegments");
                // Run recorder in asynchronous mode to allow real-time recording in background.
                const bool THREADING = true;
                // Dump shared images and shared data?
                const bool DUMP_SHARED_DATA = getKeyValueConfiguration().getValue<uint32_t>("proxy.recorder.dumpshareddata") == 1;

                m_recorder = unique_ptr<Recorder>(new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));
            }

            // Create the camera grabber.
            const string NAME = getKeyValueConfiguration().getValue<string>("proxy.camera.name");
            string TYPE = getKeyValueConfiguration().getValue<string>("proxy.camera.type");
            std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
            const uint32_t ID = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.id");
            const uint32_t WIDTH = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.width");
            const uint32_t HEIGHT = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.height");
            const uint32_t BPP = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.bpp");

            if (TYPE.compare("opencv") == 0) {
                m_camera = unique_ptr<Camera>(new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP));
            }
            if (TYPE.compare("ueye") == 0) {
                #ifdef HAVE_UEYE
                m_camera = unique_ptr<Camera>(new uEyeCamera(NAME, ID, WIDTH, HEIGHT, BPP));
                #endif
            }

            if (m_camera.get() == NULL) {
                cerr << "No valid camera type defined." << endl;
            }

            // End of pre-made camera set-up code
        }

        void Proxy::tearDown() {
        }

        void Proxy::distribute(Container c) {
            // Pre-made distribute function for containers
            // Store data to recorder.
            if (m_recorder.get() != NULL) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                m_recorder->store(c);
            }
            // Share data.
            getConference().send(c);
        }


        bool cmpf(float A, float B, float epsilon = 0.005f) {  // Compare float values, found at http://noobtuts.com/cpp/compare-float-values
            return (fabs(A - B) < epsilon);
        }

        double getMedian(int index, double val){ // Given a new sensor value (val) for an IR sensor (index) return the median.
            if(medianBuffers[index].size() < 5) { // Median window of 5 values
                medianBuffers[index].push_back(val);
            }
            else {
                medianBuffers[index].erase(medianBuffers[index].begin()); // If here are more than 5 values, remove the first and append the latest value to the end
                medianBuffers[index].push_back(val);
            }

            vector<double> sortedBuffer = medianBuffers[index]; // Temporary 'sorted' buffer, in order to find the median 
            sort(sortedBuffer.begin(), sortedBuffer.end()); // Sort the buffer

            if(sortedBuffer.size() < 5) return sortedBuffer[sortedBuffer.size() / 2]; //Return the 'middle' value 
            else return sortedBuffer[3]; //Return the 'middle' value 
        }

        double getMedian(int index) { // Given an IR sensor get the current medain.
            vector<double> sortedBuffer = medianBuffers[index]; // Temporary 'sorted' buffer, in order to find the median 
            sort(sortedBuffer.begin(), sortedBuffer.end()); // Sort the buffer

            if(sortedBuffer.size() < 5) return sortedBuffer[sortedBuffer.size() / 2]; //Return the 'middle' value 
            else return sortedBuffer[3];
        }

        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {

            // SERIAL PORT SETUP

            const string SERIAL_PORT = "/dev/ttyACM0";
            const uint32_t BAUD_RATE = 115200;
            std::shared_ptr<SerialPort>serial(SerialPortFactory::createSerialPort(SERIAL_PORT, BAUD_RATE));
            SerialReceiveBytes handler;
            serial->setStringListener(&handler);
            serial->start();

            // END PORT SETUP

            vector<double> oldUsVals; // In case there are no latest values, emit old values

            uint32_t captureCounter = 0;
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                // Capture frame.
                if (m_camera.get() != NULL) {
                    odcore::data::image::SharedImage si = m_camera->capture();

                    Container c(si);
                    distribute(c);
                    captureCounter++;
                }

                // SENSOR DATA PROXY (ARDUINO->ODROID) ----------------------------------------------------------------------------------

                if(captureCounter > 60){ // To ensure serial handshakes and value buffers inside 
                                        // SerialRecieveBytes have enough time to be instantiated.
                                        // Increase this in case of segmentation fault on start-up

                    uint32_t sensors = 5; // Number of sensors on the vehicle
                    map<uint32_t, double> vals;

                    // cout << "We are checking if there are new IR values" << endl;

                    if(handler.irHasNewVals()) { // If the IR buffer has new values..
                        vector<double> irVals = handler.getIR(); // Get latest values for all IR sensors
                        for(uint32_t i=0; i < irVals.size(); i++){ // Get and store the new median for every IR sensor
                            vals[i] = getMedian(i, irVals[i]);

                            // cout << "IRMedian:" << getMedian(i, irVals[i]) << endl;
                            // vals[i] = irVals[i];
                            // cout << "IRval" << i << ":" << irVals[i] << endl;
                        }

                    }

                    else {
                        for(uint32_t i=0; i < 3; i++) { // If there are no new values, just get the median of the existing values.
                            vals[i] = getMedian(i);
                        }
                    }

                    if(handler.usHasNewVals()) { // If the US buffer has new values..
                        // cout << "US new vals" << endl;
                        vector<double> usVals = handler.getUS(); // Get the latest values for all US sensors
                        for(uint32_t i=0; i < usVals.size(); i++){
                            // cout << "USMedian:" << getMedian(i + 3, usVals[i]) << endl;
                            // vals[i + 3] = usVals[i]; // Store the latest US values.
                            vals[i+3] = getMedian(i + 3, usVals[i]); // Store the latest median of US values.
                            // cout << "USval" << i << ":" << usVals[i] << endl;
                        }

                        oldUsVals = usVals;
                    }

                    else { // If there are no values, just re-use the latest (previous) values
                        for(uint32_t i=3; i < 5; i++) {
                            vals[i] = getMedian(i);
                            // vals[i] = oldUsVals[i - 3];
                        }
                    }

                    SensorBoardData sbd(sensors, vals); // Pack and send SBD
                    Container csbd(sbd);
                    getConference().send(csbd);

                    VehicleData vd; // Pack and send VD
                    vd.setAbsTraveledPath((handler.getDist()));
                    Container cvd(vd);
                    getConference().send(cvd);
                }

                // END OF SENSOR STUFF ----------------------------------------------------------------------------------
                
                // VEHICLE COMMAND PROXY (Odroid->Arduino)
                Container vehicleControlContainer = getKeyValueDataStore().get(automotive::VehicleControl::ID()); // Vehicle Control for Lane Follower
                VehicleControl vehicleControlData = vehicleControlContainer.getData<VehicleControl> ();

                Container overtakerDataContainer = getKeyValueDataStore().get(automotive::miniature::SteeringData::ID()); // Steering Data for Overtaker
                SteeringData overtakerData = overtakerDataContainer.getData<SteeringData> ();

                double speed = vehicleControlData.getSpeed(); // Get the speed set by lane follower
                double lanefollowerSteeringAngle = vehicleControlData.getSteeringWheelAngle(); // Get the steering angle set by lane follower
                double overtakerSteeringAngle = overtakerData.getExampleData(); // Get the steering angle set by overtaker

                int lanefollowerSteeringAngleInt = (int) lanefollowerSteeringAngle; // Cast angle to int
                int overtakerSteeringAngleInt = (int) overtakerSteeringAngle;

                // cout << "lanefollowerSteeringAngleInt:" << lanefollowerSteeringAngleInt << endl;

                if(speed < 0){ // Send BACKWARDS command
                    serial->send("[M,B]");
                } 
                
                if(cmpf(speed, 0)){ // If speed == 0, send BRAKE command
                    serial->send("[M,F,0]");
                } 
                
                if(cmpf(speed, 0.5)){ // If speed == 0.5, send SLOW FORWARD command
                    serial->send("[M,F,S]");
                } 
                
                if(cmpf(speed, 1)){ // If speed == 1, send FAST FORWARD command
                    serial->send("[M,F,F]");
                }

                if(overtakerSteeringAngleInt == 0){ // If overtaker is not controlling, parse the lane-follower steering command
                    if(lanefollowerSteeringAngleInt < 0){ // If it is less than zero, send the TURN LEFT COMMAND
                        lanefollowerSteeringAngleInt = lanefollowerSteeringAngleInt * (-1); // Make angle positive
                        std::string angleString = std::to_string(lanefollowerSteeringAngleInt); // Stringify
                        serial->send("[T,L," + angleString + "]"); // Send string over serial
                    }
                    else if(lanefollowerSteeringAngleInt == 0){ // If angle is 0, send STRAIGHT command
                        serial->send("[T,R,0]");
                    }
                    else if(lanefollowerSteeringAngleInt > 0){ // If angle > 0, send TURN RIGHT COMMAND
                        std::string angleString = std::to_string(lanefollowerSteeringAngleInt);
                        serial->send("[T,R," + angleString + "]");
                    }
                }

                else { // If overtaker is sending an angle, override lane-follower
                    if(overtakerSteeringAngleInt < 0){
                        overtakerSteeringAngleInt = overtakerSteeringAngleInt * (-1);
                        std::string angleString = std::to_string(overtakerSteeringAngleInt);
                        serial->send("[T,L," + angleString + "]");
                    }
                    else if(overtakerSteeringAngleInt > 0){ 
                        std::string angleString = std::to_string(overtakerSteeringAngleInt);
                        serial->send("[T,R," + angleString + "]");
                    }
                }

            }

            serial->stop();
            serial->setStringListener(NULL); 

            cout << "Proxy: Captured " << captureCounter << " frames." << endl;

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }
    }
} // automotive::miniature

