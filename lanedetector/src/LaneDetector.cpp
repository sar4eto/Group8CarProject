/**
 * lanedetector - Application skeleton for detecting lane markings.
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
#include <memory>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "opendavinci/odtools/player/Player.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "LaneDetector.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace odtools::player;
        using namespace automotive;
        using namespace cv;

        LaneDetector::LaneDetector(const int32_t &argc, char **argv) : 
        TimeTriggeredConferenceClientModule(argc, argv, "LaneDetector"),
            m_hasAttachedToSharedImageMemory(false),
            m_sharedImageMemory(),
            m_image(NULL),
            m_debug(false),
            m_vehicleControl() {}

        LaneDetector::~LaneDetector() {}

        void LaneDetector::setUp() {
            // This method will be called automatically _before_ running body().

            // If in debug mode, display the image from the camera feed.
            if (m_debug) {
                cvNamedWindow("Camera Feed Image", CV_WINDOW_AUTOSIZE);
                cvMoveWindow("Camera Feed Image", 300, 100);
            }
        }

        void LaneDetector::tearDown() {
            // This method will be called automatically _after_ return from body().
            if (m_image != NULL) {
                cvReleaseImage(&m_image);
            }

            if (m_debug) {
                cvDestroyWindow("Camera Feed Image");
            }
        }

        bool LaneDetector::readSharedImage(Container &c) {
            bool retVal = false;

            if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage> ();

                // Check if we have already attached to the shared memory containing the image from the virtual camera.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
                }

                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {
                    // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);

                    if (m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, si.getBytesPerPixel());
                    }

                    // Example: Simply copy the image into our process space.
                    if (m_image != NULL) {
                        memcpy(m_image->imageData, m_sharedImageMemory->getSharedMemory(), si.getWidth() * si.getHeight() * si.getBytesPerPixel());
                    }

                    // Mirror the image.
                    cvFlip(m_image, 0, -1);

                    retVal = true;
                }
            }
            return retVal;
        }



        // This method is called to process an image described by the SharedImage data structure.
        void LaneDetector::processImage() {
             Mat dst;
            if (m_debug) {
                // THREE - Colors
                // We convert the image from one color space to another using CvtColor, smooth the image
                //using cvSmooth, find edges using cvCanny, then applying everything from the Ipl image onto 
                //the mat img so that we can see the lines being drawn on the grayscale background
                IplImage *gray = cvCreateImage(cvGetSize(m_image),IPL_DEPTH_8U,1);
                cvCvtColor(m_image,gray,CV_BGR2GRAY);
                cvSmooth(gray,gray, CV_BLUR, 3,3);
                cvCanny(gray,gray, 50,200,3);
                dst = gray;
                cvMerge(gray,gray,gray, NULL, m_image);
            } 

            if (m_image !=NULL){
            // Christians code calculations of where to draw the lines && drawing the colored lines!
            // Changed the for loop to only draw one line and only at the bottom of the screen
            for(int32_t y = m_image->height - 90; y > m_image->height * .8; y -= 10) {
                // Search from middle to the left:
                CvScalar pixelLeft;
                CvPoint left;
                left.y = y;
                left.x = -1;
                for(int x = m_image->width/2; x > 0; x--) {
                    pixelLeft = cvGet2D(m_image, y, x);
                    if (pixelLeft.val[0] >= 200) {
                        left.x = x;
                        break;
                    }
                }
                
                // Search from middle to the right:
                CvScalar pixelRight;
                CvPoint right;
                right.y = y;
                right.x = -1;
                for(int x = m_image->width/2; x < m_image->width; x++) {
                    pixelRight = cvGet2D(m_image, y, x);
                    if (pixelRight.val[0] >= 200) {
                        right.x = x;
                        break;
                    }
                }

                    if (left.x > 0) {
                        //Draws a green line
                        cvLine(m_image, cvPoint(m_image->width/2, y), left, Scalar(0, 255, 0), 1, 8);
                    }
                    if (right.x > 0) {
                        //Draws a red line
                        cvLine(m_image, cvPoint(m_image->width/2, y), right, Scalar(0, 0, 255), 1, 8);
                    }

                //Here starts the Lane Following code
                //The speed is constant and it never changes
                m_vehicleControl.setSpeed(2);

                int leftValue = (m_image->width/2 - left.x); 
                int rightValue = (right.x - m_image->width/2);
                int difference = (leftValue - rightValue); 
                
                //here we get the absolute value of the difference between the two lines (green and red)

                //INTERSECTION VALUES
                
                int noLineDifferenceBig = 321;
                int noLine = 642;
                
                //FORWARD VALUES - repositioning in the middle
                
                int ForwardValueMin = 150;
                int ForwardValueMax = 400;

                //LEFT TURN VALUES

                int LeftTurnLeftValueMin = 290;
                int LeftTurnLeftValueMax = 310;

                int LeftTurnRightValueMin = 90;
                int LeftTurnRightValueMax = 180; 

                //RIGHT TURN VALUES

                int RightTurnLeftValueMin = 35;
                int RightTurnLeftValueMax = 130;

                int RightTurnRightValueMin = 185;
                int RightTurnRightValueMax = 300;
                

                    //FORWARD
                if (leftValue > ForwardValueMin && leftValue < ForwardValueMax && rightValue > ForwardValueMin && rightValue < ForwardValueMax){      
                    m_vehicleControl.setSteeringWheelAngle(0);
                    cout<<"Forward"<<endl;

                    //TURN RIGHT
                } else if ((leftValue > RightTurnLeftValueMin && leftValue < RightTurnLeftValueMax) || (rightValue > RightTurnRightValueMin && rightValue < RightTurnRightValueMax)){
                    m_vehicleControl.setSteeringWheelAngle(25);
                    cout<<"Turning right"<<endl;

                    //TURN LEFT
                } else if ((leftValue > LeftTurnLeftValueMin && leftValue < LeftTurnLeftValueMax) || (rightValue > LeftTurnRightValueMin && rightValue < LeftTurnRightValueMax)){
                    m_vehicleControl.setSteeringWheelAngle(-25);
                    cout<<"Turning left"<<endl;

                    //KEEP FORWARD INTERSECTION
                } else if ((difference == noLineDifferenceBig)|| (difference == noLine)){
                    m_vehicleControl.setSteeringWheelAngle(0);
                    cout <<"Intersection"<<endl;
                   }       
                }
            }

                if (m_image != NULL) {
                    cvShowImage("Camera Feed Image", m_image);
                    cvWaitKey(10);
                }

            // Create container for finally sending the data.
            Container c(m_vehicleControl);
            // Send container.
            getConference().send(c);

}

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LaneDetector::body() {
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t> ("lanedetector.debug") == 1;

            unique_ptr<Player> player;
/*
            // Lane-detector can also directly read the data from file. This might be interesting to inspect the algorithm step-wisely.
            odcore::io::URL url("file://recording.rec");

            // Size of the memory buffer.
            const uint32_t MEMORY_SEGMENT_SIZE = kv.getValue<uint32_t>("global.buffer.memorySegmentSize");

            // Number of memory segments.
            const uint32_t NUMBER_OF_SEGMENTS = kv.getValue<uint32_t>("global.buffer.numberOfMemorySegments");

            // If AUTO_REWIND is true, the file will be played endlessly.
            const bool AUTO_REWIND = true;

            // We do not want player to run in parallel but we want to process frame by frame sequentially.
            const bool THREADING = false;

            // Construct the player.
            player = unique_ptr<Player>(new Player(url, AUTO_REWIND, MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING));
*/

            // Main data processing loop.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;

                // Use the shared memory image.
                Container c;
                if (player.get() != NULL) {
                    // Read the next container from file.
                    c = player->getNextContainerToBeSent();
                }
                else {
                    // Get the most recent available container for a SHARED_IMAGE.
                    c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());
                }

                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image.
                if (true == has_next_frame) {
                    processImage();
                }

            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    } // miniature
} // automotive
