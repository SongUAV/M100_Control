/* Reforges.cpp
 *
 * Copyright Song Chen, Reforges
 *
 */

#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
using namespace std;
using namespace cv;
#include <cmath>
#include "includes/TagDetector.h"
#include "includes/Tag36h11.h"
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <cstring>


#include "includes/LinuxSerialDevice.h"
#include "includes/LinuxThread.h"
#include "includes/LinuxSetup.h"
#include "includes/LinuxCleanup.h"
#include "includes/ReadUserConfig.h"
//#include "LinuxMobile.h"
#include "includes/LinuxFlight.h"
//#include "LinuxInteractive.h"
#include "includes/LinuxWaypoint.h"
//#include "LinuxCamera.h"

//#include "DJI_Follow.h"
#include "includes/DJI_Flight.h"
#include "includes/DJI_Version.h"
#include "includes/DJI_WayPoint.h"

#include "includes/serialib.h"
using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;
#define         DEVICE_PORT             "/dev/ttyTHS1"                         // ttyS0 for linux

inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}

/*void wRo_to_euler(const Eigen::Matrix3d &wRo, double &yaw, double &pitch, double &roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}*/

class AprilTAG {
public:
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;


  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  float bias_z;
  float bias_y;
  float  ATpos_x;
  float  ATpos_y;
  float  ATpos_z;

  bool detected;
  cv::VideoCapture m_cap;



  // default constructor
  AprilTAG():
    
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    //m_width(800),
    //m_height(600),
    m_width(800),
    m_height(600),
    m_tagSize(0.162),
    //m_fx(1080*0.78*0.9),
    //m_fy(1920*0.78*0.9),
    m_fx(800*1.01),
    m_fy(600*1.01),
    m_px(m_width/2),
    m_py(m_height/2),
    m_deviceId(0),
    //bias_z(0.33),
    //bias_y(0.4)
    bias_z(0),
    bias_y(0)
    {}
  // parse command line options to change default behavior

  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
  }

  void setupVideo() {

    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
      cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }

    //m_cap.set(CAP_PROP_FPS, 3);
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;


  }


  void print_detection(AprilTags::TagDetection& detection)  {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    /*Eigen::Matrix3d F;
    F <<
      1, 0,  0,
      0,  -1,  0,
      0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);*/
    ATpos_x= translation(0);
    ATpos_y=translation(1)-bias_y;
    ATpos_z=translation(2)-bias_z;
    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << ATpos_y
         << ", z=" << ATpos_z
         << endl;

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
  }

  void processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);


    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);


    // print out each detection
    if (detections.size()>=1){
      detected =true;
    }
    else{
      detected=false;
    }
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
      print_detection(detections[i]);
    }
  }

  // Load and process a single image
  void getmage() {
    cv::Mat image;
    cv::Mat image_gray;
    m_cap >> image;
    processImage(image, image_gray);
      
    
  }

};
pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
  void* loop( void *reforeges) {
    //int image_count =0;
    AprilTAG *mydata;
    mydata=(AprilTAG *)reforeges;
    cv::Mat image;
    cv::Mat image_gray;
    //string name="capture.png";
    //stringstream itostring;
    while (true) {
      //image_count++;
     // for (int i=0;i<6;i++){
      // m_cap>>image;
      //}
      pthread_mutex_lock(&mutex);
      //usleep(10000);
      mydata->m_cap >> image;

      //itostring << image_count;
      //imwrite(name.insert(7,itostring.str()), image);
      //cout << "here" << endl;
      mydata->processImage(image, image_gray);
      pthread_mutex_unlock(&mutex);
    }
  }

  void* grabpictures(void *reforeges){
       AprilTAG *mydata;
       mydata=(AprilTAG *)reforeges;
      //if (!mydata->m_cap.grab()){
       // cout << "can not grab images." << endl;
      //}
      //else
      //{
       cv::Mat image;
        while(true){
        
        mydata-> m_cap >> image;
        usleep(40000);
        //cout << mydata-> m_cap.get(CV_CAP_PROP_POS_MSEC ) << endl;
        //cout << "grabbing images."<< endl;
        }
      //}
      return NULL;
    }
/*precision_landing(CoreAPI* api,Flight* flight, AprilTAG* reforges, float descending_speed){
  if (reforges->detected){
    moveByPositionOffset(api, flight, reforges->ATpos_z,reforges->ATpos_y,descending_speed,0, 1000,1,5);
  }
  else{
    moveByPositionOffset(api, flight,0,0,0,0, 1000,1,5);
  }
  
}*/
int main() {
	int deploy=2;
	int done=9;

	LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
	CoreAPI* api = new CoreAPI(serialDevice);
	Flight* flight = new Flight(api);
	WayPoint* waypointObj = new WayPoint(api);
	//Camera* camera = new Camera(api);
	LinuxThread read(api, 2);
	//LinuxThread send(api, 1);
	extern int obtainingControl;
	obtainingControl=0;
	int setupStatus = setup(serialDevice, api, &read);
	if (setupStatus == -1)
	{
		std::cout << "This program will exit now. \n";
		return 0;
	}
    serialib LS;                                                            // Object of the serialib class
    int Ret;                                                                // Used for return values
    //char Buffer[2];
    char Buffer;
    Ret=LS.Open(DEVICE_PORT,9600);                                        // Open serial link at 115200 bauds
    if (Ret!=1) {                                                           // If an error occured...
        printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
        return Ret;                                                         // ... quit the application
    }
    printf ("Serial port opened successfully !\n");
    Ret=LS.WriteString("AT\n");                                             // Send the command on the serial port
    if (Ret!=1) {                                                           // If the writting operation failed ...
        printf ("Error while writing data\n");                              // ... display a message ...
        return Ret;                                                         // ... quit the application.
    }
    printf ("Write operation is successful \n");

    while(true)
    {
    	// Read a string from the serial device
    	// Ret=LS.ReadString(Buffer,1000);                                // Read a maximum of 128 characters with a timeout of 5 seconds
    	Ret=LS.ReadChar(&Buffer,1000);                                                                        // The final character of the string must be a line feed ('\n')
    	//if (Ret>0)
      int counter=0;
    	std::cout << Buffer << endl;
    	std::cout << Ret << endl;

    	if (Ret==0)
    	{
        	Ret=LS.ReadChar(&Buffer,1000);                                                                        // The final character of the string must be a line feed ('\n')
        	//if (Ret>0)
        	std::cout << Buffer << endl;
        	std::cout << Ret << endl;
    	}
    	// If a string has been read from, print the string
    	//    printf ("String read from serial port : %s",Buffer);
    	// else
    	//  printf ("TimeOut reached. No data received !\n");                   // If not, print a message.
    	//std::cout << (Buffer-'0') << endl;
    	//if (Ret==1 && deploy==(Buffer-'0'))
      if (true)
    	{
    		//LS.Close();
    		//usleep(15000000);
    		//std::cout << "here" << endl;
    		unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
    		usleep(500000);
    		int blockingTimeout =1;
    		ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);
        usleep(10000000);
    		if  (takeoffStatus.status == 1)
    		{
				  //wayPointMissionExample(api, waypointObj,blockingTimeout);
    		  moveByPositionOffset(api, flight, 2,0,0,0, 1000,1,20);
    		      //! Land
				  usleep(500000);
				  pthread_t grabbing, processing;

				  AprilTAG* reforges = new AprilTAG();
				  reforges->setup();
				  reforges->setupVideo();
				  pthread_create(&grabbing, NULL, grabpictures,  (void*)reforges);
          pthread_create(&processing, NULL, loop,  (void*)reforges);
				  //pthread_join(grabbing, NULL);
          cv::Mat image;
          cv::Mat image_gray;
          while (counter <100) {
              /*for (int i=0;i<6;i++){
                reforges->m_cap >> image;
              }
              reforges->m_cap >> image;
              reforges->processImage(image, image_gray);*/
              moveByPositionOffset(api, flight, reforges->ATpos_z,-reforges->ATpos_y,0,0, 1000,1,5);
              usleep(400000);
              counter ++;
            }
          /*moveByPositionOffset(api, flight, 0,0,0,0, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 2,0,0,0, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 0,2,0,0, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 0,0,1,0, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 0,0,0,30, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 2,0,0,30, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 0,2,0,30, 1000,1,5);
          usleep(1000000);
          moveByPositionOffset(api, flight, 0,0,1,30, 1000,1,5);
          usleep(1000000);*/
				  ackReturnData landingStatus = landing(api, flight,blockingTimeout);
          int cleanupStatus = cleanup(serialDevice, api, flight, &read);
    		      //int moveByPositionOffset(CoreAPI* api, Flight* flight, float32_t xOffsetDesired, float32_t yOffsetDesired, float32_t zOffsetDesired, float32_t yawDesired ,  int timeoutInMs, float yawThresholdInDeg, float posThresholdInCm)

        }
    	    else
        {
    	      //Try to land directly
    	     ackReturnData landingStatus = landing(api, flight,blockingTimeout);
        }
    	}
    	if (Ret==1 && done==(Buffer-'0'))
    	{
    		 int cleanupStatus = cleanup(serialDevice, api, flight, &read);
    	}
    }
  return 0;
}
	
