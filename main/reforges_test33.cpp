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
#include <fstream>

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
  int AT_ID_L;
  int AT_ID_S;
  bool detected_L;
  bool detected_S;
  bool thread_kill;
  float  ATpos_x_L;
  float  ATpos_y_L;
  float  ATpos_z_L;
  float  ATpos_x_S;
  float  ATpos_y_S;
  float  ATpos_z_S;
  std::fstream fs;
  cv::VideoCapture m_cap;



  // default constructor
  AprilTAG():
    
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    //m_width(800),
    //m_height(600),
    m_width(1920),
    m_height(1080),
    m_tagSize(0.162),
    //m_fx(1080*0.78*0.9),
    //m_fy(1920*0.78*0.9),
    m_fx(1080*0.53),
    m_fy(1080*0.53),
    m_px(1080*0.6/2),
    m_py(1080*0.45/2),
    m_deviceId(0),
    AT_ID_L(17),
    AT_ID_S(18),
    detected_L(false),
    detected_S(false),
    thread_kill(false)
    {}
  // parse command line options to change default behavior
  void reset(){
    detected_L=false;
    detected_S=false;
  }
  void setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
  }
  void releaseVideo(){
    if(m_cap.isOpened()) {
      m_cap.release();
    }
    if(m_cap.isOpened()) {
      cout << "cant release the camera" << endl;
    }
  }
  void setupVideo() {

    // find and open a USB camera (built in laptop camera, web cam etc)
    if (!m_cap.isOpened()){
      m_cap = cv::VideoCapture(m_deviceId);
    }
    if(!m_cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
      exit(1);
    }

    //m_cap.set(CAP_PROP_FPS, 3);
    //m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    //m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;


  }


  void print_detection(AprilTags::TagDetection& detection)  {
    //cout << "  Id: " << detection.id
     //    << " (Hamming: " << detection.hammingDistance << ")";

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
    
    if (detection.id==AT_ID_L){
      detected_L=true;
      ATpos_x_L= translation(0);
      ATpos_y_L=-translation(1);
      ATpos_z_L=translation(2);

    }
    if (detection.id==AT_ID_S){
      detected_S=true;
      ATpos_x_S= translation(0)/(0.162/0.066);
      ATpos_y_S=-translation(1)/(0.162/0.066);
      ATpos_z_S=translation(2)/(0.162/0.066);
    }

   cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
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
   /*if (detections.size()>=1){
      detected =true;
    }
    else{
      detected=false;
    }*/
    //reset();
    //cout << detections.size() << " tags detected:" << endl;
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
/*  void loop() {
    cv::Mat image;
    cv::Mat image_gray;
    while (true) {
      //image_count++;
      m_cap >> image;

      //itostring << image_count;
      //imwrite(name.insert(7,itostring.str()), image);
      //cout << "here" << endl;
      processImage(image, image_gray);
    }
  }*/
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
  while (!mydata->thread_kill) {
      //image_count++;
     // for (int i=0;i<6;i++){
      // m_cap>>image;
      //}
    pthread_mutex_lock(&mutex);
      //usleep(10000);
    do{
      mydata->m_cap >> image;
    }while(image.empty());
      //itostring << image_count;
      //imwrite(name.insert(7,itostring.str()), image);
      //cout << "here" << endl;
    mydata->processImage(image, image_gray);
    pthread_mutex_unlock(&mutex);
  }
  return NULL;
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
  while(!mydata->thread_kill){
    mydata-> m_cap >> image;
    usleep(40000);
        //cout << mydata-> m_cap.get(CV_CAP_PROP_POS_MSEC ) << endl;
        //cout << "grabbing images."<< endl;
  }
      //}
  return NULL;
}

int precision_landing(CoreAPI* api,Flight* flight, AprilTAG* reforges, float descending_speed, int blockingTimeout, int trylanding){
  int landed=0;
  // height greater than 4m, descend
  if (api->getBroadcastData().pos.height>4.0){
    moveByPositionOffset(api, flight,0,0,3-api->getBroadcastData().pos.height,0, blockingTimeout*200,1,5);
    usleep(200000);
  }
  if (api->getBroadcastData().pos.height<=4.0 && api->getBroadcastData().pos.height > 0.5){
  // stage 1
  // check if the AT_L is detected
    if (reforges->detected_L){
      // height greater than 0.5 m, continuously landing 
      if (reforges->ATpos_x_L>0.5){
        moveByPositionOffset(api, flight,reforges->ATpos_y_L, reforges->ATpos_z_L,descending_speed,0, blockingTimeout*50,1,5);
        usleep(50000);
      }
      // height less than 0.5m ,check the offset 
      else{
        // the offset of x or y greater than 0.2m
        if (fabs(reforges->ATpos_z_L)>=0.2 || fabs(reforges->ATpos_z_L)>=0.2){
          // ascending to 2m and try landing again
          moveByPositionOffset(api, flight, reforges->ATpos_z_L,reforges->ATpos_y_L,1,0, blockingTimeout*500,1,5);
          usleep(500000);
          trylanding++;
        }// end if, offset too large, ascend
      }// end else, check the offset
    }// end if, stage 1
    else{
       // AT_L is not been detected
      moveByPositionOffset(api, flight,0,0,0,0, blockingTimeout*10,1,5);
    }
  }// end if, 
  if (api->getBroadcastData().pos.height <= 0.5){
    // stage 2
    // small AT detected
    if (reforges->detected_S){
    // height greater than 0.2m
      if (reforges->ATpos_x_L>0.3){
        moveByPositionOffset(api, flight, reforges->ATpos_z_S,reforges->ATpos_y_S,descending_speed,0, blockingTimeout*50,1,5);
        usleep(50000);
      }
      // height less than 0.2m ,check the offset 
      else{
        // the offset of x or y greater than 0.1m, land
        if (fabs(reforges->ATpos_z_S)<=0.1 && fabs(reforges->ATpos_z_S)<=0.1){
          ackReturnData landingStatus = landing(api, flight,blockingTimeout);
          usleep(2000000);
          landed=1;
        }
        else{
          moveByPositionOffset(api, flight, reforges->ATpos_z_S,reforges->ATpos_y_S,0.5,0, blockingTimeout*500,1,5);
          usleep(500000);
          trylanding++;
        }
      }
    }
    else{
      // small AT not detected
      moveByPositionOffset(api, flight,0,0,0,0, blockingTimeout*10,1,5);
    }// end else(no small AT)
  }
  return landed;
} 

bool AT_follow(CoreAPI* api,Flight* flight, AprilTAG* reforges, int blockingTimeout){
  bool reached=false;
  //check if the AT is detected
  // detected
  if (reforges->detected_L){
    //reforges->reset();
    if (fabs(reforges->ATpos_z_L)>=0.4||fabs(reforges->ATpos_y_L)>=0.4){
      //moveByPositionOffset(api, flight, reforges->ATpos_z_L,reforges->ATpos_y_L,2-api->getBroadcastData().pos.height,0, blockingTimeout*100,1,5);
      moveWithVelocity(api, flight, (reforges->ATpos_z_L)/0.6, (reforges->ATpos_y_L)/0.6, 0, 0 ,  blockingTimeout*30, 1, 0.05);
      usleep(30000);
      //usleep(300000);
      //LS.WriteChar('1');
      std::cout << "AT_follow" << endl;
    }
    else {
      moveByPositionOffset(api, flight, reforges->ATpos_z_L,reforges->ATpos_y_L,2-api->getBroadcastData().pos.height,0, blockingTimeout*50,1,5);
      usleep(50000);
      //usleep(300000);
      //LS.WriteChar('1');
      std::cout << "AT_follow" << endl;
     
    }
    reforges->reset();
    // offset of x and y are less than 0.1m
  }
  // not detected 
  else{
    //api->getBroadcastData().pos.
    //LS.WriteChar('0');
    moveByPositionOffset(api, flight,0,0,2-api->getBroadcastData().pos.height,0, blockingTimeout*200,1,5);
    usleep(20000);
  }
 return reached;
}
bool precision_landing_test(CoreAPI* api,Flight* flight, AprilTAG* reforges,  float descending_speed, int blockingTimeout){
  if (api->getBroadcastData().pos.height>4.0){
    moveByPositionOffset(api, flight,0,0,3-api->getBroadcastData().pos.height,0, blockingTimeout*200,1,5);
    usleep(200000);
  }

}
int main() {
	serialib LS;                                                            // Object of the serialib class
	int Ret;                                                                // Used for return values
    //char Buffer[2];
	char Buffer;
  char write;
	int deploy=2;
	int done=9;
	int ATfollow=3;
	int PreLanding=8;
	int id_follow=17;
	int id_planding=18;
	int blockingTimeout =1;
	Ret=LS.Open(DEVICE_PORT,9600);                                        // Open serial link at 115200 bauds
	if (Ret!=1) {                                                           // If an error occured...
		printf ("Error while opening port. Permission problem ?\n");        // ... display a message ...
		return Ret;                                                         // ... quit the application
	}
	else {printf ("Serial port opened successfully !\n");}
	pthread_t grabbing, processing;

  AprilTAG* reforges = new AprilTAG();
  reforges->setup();
  reforges->setupVideo();
  reforges->thread_kill = false;
          //pthread_join(grabbing, NULL);
  cv::Mat image;
  cv::Mat image_gray;
          
  pthread_create(&processing, NULL, loop,  (void*)reforges);
  pthread_create(&grabbing, NULL, grabpictures,  (void*)reforges);
  /*Ret=LS.WriteString("AT\n");                                             // Send the command on the serial port
  if (Ret!=1) {                                                           // If the writting operation failed ...
    printf ("Error while writing data\n");                              // ... display a message ...
        return Ret;                                                         // ... quit the application.
  }
  printf ("Write operation is successful \n");*/
  write='a';
  

  std::cout << "11111" << endl;
	while(true)
	{
    	// Read a string from the serial device
    	// Ret=LS.ReadString(Buffer,1000); 
      //LS.WriteChar(write);
      std::cout << "22222" << endl;                               // Read a maximum of 128 characters with a timeout of 5 seconds
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
    	if (Ret==1 && deploy==(Buffer-'0'))
      //if (true)
		{
    		//LS.Close();
    		//usleep(15000000);
    		//std::cout << "here" << endl;
      std::cout << "33333" << endl;
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
				//int setupStatus = setup(serialDevice, api, &read);
			}
			else{
    			unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
    			usleep(500000);
    			
    			ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);

    		
    			if  (takeoffStatus.status == 1)
    			{
            std::cout << "44444" << endl;
				  //wayPointMissionExample(api, waypointObj,blockingTimeout);
    		      //moveByPositionOffset(api, flight, 20,20,10,180, 30000,1,20);
    		      //! Land
				  //usleep(500000);
					/*pthread_t grabbing, processing;

					AprilTAG* reforges = new AprilTAG();
					reforges->setup();
					reforges->setupVideo();

				  //pthread_join(grabbing, NULL);
					cv::Mat image;
					cv::Mat image_gray;

          pthread_create(&processing, NULL, loop,  (void*)reforges);
          pthread_create(&grabbing, NULL, grabpictures,  (void*)reforges);*/
          /*while (counter <100) {
              reforges->m_cap >> image;
              reforges->processImage(image, image_gray);
              moveByPositionOffset(api, flight, reforges->ATpos_z,reforges->ATpos_y,0,0, 1000,1,5);
              counter ++;
            }*/
					while(true){
            //std::cout << "55555" << endl;
					Ret=LS.ReadChar(&Buffer,1000); 
          usleep(20000);
				std::cout << "waiting" << endl;
				moveByPositionOffset(api, flight, 0,0,2-api->getBroadcastData().pos.height,0, blockingTimeout*200,1,3);
				//At follow
					if (Ret==1 && ATfollow==(Buffer-'0')){
            
            //reforges->fs.open("image_detection.txt", std::fstream::in | std::fstream::out | std::fstream::app);
						reforges->thread_kill = false;

						
						int test=0;
						moveByPositionOffset(api, flight, 0,0,2-api->getBroadcastData().pos.height,0, blockingTimeout*200,1,3);
						bool followed=false;
						//usleep(1000000);
						do {  
							test++;
							//Ret=LS.ReadChar(&Buffer,1000); 
							//std::cout << test << endl;
							
              //reforges->fs << test << endl;
              //reforges->fs << reforges->detected_L << endl;

							followed=AT_follow(api,flight, reforges,  blockingTimeout);
              //moveByPositionOffset(api, flight, 1,0,2-api->getBroadcastData().pos.height,0, blockingTimeout*200,1,20);
						}while(test<1000);
						//reforges->thread_kill = true;
              
						//pthread_join(grabbing,NULL);
						//pthread_join(processing,NULL);
            //reforges->releaseVideo();
					}
				// precision landing
					if (Ret==1 && PreLanding==(Buffer-'0')){

					//reforges->thread_kill = false;
         // pthread_create(&processing, NULL, loop,  (void*)reforges);
					//pthread_create(&grabbing, NULL, grabpictures,  (void*)reforges);
					
					int trylanding=0;
					int landed=0;
              // descending speed, m/time
					float descending_speed =-0.2;

					do{
						//usleep(200000);
						landed= precision_landing(api,flight, reforges,  descending_speed,  blockingTimeout,  trylanding);
					}while(landed!=1 || trylanding<5);
					//reforges->thread_kill = true;
					//pthread_join(grabbing,NULL);
					//pthread_join(processing,NULL);
					}
            
					if (Ret==1 && done==(Buffer-'0')){
					ackReturnData landingStatus = landing(api, flight,blockingTimeout);
					int cleanupStatus = cleanup(serialDevice, api, flight, &read);
          reforges->thread_kill = true;
              
          pthread_join(grabbing,NULL);
          pthread_join(processing,NULL);
          //reforges->releaseVideo();
					break;
    		      //int moveByPositionOffset(CoreAPI* api, Flight* flight, float32_t xOffsetDesired, float32_t yOffsetDesired, float32_t zOffsetDesired, float32_t yawDesired ,  int timeoutInMs, float yawThresholdInDeg, float posThresholdInCm)
					}
					}// end while, listening further command
        		}// end if, the drone is alread taken off
        	}
		}
	} //end while
	
	return 0;
}
	
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             