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
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;
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
  double ATpos_x;
  double ATpos_y;
  double ATpos_z;
  cv::VideoCapture m_cap;



  // default constructor
  AprilTAG():
    
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    //m_width(800),
    //m_height(600),
    m_width(800*0.5),
    m_height(600*0.5),
    m_tagSize(0.162),
    //m_fx(1080*0.78*0.9),
    //m_fy(1920*0.78*0.9),
    m_fx(1080*0.78*0.4),
    m_fy(1920*0.78*0.4),
    m_px(m_width/2),
    m_py(m_height/2),
    m_deviceId(0),
    ATpos_x(0.0),
    ATpos_y(0.0),
    ATpos_z(0.0)
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


  void print_detection(AprilTags::TagDetection& detection) const {
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
    ATpos_y=translation(1);
    ATpos_z=translation(2);
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
  void loop() {
    int image_count =0;
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
  }
};
  void* grabpictures(void *reforeges){
      
      if (!((AprilTAG *)reforeges)->m_cap.grab()){
        cout << "can not grab images." << endl;
      }
      else
      {
        cout << "grabbing images."<< endl;
      }
      return NULL;
    }

int main()
{
    pthread_t grabbing;

    AprilTAG* reforeges = new AprilTAG();
    reforeges->setup();
    reforeges->setupVideo();
    pthread_create(&grabbing, NULL, grabpictures,  (void*)reforeges);
    pthread_join(grabbing, NULL);
    reforeges->loop();
    return 0;
}
