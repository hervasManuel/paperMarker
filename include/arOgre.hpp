#ifndef OGRE_DEMO_HPP
#define OGRE_DEMO_HPP

#include "OgreFramework.hpp"
//#include "opencv2/opencv.hpp"
#include "VideoManager.hpp"
//#include "arDetector.hpp"
//#include "Camera3D.hpp"
//#include "marker.hpp"

// OpenCV library
#include <opencv2/opencv.hpp>
// Detection stuff
#include "PaperDetector.h"
#include "Paper.h"
#include "CameraModel.h"
#include "DrawCV.h"
#include "ConfigManager.h"

// Singleton
//#include "Singleton.h"

#define MAX_OBJECTS 64

using namespace cv;

class arOgre : public OIS::KeyListener{
public:
  arOgre();
  ~arOgre();
  
  void start();
  
  bool keyPressed(const OIS::KeyEvent &keyEventRef);
  bool keyReleased(const OIS::KeyEvent &keyEventRef);
  
private:
  void createBackground(int cols, int rows);
  void runLoop();
  void DrawCurrentFrame(cv::Mat &frameMat);
  
  Ogre::Entity* ogreEntity[MAX_OBJECTS];
  Ogre::SceneNode* ogreNode[MAX_OBJECTS];
  //const float scale = 0.00675f;
  float scale;
  Ogre::AnimationState *baseAnim[MAX_OBJECTS], *topAnim[MAX_OBJECTS];

  VideoManager* _videoManager;
   
  //arToolkit::arDetector aDetector;    // ar markers detector
  //float markerSize;           // Marker size
  bool _shutdown;

  cv::Size paperSize;           ///<  Paper size in cm
  vector<Paper> paperList;      ///<  Vector of detected papers
  //vector<arToolkit::Marker> detectMarkers;
 
  CameraModel camera;  
  //arToolkit::Camera3D cameraParameters;
  //arToolkit::Camera3D cameraUndistort;
  //std::string path;
  
};

#endif
