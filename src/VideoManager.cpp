#include "VideoManager.hpp"

VideoManager::VideoManager(int device, int w, int h){
  _capture = cvCreateCameraCapture(device);
  cvSetCaptureProperty(_capture, CV_CAP_PROP_FRAME_WIDTH, w);
  cvSetCaptureProperty(_capture, CV_CAP_PROP_FRAME_HEIGHT, h); 
  _frameIpl = NULL; 
  _frameMat = NULL;
}

VideoManager::~VideoManager(){
  cvReleaseCapture(&_capture); 
  delete _frameIpl; 
  delete _frameMat;
}
// ================================================================
// UpdateFrame: Actualiza los punteros de frame Ipl y frame Mat
void VideoManager::UpdateFrame(){
  _frameIpl = cvQueryFrame(_capture);
  _frameMat = new cv::Mat(_frameIpl);
}

// = IplImage* getCurrentFrameIpl =================================
IplImage* VideoManager::getCurrentFrameIpl(){ return _frameIpl; }

// = IplImage* getCurrentFrameMat =================================
cv::Mat* VideoManager::getCurrentFrameMat(){ return _frameMat; }

