#include "VideoManager.hpp"

VideoManager::VideoManager(int device, int w, int h){
  _capture.set(CV_CAP_PROP_FRAME_WIDTH, w);
  _capture.set(CV_CAP_PROP_FRAME_HEIGHT, h);
  _capture.open(device);

  if(!_capture.isOpened()) {
    cout << "Failed opening the camera." << endl;
  }
}

VideoManager::~VideoManager(){
}

// ================================================================
// UpdateFrame: Actualiza las imagenes de frameMat
void VideoManager::UpdateFrame(){
  _capture.grab();
  _capture.retrieve(_frameMat);
}

cv::Mat& VideoManager::getCurrentFrameMat(){ return _frameMat; }

