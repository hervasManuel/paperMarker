#include <Ogre.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

class VideoManager{

private:
  void ReleaseCapture();
  CvCapture* _capture;
  IplImage* _frameIpl;
  cv::Mat* _frameMat;

public:
  /* Constructor */
  VideoManager(int device, int w, int h);
  /* Destructor */
  ~VideoManager();
  
  void UpdateFrame();
  IplImage* getCurrentFrameIpl();
  cv::Mat* getCurrentFrameMat();
};
