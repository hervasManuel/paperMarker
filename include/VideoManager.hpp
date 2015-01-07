#include <Ogre.h>
#include <iostream>
#include <opencv2/opencv.hpp>

//#include "opencv2/videoio.hpp"
//#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

class VideoManager{
  
private:
  //void ReleaseCapture();
  //CvCapture* _capture;
  cv::VideoCapture _capture;
  //IplImage* _frameIpl;
  cv::Mat _frameMat;
  
public:
  /* Constructor */
  VideoManager(int device, int w, int h);
  /* Destructor */
  ~VideoManager();
  
  void UpdateFrame();
  //IplImage* getCurrentFrameIpl();
  cv::Mat& getCurrentFrameMat();
};
