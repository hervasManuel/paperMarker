#include "DrawCV.h"
using namespace cv;

/*******************************************************************************************
*  SCREEN DISPLAY
********************************************************************************************/
  
  void DrawCV::draw3DAxis(cv::Mat& image, Paper& paper, CameraModel& camera){
    
    float size = paper.getPaperSize().width / 2;
    
    Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0) = 0;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = 0;

    objectPoints.at<float>(1,0) = size;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = 0;
    
    objectPoints.at<float>(2,0) = 0;
    objectPoints.at<float>(2,1) = size;
    objectPoints.at<float>(2,2) = 0;

    objectPoints.at<float>(3,0) = 0;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = size;
    
    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(), camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    cv::line(image, imagePoints[0], imagePoints[1], Scalar(0,0,255,255), 1, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[2], Scalar(0,255,0,255), 1, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[3], Scalar(255,0,0,255), 1, CV_AA);
    putText(image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255,255), 2);
    putText(image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0,255), 2);
    putText(image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0,255), 2);
  }

  
  void DrawCV::draw3DAxisInPoint(cv::Mat& image, Paper& paper, CameraModel& camera, cv::Point2f& finger){
  
    float size = paper.getPaperSize().width / 2;
    
    Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0) = finger.x;
    objectPoints.at<float>(0,1) = finger.y;
    objectPoints.at<float>(0,2) = 0;

    objectPoints.at<float>(1,0) = finger.x + size;
    objectPoints.at<float>(1,1) = finger.y;
    objectPoints.at<float>(1,2) = 0;
    
    objectPoints.at<float>(2,0) = finger.x;
    objectPoints.at<float>(2,1) = finger.y + size;
    objectPoints.at<float>(2,2) = 0;

    objectPoints.at<float>(3,0) = finger.x;
    objectPoints.at<float>(3,1) = finger.y;
    objectPoints.at<float>(3,2) = size;
    
    /*
    Mat objectPoints (4,3,CV_32FC1);
    objectPoints.at<float>(0,0) = 0;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = 0;

    objectPoints.at<float>(1,0) = size;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = 0;
    
    objectPoints.at<float>(2,0) = 0;
    objectPoints.at<float>(2,1) = size;
    objectPoints.at<float>(2,2) = 0;

    objectPoints.at<float>(3,0) = 0;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = size;
    */

    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(), camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    cv::line(image, imagePoints[0], imagePoints[1], Scalar(0,0,255,255), 1, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[2], Scalar(0,255,0,255), 1, CV_AA);
    cv::line(image, imagePoints[0], imagePoints[3], Scalar(255,0,0,255), 1, CV_AA);
    putText(image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255,255), 2);
    putText(image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0,255), 2);
    putText(image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0,255), 2);
  }
 
  void DrawCV::draw3DCube(cv::Mat& image, Paper& paper, CameraModel& camera){

    Mat objectPoints (8,3,CV_32FC1);
  
    double halfSize = paper.getPaperSize().width / 4;

    objectPoints.at<float>(0,0) = -halfSize;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = -halfSize;
    objectPoints.at<float>(1,0) = halfSize;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = -halfSize;
    objectPoints.at<float>(2,0) = halfSize;
    objectPoints.at<float>(2,1) = 0;
    objectPoints.at<float>(2,2) = halfSize;
    objectPoints.at<float>(3,0) = -halfSize;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = halfSize;
  
    objectPoints.at<float>(4,0) = -halfSize;
    objectPoints.at<float>(4,1) = 2 * halfSize;
    objectPoints.at<float>(4,2) = -halfSize;
    objectPoints.at<float>(5,0) = halfSize;
    objectPoints.at<float>(5,1) = 2 * halfSize;
    objectPoints.at<float>(5,2) = -halfSize;
    objectPoints.at<float>(6,0) = halfSize;
    objectPoints.at<float>(6,1) = 2 * halfSize;
    objectPoints.at<float>(6,2) = halfSize;
    objectPoints.at<float>(7,0) = -halfSize;
    objectPoints.at<float>(7,1) = 2 * halfSize;
    objectPoints.at<float>(7,2) = halfSize;

    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(),  camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[(i+1)%4], Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i+4], imagePoints[4+(i+1)%4], Scalar(0,0,255,255),1,CV_AA);
    
    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[i+4], Scalar(0,0,255,255),1,CV_AA);
    
  }
  
  void DrawCV::draw3DPaper(cv::Mat& image, Paper& paper, CameraModel& camera){

    Mat objectPoints (8,3,CV_32FC1);
  
    double halfSize = paper.getPaperSize().width / 6;
  
    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;
  
    objectPoints.at<float>(0,0) = -halfWidthSize;
    objectPoints.at<float>(0,1) = 0;
    objectPoints.at<float>(0,2) = -halfHeightSize;
    objectPoints.at<float>(1,0) = halfWidthSize;
    objectPoints.at<float>(1,1) = 0;
    objectPoints.at<float>(1,2) = -halfHeightSize;
    objectPoints.at<float>(2,0) = halfWidthSize;
    objectPoints.at<float>(2,1) = 0;
    objectPoints.at<float>(2,2) = halfHeightSize;
    objectPoints.at<float>(3,0) = -halfWidthSize;
    objectPoints.at<float>(3,1) = 0;
    objectPoints.at<float>(3,2) = halfHeightSize;
  
    objectPoints.at<float>(4,0) = -halfWidthSize;
    objectPoints.at<float>(4,1) = 2 * halfSize;
    objectPoints.at<float>(4,2) = -halfHeightSize;
    objectPoints.at<float>(5,0) = halfWidthSize;
    objectPoints.at<float>(5,1) = 2 * halfSize;
    objectPoints.at<float>(5,2) = -halfHeightSize;
    objectPoints.at<float>(6,0) = halfWidthSize;
    objectPoints.at<float>(6,1) = 2 * halfSize;
    objectPoints.at<float>(6,2) = halfHeightSize;
    objectPoints.at<float>(7,0) = -halfWidthSize;
    objectPoints.at<float>(7,1) = 2 * halfSize;
    objectPoints.at<float>(7,2) = halfHeightSize;

    vector<Point2f> imagePoints;
    cv::projectPoints(objectPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(),  camera.getDistCoeffs(), imagePoints);
    //draw lines of different colours
    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[(i+1)%4], Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i+4], imagePoints[4+(i+1)%4], Scalar(0,0,255,255),1,CV_AA);

    for (int i=0;i<4;i++)
      cv::line(image, imagePoints[i], imagePoints[i+4], Scalar(0,0,255,255),1,CV_AA);

  }

  void DrawCV::cameraPaper(cv::Mat& image, Paper& paper, CameraModel& camera, const string& text){
  
    double halfWidthSize  =  paper.getPaperSize().width / 2.;
    double halfHeightSize =  paper.getPaperSize().height / 2.;
  
    cv::Mat objPoints(4,3,CV_32FC1);
  
    objPoints.at<float>(0,0)=-halfWidthSize;
    objPoints.at<float>(0,1)=0;
    objPoints.at<float>(0,2)=-halfHeightSize;
    objPoints.at<float>(1,0)=-halfWidthSize;
    objPoints.at<float>(1,1)=0;
    objPoints.at<float>(1,2)=halfHeightSize;
    objPoints.at<float>(2,0)=halfWidthSize;
    objPoints.at<float>(2,1)=0;
    objPoints.at<float>(2,2)=halfHeightSize;
    objPoints.at<float>(3,0)=halfWidthSize;
    objPoints.at<float>(3,1)=0;
    objPoints.at<float>(3,2)=-halfHeightSize;

    vector<cv::Point2f> imagePoints;

    cv::projectPoints(objPoints, paper.getRotVec(), paper.getTransVec(), camera.getDistortedCamMatrix(), camera.getDistCoeffs(), imagePoints);
  
    vector<cv::Point> imageIntPoints;
    for (size_t i=0; i<imagePoints.size(); i++)
      imageIntPoints.push_back(imagePoints[i]);

  
    if(imagePoints.size() <= 0) return;
    //drawContour(image, imagePoints, Scalar(255,0,255));
    fillConvexPoly(image, &imageIntPoints[0], 4 , Scalar(255,0,255), 8, 0);
 
    //determine the centroid
    cv::Point centroid(0,0);
    for (int i=0; i<4; i++){
      centroid.x+=imagePoints[i].x;
      centroid.y+=imagePoints[i].y;
    }
    centroid.x/= 4.;
    centroid.y/= 4.;
    putText(image, text, centroid ,FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,255),2);
  

    // for(size_t i = 0; i < imagePoints.size(); i++)
    //	  cv::circle(image, imagePoints[i], 10,  CV_RGB(255,255,255), -1, 8);
  
  }

  void DrawCV::drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color){  
  
    int size = 20;
    int thickness = 4;

    cv::Point lTemp1;
    cv::Point lTemp2;
  
    cv::Point lTemp3;
    cv::Point lTemp4;

  
    for(size_t i = 0; i<contour.size(); i++){
      LineIterator iterator(in,contour[i],contour[(i+1)%contour.size()]);
      for(int j = 0; j < iterator.count; j++, iterator++){
	if ( j == 0 )   
	  lTemp1 =  iterator.pos();
      
	if ( j == size) 
	  lTemp2 =  iterator.pos();
      
	if ( j == iterator.count - size) 
	  lTemp3 =  iterator.pos();
      
	if ( j == (iterator.count - 1)) 
	  lTemp4 =  iterator.pos();
      }
      line(in,lTemp1,lTemp2,color,thickness,8);
      line(in,lTemp3,lTemp4,color,thickness,8);
    }
  }


  /* Draw Dotted contour
     void DrawCV::drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color){  
  
     int lenghOfDots = 40;
     int thickness = 2;

     cv::Point lTemp1;
     cv::Point lTemp2;
  
     for(unsigned int i = 0; i<contour.size(); i++){
     LineIterator iterator(in,contour[i],contour[(i+1)%contour.size()]);

     for(int j = 0; j < iterator.count; j++, iterator++){
     if ( j%lenghOfDots == 0 )  {
     lTemp1 =  iterator.pos();
     //	cout << "Point1 "<< j << iterator.pos() << endl;
     }
     if ( j%lenghOfDots == lenghOfDots/2) { 
     //cout << "Point2 " << j << iterator.pos() << endl;
     lTemp2 =  iterator.pos();
     //cout << "draw line" << endl;
     line(in,lTemp1,lTemp2,color,thickness,8);
     }
     }
     }
     }
  */



  /*
    void DrawCV::drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color){  
    for ( unsigned int i=0;i<contour.size();i++ )
    cv::line (in,contour[i],contour[(i+1)%contour.size()],color,3);
    }
  */



