#ifndef _argosToolkit_DrawUtils_H_
#define _argosToolkit_DrawUtils_H_

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include "Paper.h"
#include "CameraModel.h"

/** 
   * A set of functions to draw in opencv images
   */
  class DrawCV{
    
  public:
    /********************************************************************************************
     *  SCREEN DISPLAY
     ********************************************************************************************/
    
    /** 
     * Draws 3D coordinate axis in center of paper
     */
    static void draw3DAxis(cv::Mat& image, Paper& p, CameraModel& camera);

    static void draw3DAxisInPoint(cv::Mat& image, Paper& paper, CameraModel& camera, cv::Point2f& finger);

    /** 
     * Draws 3D cube in center of paper
     */
    static void draw3DCube(cv::Mat& image, Paper& p, CameraModel& camera);
    /** 
     *  Draws 3D cube overlaping the paper
     */
    static void draw3DPaper(cv::Mat& image, Paper& paper, CameraModel& camera);
    
    static void cameraPaper(cv::Mat& image, Paper& paper, CameraModel& camera, const string& text);
  private:
    /** 
     * Draws contour of paper
     */
    static void drawContour(cv::Mat &in, vector<cv::Point2f>& contour, cv::Scalar color);
  };

#endif

