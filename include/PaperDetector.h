/**
   @file PaperDetector.h
   @brief
   @author Manuel Hervas
   @date 06/2014
*/
#ifndef _ARGOS_PAPERDETECTOR_H
#define _ARGOS_PAPERDETECTOR_H
#include "Paper.h"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <iostream>
#include <deque>

// Singleton
#include "Singleton.h"

using namespace std;


  /**
   * Represent a candidate to be a sheet of paper
   */
  class PaperCandidate:public Paper{
  public:
    vector<cv::Point> contour;  /// All the points of its contour
    int idx;                    /// Index position in the global contour list

    /**
     * Default constructor
     */
    PaperCandidate(){}

    PaperCandidate(const Paper &P): Paper(P){}

    PaperCandidate(const  PaperCandidate &P): Paper(P){
      contour = P.contour;
      idx = P.idx;
    }

    PaperCandidate(cv::Size S): Paper(S){}

  };

  /**
   * Main class for paper detection
   */
  class PaperDetector: public Singleton<PaperDetector> {

  public:

    /**
     * Default constructor
     */
    PaperDetector();

    /**
     * Default destructor
     */
    ~PaperDetector();

    /**
     * Detects the sheet of papers in the image passed
     * With the information about the camera parameters and the size of the marker, the extrinsics of the paper are detected
     *
     * @param currentFrame input image
     * @param detectedPapers output vector with papers detected
     * @param camProjector intrinsic parameters of camera and projector system
     * @param paperSizeMeters paper dimensions
     * @param setYPerpendicular set Y axis perpendicular to paper
     * @param screenExtrinsics calculate extrinsics parameters for display in screen or projector
     */

    void detect (const cv::Mat& currentFrame, std::vector<Paper>& detectedPapers, CameraModel& camera,
     cv::Size paperSizeMeters, bool setYPerpendicular = false) throw (cv::Exception);

    /**
     *  Sets the type of thresholding methods available
     */
    enum ThresholdMethods {FIXED_THRES,ADPT_THRES,CANNY};

    /**
     * Sets the threshold method
     */
    void setThresholdMethod(ThresholdMethods m) {
      thresMethod = m;
    }

    /**
     * Returns the current threshold method
     */
    ThresholdMethods getThresholdMethod()const {
      return thresMethod;
    }

    /**
     * Sets the parameters of the threshold method
     * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
     *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
     *   @param param2: The constant subtracted from the mean or weighted mean
     */
    void setThresholdParams(double param1,double param2) {
      thresParam1 = param1;
      thresParam2 = param2;
    }

    /**
     * Sets the parameters of the threshold method
     * We are currently using the Adptive threshold ee opencv doc of adaptiveThreshold for more info
     *   @param param1: blockSize of the pixel neighborhood that is used to calculate a threshold value for the pixel
     *   @param param2: The constant subtracted from the mean or weighted mean
     */
    void getThresholdParams(double &param1,double &param2)const {
      param1 = thresParam1;
      param2 = thresParam2;
    }


    /**
     * Returns a reference to the internal image thresholded. It is for visualization purposes and to adjust manually
     * the parameters
     */
    const cv::Mat  & getThresFrame(){
      return outThres;
    }

    const cv::Mat &  getDebugFrame(){
      return debug;
    }

    /**
     * Specifies the min and max sizes of the markers as a fraction of the image size. By size we mean the maximum
     * of cols and rows.
     * @param min size of the contour to consider a possible marker as valid (0,1]
     * @param max size of the contour to consider a possible marker as valid [0,1)
     */
    void setMinMaxSize(float min=0.03,float max=0.5)throw(cv::Exception);

    void setContourArea(float area=5000.0)throw(cv::Exception);

    /**
     * Reads the min and max sizes employed
     * @param min output size of the contour to consider a possible marker as valid (0,1]
     * @param max output size of the contour to consider a possible marker as valid [0,1)
     */
    void getMinMaxSize(float &min,float &max){min=minContourValue; max=maxContourValue;}

    void getContourArea(float &area){area=contourAreaValue;}

    /**
     * Thesholds the passed image with the specified method.
     */
    void thresHold(int method, const Mat &grey, Mat &out, double param1, double param2) throw (cv::Exception);

    /**
     * Detection of candidates to be markers, i.e., rectangles.
     * This function returns in candidates all the rectangles found in a thresolded image
     */
    void detectRectangles(const cv::Mat &thresImg, vector<std::vector<cv::Point2f> > & candidates);

    /**
     * Refine PaperCandidate Corner using LINES method
     * @param candidate candidate to refine corners
     */
    void refineCandidateLines(PaperCandidate &candidate);

    cv::Mat calculateAverage(std::deque<cv::Mat> &matrix);





  private:
    // Images
    cv::Mat currentFrame;                     /// current frame
    cv::Mat greyFrame;                        /// grey frame
    cv::Mat debug;                            /// debug frame
    cv::Mat outThres,thresholdContours;       /// threshold frame
    vector<float> historicWeights;            //
    std::deque<cv::Mat> historicRot;
    std::deque<cv::Mat> historicTrans;


    // Windows
    string configWindow;
    // ThresHold parameters
    int typeThres;
    int max_Type;
    int max_Level;
    // Trackbar Threshold
    string trackbarType;
    string trackbarParm1;
    string trackbarParm2;

    //----------------------- Filters------------------------------------------------------------
    // Threshold -------------------------------------------------------------------------------
    ThresholdMethods thresMethod;             /// Current threshold method
    int thresParam1, thresParam2;          /// Threshold parameters
  
    // ----------------------- Detection methods------------------------------------------------
    // Find Rectangles
    //vector of candidates to be markers. This is a vector with a set of rectangles that have no valid id
    //vector<std::vector<cv::Point2f> > _candidates;
    
    // Convex Hull Contours
    float minContourValue, maxContourValue;   ///Minimum and maximum size of a contour lenght
    float contourAreaValue;
    float minContourAreaValue,  maxContourAreaValue;




    /**
     * Detection of candidates to be sheets of paper, i.e., rectangles in contours
     * This function returns in PaperCandidates all the rectangles found in a thresolded image
     */
    void findRectangles(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates);
    
    /**
     * Detection of candidates to be sheets of paper through Convex Hull contours
     * This function returns in PaperCandidates all the rectangles found in a thresolded image
     */
    void convexHullContours(vector<vector<cv::Point> > &contours, vector<PaperCandidate> &PaperCandidates);
    

    void houghLineTransform(cv::Mat& image, vector<PaperCandidate> &PaperCandidates);
    
    
    
     
    //-Utils-----------------------------------------------------------------------------------------

    /**
     */
    bool isInto(cv::Mat &contour,vector<cv::Point2f> &b);

    /**
     */
    int perimeter(vector<cv::Point2f> &a);

     /**
     */
    double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);

    /**
     */
    float length(cv::Vec4i v);
    
    /**
     */
    float angleBetween2Lines(cv::Vec4i line1, cv::Vec4i line2);
    
    /**
     */
    cv::Point computeIntersect(cv::Vec4i a, cv::Vec4i b);

    /**
     * Given a vector vinout with elements and a boolean vector indicating the elements from it to remove,
     * this function remove the elements
     * @param vinout input vector
     * @param toRemove boolean vector indicating which elements should be eliminated
     */
    template<typename T>
    void removeElements(vector<T> &vinout, const vector<bool> &toRemove){
      //remove the invalid ones by setting the valid in the positions left by the invalids
      size_t indexValid=0;
      for (size_t i=0;i<toRemove.size();i++) {
        if (!toRemove[i]) {
          if (indexValid!=i) vinout[indexValid]=vinout[i];
          indexValid++;
        }
      }
      vinout.resize(indexValid);
    }

    //- Graphical debug ---------------------------------------------
    void drawApproxCurve(cv::Mat &in, vector<cv::Point> &contour, cv::Scalar color);
    void drawApproxCurve(cv::Mat &in, vector<cv::Point2f> &contour, cv::Scalar color);
    void drawContour(cv::Mat &in, vector<cv::Point> &contour, cv::Scalar color);
    void drawAllContours(cv::Mat input, vector<vector<cv::Point> > &contours);
    void draw(cv::Mat out, const vector<Paper> &papers);


    
    //cv::Point computeIntersect(Vec2i line1, Vec2i line2);
    //bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);
    //vector<Point> lineToPointPair(Vec2i line);
    //void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center);
    //bool warp(cv::Mat &in, cv::Mat &out, cv::Size size, vector<Point2f> points) throw ( cv::Exception );    
  };

#endif
