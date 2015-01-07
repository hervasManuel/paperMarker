#ifndef _ARGOS_PAPER_H
#define _ARGOS_PAPER_H

#include "CameraModel.h"
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;



/**
 *\brief This class represents a sheet of paper. It is a vector of the fours corners of the paper
 *
 */

  class Paper: public vector<cv::Point2f> {
  public:
    /**
     * Default Constructor
     */
    Paper();

    /**
     *
     */
    Paper(cv::Size size);
    /**
     *
     */
    Paper(const Paper &P);
    /**
     *
     */
    Paper(const vector<cv::Point2f> &corners);

    /**
     * Default Destructor
     */
    ~Paper() {}


    cv::Size getPaperSize() const{
      return paperSize;
    }

    cv::Mat getRotVec() const{
      return rotVec;
    }

    cv::Mat getTransVec() const{
      return transVec;
    }

    void setRotVec(cv::Mat &rot) const{
      rot.copyTo(rotVec);
    }

    void setTransVec(cv::Mat &trans) const{
      trans.copyTo(transVec);
    }

    void setPaperSize(cv::Size size){
      paperSize = size;
    }

    int getId() const{
      return id;
    }

    void setId(int number){
      id = number;
    }

    cv::Point2f&  getFingerPoint(){
      return fingerPoint;
    }

    void setFingerPoint(cv::Point2f pt){
      fingerPoint = pt;
    }

    /**
     * Indicates if this object is valid
     */
    bool isValid() const{
      return size() == 4;}

    /**
     *Draws this paper in the input image
     */
    void draw(cv::Mat &in, cv::Scalar color, int lineWidth=1, bool writeId=true)const;

    /**
     * Returns the centroid of the paper
     */
    cv::Point2f getCenter()const;

    /**
     * Returns the perimeter of the paper
     */
    float getPerimeter()const;

    /**
     * Returns the area
     */
    float getArea()const;

    /**
     */
    //friend bool operator<(const Paper &M1, const Paper &M2){
    //  return M1.id < M2.id;
    //}
    void glGetModelViewMatrix(float modelview_matrix[16]) throw(cv::Exception);
    void OgreGetPoseParameters(float position[3], float orientation[4]) throw(cv::Exception);
    //void calculateExtrinsics(cv::Size paperSizeMeters, CameraModel& camera, bool setYPerperdicular, bool screenExtrinsics) throw(cv::Exception);
    void calculateScreenExtrinsics(cv::Size paperSizeMeters, CameraModel& camera, bool setYPerperdicular) throw(cv::Exception);
    //void calculateProjectorExtrinsics(cv::Size paperSizeMeters, CameraProjectorSystem& cameraProjector, bool setYPerperdicular) throw(cv::Exception);
    void rotateXAxis(Mat &rotation);
    void rotateYAxis(Mat &rotation);
    void rotateYAxis2(Mat &rotation);
    void rotateZAxis(Mat &rotation);
    //void calculateExtrinsics(cv::Size paperSizeMeters, cv::Mat camMatrix, cv::Mat distCoeff, bool setYPerperdicular) throw(cv::Exception);
    /**
     */

    friend ostream& operator<<(ostream &str, const Paper &P){
      for (int i=0; i<4; i++)
        str<<"("<<P[i].x<< ","<<P[i].y<<") ";

      str<<"Rxyz=";
      for (int i=0; i<3; i++)
        str<<P.getRotVec().ptr<float>(0)[i]<<" ";

      str<<"Txyz=";
      for (int i=0;i<3;i++)
        str<<P.getTransVec().ptr<float>(0)[i]<<" ";

      return str;
    }

  protected:
    int id;                                 // id of document detected
    cv::Size paperSize;                     // size of the paper sides in meters
    cv::Mat rotVec;                         // rotation matrix (Object->Projector)
    cv::Mat transVec;                       // translation matrix (Object->Projector)
    cv::Point2f fingerPoint;
  };

#endif
