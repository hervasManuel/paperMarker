/**
   @file CameraModel.h
   @brief Defines the mathematical model of a camera
   @author Manuel Hervas
   @date 06/2014
*/

#ifndef _ARGOS_CAMERAMODEL_H
#define _ARGOS_CAMERAMODEL_H

#include <opencv2/opencv.hpp>

using namespace std;

  /**
   * The intrinsic parameters defines the internal geometry and optics of the camera.
   *
   */
  class Intrinsics{
  public:

    /**
     * Default constructor
     */
    Intrinsics();

    /**
     * Constructor
     * @param cameraMatrix 3x3 intrinsics matrix parameters (fx 0 cx, 0 fy cy, 0 0 1)
     * @param imageSize Image dimensions
     */
    Intrinsics(cv::Mat cameraMatrix, cv::Size imageSize);
    /**
     * Constructor
     * @param intrinsics Use the instrinsics parameters to create the new object
     */
    Intrinsics(const Intrinsics& intrinsics);

    /**
     * Configures a Intrinsics object
     * @param cameraMatrix 3x3 intrinsics matrix parameters (fx 0 cx, 0 fy cy, 0 0 1)
     * @param imageSize Image dimensions
     */
    void setup(cv::Mat cameraMatrix, cv::Size imageSize);

    /**
     * Configures a Intrinsics object
     * @param cameraMatrix 3x3 intrinsics matrix parameters (fx 0 cx, 0 fy cy, 0 0 1)
     * @param imageSize Image dimensions
     * @param sensorSize Sensor dimensions
     */
    void setup(cv::Mat cameraMatrix, cv::Size imageSize, cv::Size sensorSize);

    /**
     * Sets image dimensions
     * @param imgSize Image dimensions
     */
    void setImageSize(cv::Size imgSize);

    /**
     * Gets camera matrix
     * @return 3x3 cv::Mat matrix
     */
    cv::Mat getCameraMatrix() const;

    /**
     * Gets image dimensions
     * @return cv::Size(width,height)
     */
    cv::Size getImageSize() const;

    /**
     * Gets sensor dimensions
     * @return cv::Size(width,height)
     */
    cv::Size getSensorSize() const;

    /**
     * Gets field of view
     * @return cv::Point2d(x,y)
     */
    cv::Point2d getFov() const;

    /**
     * Gets focal length
     * @return double
     */
    double getFocalLength() const;

    /**
     * Gets aspect ratio
     */
    double getAspectRatio() const;

    /**
     * Gets principal point (center)
     */
    cv::Point2d getPrincipalPoint() const;

    /**
     * Sets new image dimensions
     * @param size Image dimensions
     */
    void resize(cv::Size size)throw(cv::Exception);

    /**
     * Checks if the object has valid parameters
     */
    bool isValid() const {
      return cameraMatrix.rows != 0 && cameraMatrix.cols != 0  && imageSize.width != -1 && imageSize.height != -1;
    }

  private:
    cv::Mat cameraMatrix;             ///< 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
    cv::Size imageSize;               ///< Size of the image
    cv::Size sensorSize;              ///< Size of the image
    cv::Point2d fov;                  ///< Field of view
    double focalLength;               ///< Focal length
    double aspectRatio;               ///< Aspect Ratio
    cv::Point2d principalPoint;       ///< Principal point (center)
  };


  /**
   * Defines the mathematical model of a camera
   */
  class CameraModel{

  public:
    /**
     * Default Constructor
     */
    CameraModel();

    /**
     * Creates the object from the info passed
     * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
     * @param distCoeffs 4x1 matrix (k1,k2,p1,p2)
     * @param imageSize Image size
     */
    CameraModel(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Size imageSize);

    /**
     * Copy constructor
     * @param camera CameraModel to copy
     */
    CameraModel(const CameraModel& camera);

    /**
     * Checks whether this object is valid
     */
    bool isValid() const {
      return distortedIntrinsics.isValid() && undistortedIntrinsics.isValid() && distCoeffs.rows !=0 && distCoeffs.cols != 0;
    }

    /**
     * Gets distortion coeficients of camera
     */
    cv::Mat getDistCoeffs() const;

    /**
     * Configures distorted intrinsics parameters
     * @param distortedIntrinsics Intrinsics parameters with distortion
     * @param distortionCoefficients Distortion Coefficients
     */
    void setIntrinsics(Intrinsics& distortedIntrinsics, cv::Mat& distortionCoefficients);

    /**
     * Gets distorted intrinsics parameters
     */
    const Intrinsics& getDistortedIntrinsics() const;

    /**
     * Gets undistorted intrinsics parameters (for OpenGL)
     */
    const Intrinsics& getUndistortedIntrinsics() const;

    /**
     * Gets distorted camera matrix
     */
    cv::Mat getDistortedCamMatrix() const{
      return distortedIntrinsics.getCameraMatrix();
    }

    /**
     * Gets undistorted camera matrix
     */
    cv::Mat getUndistortedCamMatrix() const{
      return undistortedIntrinsics.getCameraMatrix();
    }

    /**
     * Returns the location of the camera in the reference system given by the rotation and translation vectors passed
     */
    static cv::Point3f getCameraLocation(cv::Mat Rvec,cv::Mat Tvec);

    /**
     * Given the intrinsic camera parameters returns the GL_PROJECTION matrix for OpenGL.
     * Please NOTE that when using OpenGL, it is assumed no camera distorsion!
     *
     * @param orgImgSize size of the original image
     * @param size of the image/window where to render (can be different from the real camera image). Please not that it must be related to CamMatrix
     * @param proj_matrix output projection matrix to give to opengl
     * @param gnear,gfar: visible rendering range
     * @param invert: indicates if the output projection matrix has to yield a horizontally inverted image because image data has not been stored in the order of glDrawPixels: bottom-to-top.
     */
    void glGetProjectionMatrix( cv::Size orgImgSize, cv::Size size,float proj_matrix[16],float gnear,float gfar,bool invert=false) throw (cv::Exception);

    void OgreGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, float proj_matrix[16], float gnear, float gfar, bool invert) throw(cv::Exception);


     /**
     * Set the directory where calibration config can be found
     * @param path The directory path
     */
    void setCalibrationsPath(const string& path);

    /**
     * Saves this to a file
     */
    void saveFile(const string& filename) const throw(cv::Exception);

    /**
     * Reads from a XML/YAML file generated with the camera model
     */
    void loadFile(const string& filename) throw(cv::Exception);



  private:
    Intrinsics distortedIntrinsics;   ///< Principal intrinsics parameters
    Intrinsics undistortedIntrinsics; ///< Undistorted intrinsics parameters for OPENGL
    cv::Mat distCoeffs;               ///< 4x1 matrix distortion coefficients (k1,k2,p1,p2)

    cv::Mat __undistortMapX;          ///< Image transformation map fx(x,y)
    cv::Mat __undistortMapY;          ///< Image transformation map fy(x,y)

    string calibrationsPath;          ///< The directory where calibration config can be found

    /**
     * Generates OpenGL projection matrix based on real camera parameter and user-specified far/near clip plane.
     */
    void argConvGLcpara2( float cparam[3][4], int width, int height, float gnear, float gfar, float m[16], bool invert)throw(cv::Exception);

    /**
     * If the specified cparam includes translation and rotation components, this function divides it into perspective projection
     * component and translation/rotation components.
     */
    int arParamDecompMat( float source[3][4], float cpara[3][4], float trans[3][4] )throw(cv::Exception);

    /**
     * Calculate the euclidean norm of (a,b,c) vector
     */
    float norm( float a, float b, float c );

    /**
     * Returns dot product of (a1,a2,a3) and (b1,b2,b3) vectors
     */
    float dot( float a1, float a2, float a3, float b1, float b2, float b3 );

    /**
     * Transforms an image to compensate for lens distortion.
     * @param img
     */
    void undistort(cv::Mat& img, int interpolationMode = cv::INTER_NEAREST);

    /**
     * Applies a generic geometrical transformation to an image.
     * @param src Source image
     * @param dst Destination image. It has the same size as map1 and the same type as src
     * @param interpolationMode Interpolation method (a nearest-neighbor interpolation by default)
     */
    void undistort(const cv::Mat& src, cv::Mat& dst, int interpolationMode = cv::INTER_NEAREST);

    /**
     * Computes and Update the undistortion and rectification transformation map.
     */
    void updateUndistortion();
  };
#endif
