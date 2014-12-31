#include "CameraModel.h"
#include <iostream>

/* ---------------- Intrinsics ------------------------------*/
  Intrinsics::Intrinsics(){
    cameraMatrix = cv::Mat::eye(3,3,CV_32FC1);
    imageSize = cv::Size(-1,-1);
    sensorSize = cv::Size(0,0);
  }

  Intrinsics::Intrinsics(cv::Mat cameraMatrix, cv::Size imageSize){
    cameraMatrix.copyTo(this->cameraMatrix);
    this->imageSize = imageSize;
    this->sensorSize = cv::Size(0,0);
    cv::calibrationMatrixValues(cameraMatrix, imageSize, sensorSize.width, sensorSize.height, fov.x, fov.y, focalLength, principalPoint, aspectRatio);
  }

  Intrinsics::Intrinsics(const Intrinsics& intrinsics){
    intrinsics.cameraMatrix.copyTo(this->cameraMatrix);
    this->imageSize = intrinsics.imageSize;
    this->sensorSize = intrinsics.sensorSize;
    this->fov = intrinsics.fov;
    this->focalLength = intrinsics.focalLength;
    this->aspectRatio = intrinsics.aspectRatio;
    this->principalPoint = intrinsics.principalPoint;
  }

  void Intrinsics::setup(cv::Mat cameraMatrix, cv::Size imageSize, cv::Size sensorSize){
    cameraMatrix.copyTo(this->cameraMatrix);
    this->imageSize = imageSize;
    this->sensorSize = sensorSize;
    cv::calibrationMatrixValues(cameraMatrix, imageSize, sensorSize.width, sensorSize.height, fov.x, fov.y, focalLength, principalPoint, aspectRatio);
  }

  void Intrinsics::setup(cv::Mat cameraMatrix, cv::Size imageSize){
    cameraMatrix.copyTo(this->cameraMatrix);
    this->imageSize = imageSize;
    this->sensorSize = cv::Size(0,0);
    cv::calibrationMatrixValues(cameraMatrix, imageSize, sensorSize.width, sensorSize.height, fov.x, fov.y, focalLength, principalPoint, aspectRatio);
  }

  void Intrinsics::setImageSize(cv::Size imageSize) {
    this->imageSize = imageSize;
  }

  cv::Mat Intrinsics::getCameraMatrix() const {
    return cameraMatrix;
  }

  cv::Size Intrinsics::getImageSize() const {
    return imageSize;
  }

  cv::Size Intrinsics::getSensorSize() const {
    return sensorSize;
  }

  cv::Point2d Intrinsics::getFov() const {
    return fov;
  }

  double Intrinsics::getFocalLength() const {
    return focalLength;
  }

  double Intrinsics::getAspectRatio() const {
    return aspectRatio;
  }

  cv::Point2d Intrinsics::getPrincipalPoint() const {
    return principalPoint;
  }

  /**Adjust the parameters to the size of the image indicated
   */
  void Intrinsics::resize(cv::Size size)throw(cv::Exception){
    if (!isValid())  throw cv::Exception(9007,"invalid object","Camera::resize",__FILE__,__LINE__);
    if (size == imageSize) return;
    //now, read the camera size
    //resize the camera parameters to fit this image size
    float AxFactor = float(size.width) / float(imageSize.width);
    float AyFactor = float(size.height) / float(imageSize.height);
    cameraMatrix.at<float>(0,0) *= AxFactor;
    cameraMatrix.at<float>(0,2) *= AxFactor;
    cameraMatrix.at<float>(1,1) *= AyFactor;
    cameraMatrix.at<float>(1,2) *= AyFactor;
  }




  /* ------------- Camera Model -----------------------------------*/


  CameraModel::CameraModel(){
    distortedIntrinsics = Intrinsics();
    undistortedIntrinsics = Intrinsics();
    distCoeffs = cv::Mat::zeros(5,1,CV_32FC1);
  }

  CameraModel::CameraModel(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Size imageSize){
    distortedIntrinsics = Intrinsics(cameraMatrix, imageSize);
    distCoeffs.copyTo(this->distCoeffs);
  }

  CameraModel::CameraModel(const CameraModel& camera){
    this->distortedIntrinsics = Intrinsics(camera.getDistortedIntrinsics());
    this->undistortedIntrinsics = Intrinsics(camera.getUndistortedIntrinsics());
    camera.getDistCoeffs().copyTo(this->distCoeffs);
  }

  void CameraModel::setIntrinsics(Intrinsics& distortedIntrinsics, cv::Mat& distortionCoefficients){
    this->distortedIntrinsics = distortedIntrinsics;
    this->distCoeffs = distortionCoefficients;
    updateUndistortion();
  }

  const Intrinsics& CameraModel::getDistortedIntrinsics() const {
    return distortedIntrinsics;
  }

  const Intrinsics& CameraModel::getUndistortedIntrinsics() const {
    return undistortedIntrinsics;
  }

  cv::Mat CameraModel::getDistCoeffs() const {
    return distCoeffs;
  }


  void CameraModel::setCalibrationsPath(const string& path) {
    calibrationsPath = path;
  }


  void CameraModel::saveFile(const string& filename) const throw(cv::Exception){
    if(!isValid())
      cout << "CameraModel::saveFile() failed, because camera does not contains valid parameters."<< endl;
    else{
      cv::FileStorage fs(filename, cv::FileStorage::WRITE);
      cv::Size imageSize = distortedIntrinsics.getImageSize();
      cv::Size sensorSize = distortedIntrinsics.getSensorSize();
      cv::Mat cameraMatrix = distortedIntrinsics.getCameraMatrix();

      fs << "cameraMatrix" << cameraMatrix;
      fs << "imageSize_width" << imageSize.width;
      fs << "imageSize_height" << imageSize.height;
      fs << "sensorSize_width" << sensorSize.width;
      fs << "sensorSize_height" << sensorSize.height;
      fs << "distCoeffs" << distCoeffs;
    }
  }

  void CameraModel::loadFile(const string& filename) throw(cv::Exception){
    cv::FileStorage fs(calibrationsPath + filename, cv::FileStorage::READ);
    cv::Size imageSize, sensorSize;
    cv::Mat cameraMatrix, cameraMatrixF;

    fs["cameraMatrix"] >> cameraMatrix;
    fs["imageSize_width"] >> imageSize.width;
    fs["imageSize_height"] >> imageSize.height;
    fs["sensorSize_width"] >> sensorSize.width;
    fs["sensorSize_height"] >> sensorSize.height;
    fs["distCoeffs"] >> distCoeffs;

    if (cameraMatrix.cols == 0 || cameraMatrix.rows == 0)
      throw cv::Exception(9007,"File :"+filename+" does not contains valid camera matrix","CameraModel::loadFile",__FILE__,__LINE__);

    if (imageSize.width == -1 || imageSize.height == 0)
      throw cv::Exception(9007,"File :"+filename+" does not contains valid camera dimensions","CameraModel::loadFile",__FILE__,__LINE__);

    if (cameraMatrix.type()!=CV_32FC1) cameraMatrix.convertTo(cameraMatrixF,CV_32FC1);
    else cameraMatrixF = cameraMatrix;

    if (distCoeffs.total() < 5)
      throw cv::Exception(9007,"File :"+filename+" does not contains valid distortion_coefficients","CameraModel::loadFile",__FILE__,__LINE__);

    distortedIntrinsics.setup(cameraMatrixF, imageSize, sensorSize);
    updateUndistortion();
  }


  cv::Point3f CameraModel::getCameraLocation(cv::Mat rotVec,cv::Mat transVec){
    cv::Mat rot3x3(3,3,CV_32FC1);
    cv::Rodrigues(rotVec, rot3x3); //Converts a rotation vector to rotation matrix

    cv::Mat m4x4 = cv::Mat::eye(4,4,CV_32FC1);
    for (int i=0;i<3;i++)
      for (int j=0;j<3;j++)
  m4x4.at<float>(i,j) = rot3x3.at<float>(i,j);

    //now, add translation information
    for (int i=0;i<3;i++)
      m4x4.at<float>(i,3) = transVec.at<float>(0,i);

    //invert the matrix
    m4x4.inv();
    return  cv::Point3f(m4x4.at<float>(0,0), m4x4.at<float>(0,1), m4x4.at<float>(0,2));

  }

  void CameraModel::glGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, float proj_matrix[16], float gnear, float gfar, bool invert)throw(cv::Exception){
    //if (cv::countNonZero(Distorsion)!=0)
    //  std::cerr<< "Camera::glGetProjectionMatrix :: The camera has distortion coefficients " <<__FILE__<<" "<<__LINE__<<endl;

    if (isValid() == false)
      throw cv::Exception(9100,"invalid camera parameters","Camera::glGetProjectionMatrix",__FILE__,__LINE__);

    //Deterime the rsized info
    cv::Mat cameraMatrix = undistortedIntrinsics.getCameraMatrix();


    float Ax=float(size.width)/float(orgImgSize.width);
    float Ay=float(size.height)/float(orgImgSize.height);
    float fx=cameraMatrix.at<float>(0,0)*Ax;
    float cx=cameraMatrix.at<float>(0,2)*Ax;
    float fy=cameraMatrix.at<float>(1,1)*Ay;
    float cy=cameraMatrix.at<float>(1,2)*Ay;
    float cparam[3][4] =
      {{fx,  0, cx, 0},
       {  0, fy, cy, 0},
       {  0,  0,  1, 0}};

    //cout <<" fx "<< fx << " cx " << cx <<" fy " << fy << " cy "<< cy<< endl;

    argConvGLcpara2(cparam, size.width, size.height, gnear, gfar, proj_matrix, invert);

  }
	
  void CameraModel::OgreGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, float proj_matrix[16], float gnear, float gfar, bool invert) throw(cv::Exception){
    float temp_matrix[16];
    (*this).glGetProjectionMatrix(orgImgSize, size, temp_matrix, gnear, gfar, invert);
    proj_matrix[0]=-temp_matrix[0];
    proj_matrix[1]=-temp_matrix[4];
    proj_matrix[2]=-temp_matrix[8];
    proj_matrix[3]=temp_matrix[12];

    proj_matrix[4]=-temp_matrix[1];
    proj_matrix[5]=-temp_matrix[5];
    proj_matrix[6]=-temp_matrix[9];
    proj_matrix[7]=temp_matrix[13];

    proj_matrix[8]=-temp_matrix[2];
    proj_matrix[9]=-temp_matrix[6];
    proj_matrix[10]=-temp_matrix[10];
    proj_matrix[11]=temp_matrix[14];

    proj_matrix[12]=-temp_matrix[3];
    proj_matrix[13]=-temp_matrix[7];
    proj_matrix[14]=-temp_matrix[11];
    proj_matrix[15]=temp_matrix[15];
}


























  /*******************
   *
   *
   *******************/
  float CameraModel::norm( float a, float b, float c ){
    return( sqrt( a*a + b*b + c*c ) );
  }

  /*******************
   *
   *
   *******************/

  float CameraModel::dot( float a1, float a2, float a3, float b1, float b2, float b3 ){
    return( a1 * b1 + a2 * b2 + a3 * b3 );
  }

  /*******************
   *
   *
   *******************/

  void CameraModel::argConvGLcpara2( float cparam[3][4], int width, int height, float gnear, float gfar, float m[16], bool invert)throw(cv::Exception){

    float   icpara[3][4];
    float   trans[3][4];
    float   p[3][3], q[4][4];
    int      i, j;

    cparam[0][2] *= -1.0;
    cparam[1][2] *= -1.0;
    cparam[2][2] *= -1.0;

    if (arParamDecompMat(cparam, icpara, trans) < 0)
      throw cv::Exception(9002,"parameter error","CameraModel::argConvGLcpara2",__FILE__,__LINE__);

    for (i=0; i<3; i++){
      for (j=0; j<3; j++){
  p[i][j] = icpara[i][j] / icpara[2][2];
      }
    }
    q[0][0] = (2.0 * p[0][0] / width);
    q[0][1] = (2.0 * p[0][1] / width);
    q[0][2] = ((2.0 * p[0][2] / width)  - 1.0);
    q[0][3] = 0.0;

    q[1][0] = 0.0;
    q[1][1] = (2.0 * p[1][1] / height);
    q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
    q[1][3] = 0.0;

    q[2][0] = 0.0;
    q[2][1] = 0.0;
    q[2][2] = (gfar + gnear)/(gfar - gnear);
    q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

    q[3][0] = 0.0;
    q[3][1] = 0.0;
    q[3][2] = 1.0;
    q[3][3] = 0.0;

    for(i=0; i<4; i++){
      for(j=0; j<3; j++){
  m[i+j*4] = q[i][0] * trans[0][j] + q[i][1] * trans[1][j]	+ q[i][2] * trans[2][j];
      }
      m[i+3*4] = q[i][0] * trans[0][3] + q[i][1] * trans[1][3] + q[i][2] * trans[2][3] + q[i][3];
    }

    if (!invert){
      m[13]=-m[13] ;
      m[1]=-m[1];
      m[5]=-m[5];
      m[9]=-m[9];
    }

  }
  /*******************
   *
   *
   *******************/

  int CameraModel::arParamDecompMat( float source[3][4], float cpara[3][4], float trans[3][4] )throw(cv::Exception){
    int       r, c;
    float    Cpara[3][4];
    float    rem1, rem2, rem3;

    if (source[2][3] >= 0){
      for (r=0; r<3; r++){
  for (c=0; c<4; c++){
    Cpara[r][c] = source[r][c];
  }
      }
    }
    else{
      for(r=0; r<3; r++){
  for (c=0; c<4; c++){
    Cpara[r][c] = -(source[r][c]);
  }
      }
    }

    for(r=0; r<3; r++){
      for (c=0; c<4; c++){
  cpara[r][c] = 0.0;
      }
    }
    cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
    trans[2][0] = Cpara[2][0] / cpara[2][2];
    trans[2][1] = Cpara[2][1] / cpara[2][2];
    trans[2][2] = Cpara[2][2] / cpara[2][2];
    trans[2][3] = Cpara[2][3] / cpara[2][2];

    cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
           Cpara[1][0], Cpara[1][1], Cpara[1][2] );
    rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
    rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
    rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
    cpara[1][1] = norm( rem1, rem2, rem3 );
    trans[1][0] = rem1 / cpara[1][1];
    trans[1][1] = rem2 / cpara[1][1];
    trans[1][2] = rem3 / cpara[1][1];

    cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
           Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
           Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
    rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
    rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
    cpara[0][0] = norm( rem1, rem2, rem3 );
    trans[0][0] = rem1 / cpara[0][0];
    trans[0][1] = rem2 / cpara[0][0];
    trans[0][2] = rem3 / cpara[0][0];

    trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
    trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
       - cpara[0][2]*trans[2][3]) / cpara[0][0];

    for(r=0; r<3; r++){
      for(c=0; c<3; c++){
  cpara[r][c] /= cpara[2][2];
      }
    }

    return 0;
  }

  void CameraModel::undistort(cv::Mat& img, int interpolationMode) {
    cv::Mat undistortBuffer;
    img.copyTo(undistortBuffer);
    undistort(undistortBuffer, img, interpolationMode);
  }



  void CameraModel::undistort(const cv::Mat& src, cv::Mat& dst, int interpolationMode) {
    cv::remap(src, dst, __undistortMapX, __undistortMapY, interpolationMode);
  }


  void CameraModel::updateUndistortion() {
    cv::Mat undistortedCameraMatrix = cv::getOptimalNewCameraMatrix(distortedIntrinsics.getCameraMatrix(), distCoeffs, distortedIntrinsics.getImageSize(), false);
    cv::initUndistortRectifyMap(distortedIntrinsics.getCameraMatrix(),distCoeffs, cv::Mat(), undistortedCameraMatrix, distortedIntrinsics.getImageSize(),
        CV_16SC2, __undistortMapX, __undistortMapY);
    undistortedIntrinsics.setup(undistortedCameraMatrix, distortedIntrinsics.getImageSize());
  }
