/**
   @file ConfigManager.h
   @brief
   @author Manuel Hervas
   @date 07/2014
*/
#ifndef _ARGOS_CONFIGMANAGER_H
#define _ARGOS_CONFIGMANAGER_H
#include <opencv2/opencv.hpp>

using namespace std;

class ConfigManager{

  public:

    static void loadConfiguration(const string &filename);


    static const cv::Size& getPaperSize(){
      return paperSize;
    }

    static const string& getCameraCalibration(){
      return cameraCalibrationFile;
    }

    //static const string& getProjectorCalibration(){
    //  return projectorCalibrationFile;
    //}

    //static const string& getExtrinsics(){
    //  return extrinsicsParametersFile;
    //}

    //static const vector<std::pair<int, string> >& getScriptsList(){
    //  return scriptsList;
    //}

    //static const vector<string>& getDescriptorsList(){
    //  return descriptorsList;
    //}

    //static const vector<string>& getDescriptionsList(){
    //  return descriptionsList;
    //}

    //static const bool& getOutputDisplay(){
    //  return calculateScreenExtrinsics;
    //}

  private:

    static cv::Size paperSize;

    static string cameraCalibrationFile;
    //static string projectorCalibrationFile;
    //static string extrinsicsParametersFile;

    //static vector<std::pair<int, string> > scriptsList;
    //static vector<string> descriptorsList;
    //static vector<string> descriptionsList;

    //static bool calculateScreenExtrinsics;

  };

#endif
