/**
   @file ConfigManager.cpp
   @brief
   @author Manuel Hervas
   @date 07/2014
*/
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ConfigManager.h"
//#include "Log.h"
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"

using namespace std;
using namespace cv;
	
   // static variables ---
   cv::Size ConfigManager::paperSize;
	
  string ConfigManager::cameraCalibrationFile;
  //string ConfigManager::projectorCalibrationFile;
  //string ConfigManager::extrinsicsParametersFile;

  //vector<std::pair<int, string> > ConfigManager::scriptsList;
  //vector<string> ConfigManager::descriptorsList;
  //vector<string> ConfigManager::descriptionsList;

  //bool ConfigManager::calculateScreenExtrinsics;

  // static functions ---
  void ConfigManager::loadConfiguration(const string &filename){
    rapidxml::file<> xml_file(filename.c_str());
    rapidxml::xml_document<> xml_doc;
    rapidxml::xml_node<>* xml_root;
    rapidxml::xml_node<>* calibrationNode;
    //rapidxml::xml_node<>* argosPaperNode;
    //rapidxml::xml_node<>* paperNode;
    rapidxml::xml_node<>* sizeNode;
    //rapidxml::xml_node<>* outputNode;

    xml_doc.parse<0>(xml_file.data());
    xml_root = xml_doc.first_node("config");

    // <paper_size>
    sizeNode = xml_root->first_node("paper_size");
    paperSize = cv::Size(stof(sizeNode->first_attribute("width")->value()), stof(sizeNode->first_attribute("height")->value()));

    // <calibration_files>
    calibrationNode = xml_root->first_node("calibration_files");
    cameraCalibrationFile    = calibrationNode->first_attribute("camera")->value();
    //projectorCalibrationFile = calibrationNode->first_attribute("projector")->value();
    //extrinsicsParametersFile = calibrationNode->first_attribute("extrinsics")->value();

    /* <argos_papers>
    argosPaperNode = xml_root->first_node("argos_papers");
    paperNode = argosPaperNode->first_node("paper");
    while(paperNode){
      int id = atoi(paperNode->first_attribute("id")->value());
      std::cout <<  "Loaded Id: '" << std::to_string(id) << "'";
     

      string description = paperNode->first_attribute("description")->value();
      if(description.empty()) 
	
        std::cout << "Error on description loading";
      else 
	
	std::cout << "Loaded description: '" << description << "'";

      string scriptFileName = paperNode->first_attribute("scriptFileName")->value();
      if(scriptFileName.empty()) 
        std::cout << "Error on scriptFileName loading";
	
      else 
	
        std::cout << "Loaded scriptFileName: '" << scriptFileName << "'";
      string descriptorFileName = paperNode->first_attribute("descriptorFileName")->value() ;
      if(descriptorFileName.empty()) 

	std::cout << "Error on descriptorFileName loading";
      else 
	
	std::cout << "Loaded descriptorFileName: '" << descriptorFileName << "'";
      scriptsList.push_back(std::make_pair(id, scriptFileName));
      descriptorsList.push_back(descriptorFileName);
      descriptionsList.push_back(description);

      paperNode = paperNode->next_sibling("paper");

    }
    */
    /* <output_display>
    outputNode = xml_root->first_node("output_display");
    if(string(outputNode->first_attribute("type")->value()) == "projector")
      calculateScreenExtrinsics = false;
    else
      calculateScreenExtrinsics = true;
    */

    // Other configuration
    // ...

  }

