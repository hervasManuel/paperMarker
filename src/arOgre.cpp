#include "arOgre.hpp"
#include <OgreLight.h>
#include <OgreWindowEventUtilities.h>

using namespace Ogre;

arOgre::arOgre(){
  //markerSize = 0.14;           // Marker size
  //path = "calib.yml";  
  //cameraParameters.readFromXMLFile(path);
  //scale = 0.00675f;
  scale = 0.01f;

  //- Read configuration file ----
  //Log::info("Reading configuration file... ");
  ConfigManager::loadConfiguration("config.xml");
  
  //- Camera  Parameters ----
  //Log::info("Loading camera & projector parameters... ");
  camera.setCalibrationsPath("calibrations/");
  camera.loadFile(ConfigManager::getCameraCalibration());

  // Paper detector initilization
  PaperDetector::getInstance();
  
  // Document detector initialization
  //DocumentDetector::getInstance().setImagesPath("data/templates/");
  //DocumentDetector::getInstance().configure();
  //HandDetector::getInstance();
  
}

arOgre::~arOgre(){
  delete OgreFramework::getSingletonPtr();
}

void arOgre::start(){
  new OgreFramework();
  if(!OgreFramework::getSingletonPtr()->initOgre("arOgreTest 1.0", this, 0))
    return;
  _shutdown = false;
  OgreFramework::getSingletonPtr()->_log->logMessage("Demo initialized!");
  
  if(!camera.isValid())
    OgreFramework::getSingletonPtr()->_log->logMessage("Camera parameters is not set, need to run the CameraCalibrator tool");
  

  _videoManager = new VideoManager(0, 640, 480);

  createBackground(640, 480);

  //------------- Configure Camera
  //cv::undistort(TheInputImage,TheInputImageUnd,cameraParameters.CameraMatrix,cameraParameters.Distorsion);
  
  // cameraUndistort = cameraParameters;
  //cameraUndistort.Distorsion=cv::Mat::zeros(4,1,CV_32F);
  
  OgreFramework::getSingletonPtr()->_camera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  OgreFramework::getSingletonPtr()->_camera->setNearClipDistance(0.01f);
  OgreFramework::getSingletonPtr()->_camera->setFarClipDistance(10.0f);
  
  float pMatrix[16];
  camera.OgreGetProjectionMatrix(camera.getUndistortedIntrinsics().getImageSize(),camera.getUndistortedIntrinsics().getImageSize() , pMatrix, 0.05f, 1000.0f, true); //1000.0f
   
  Ogre::Matrix4 PM(pMatrix[0], pMatrix[1], pMatrix[2] , pMatrix[3],
		   pMatrix[4], pMatrix[5], pMatrix[6] , pMatrix[7],
		   pMatrix[8], pMatrix[9], pMatrix[10], pMatrix[11],
		   pMatrix[12], pMatrix[13], pMatrix[14], pMatrix[15]);
  
  OgreFramework::getSingletonPtr()->_camera->setCustomProjectionMatrix(true, PM);
  OgreFramework::getSingletonPtr()->_camera->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);
  
  //------------- Init Ogre scene
  for(unsigned int i=0; i<MAX_OBJECTS; i++) {
    std::stringstream EN; EN << "Paper "<<i;
    ogreEntity[i] = OgreFramework::getSingletonPtr()->_sceneManager->createEntity(EN.str(), "Sinbad.mesh");
    ogreNode[i] = OgreFramework::getSingletonPtr()->_sceneManager->getRootSceneNode()->createChildSceneNode();
    ogreNode[i]->attachObject(ogreEntity[i]);
    ogreNode[i]->setScale(scale, scale, scale);
    ogreNode[i]->setVisible(false);
    
    // Init animation
    ogreEntity[i]->getSkeleton()->setBlendMode(Ogre::ANIMBLEND_CUMULATIVE);
    baseAnim[i] = ogreEntity[i]->getAnimationState("RunBase");
    //topAnim[i] = ogreEntity[i]->getAnimationState("Dance");
    baseAnim[i]->setLoop(true);
    //topAnim[i]->setLoop(true);
    baseAnim[i]->setEnabled(true);
    //topAnim[i]->setEnabled(true);
  }   
    OgreFramework::getSingletonPtr()->_log->logMessage("Initialition Ogre Scene   ..... OK");
  runLoop();
}

void arOgre::createBackground(int cols, int rows){
  /* Ogre code */
  Ogre::TexturePtr texture=Ogre::TextureManager::getSingleton().
    createManual(
		 "BackgroundTex",// name
		 Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		 Ogre::TEX_TYPE_2D,// texture type
		 cols,
		 rows,
		 0,// number of mipmaps
		 Ogre::PF_BYTE_BGRA, // pixel format
		 Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY_DISCARDABLE
		 );
  
  
  //Create background, from http://www.ogre3d.org/wiki/index.php/Displaying_2D_Backgrounds
  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().
    create("Background", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->getTechnique(0)->getPass(0)->createTextureUnitState();
  // material->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
  material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
  material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  material->getTechnique(0)->getPass(0)->getTextureUnitState(0)->setTextureName("BackgroundTex");
  
  //Create background rectangle covering the whole screen
  Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
  
  rect->setCorners(-1.0,1.0,1.0,-1.0);
  rect->setMaterial("Background");
  
  // Render the background before everything else
  rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);
  
  // Hacky, but we need to set the bounding box to something big
  rect->setBoundingBox(Ogre::AxisAlignedBox(-100000 * Ogre::Vector3::UNIT_SCALE,100000 * Ogre::Vector3::UNIT_SCALE));
  
  //Attach background to the scene
   Ogre::SceneNode* sceneNode = OgreFramework::getSingletonPtr()->_sceneManager->getRootSceneNode()->createChildSceneNode("BackgroundNode");
  sceneNode->attachObject(rect);
 
  OgreFramework::getSingletonPtr()->_log->logMessage("Background created");
}

void arOgre::runLoop(){
  OgreFramework::getSingletonPtr()->_log->logMessage("Start main loop...");
  //double timeSinceLastFrame = 0;
  //double startTime = 0;
  
  OgreFramework::getSingletonPtr()->_renderWindow->resetStatistics();
  
  while(!_shutdown && !OgreFramework::getSingletonPtr()->isOgreToBeShutDown()){
    if(OgreFramework::getSingletonPtr()->_renderWindow->isClosed())
      _shutdown = true;
    Ogre::WindowEventUtilities::messagePump();

    if(OgreFramework::getSingletonPtr()->_renderWindow->isActive()){
      //startTime = OgreFramework::getSingletonPtr()->_timer->getMillisecondsCPU();
      
      OgreFramework::getSingletonPtr()->_keyboard->capture();
      OgreFramework::getSingletonPtr()->_mouse->capture();
      
      _videoManager->UpdateFrame();
      DrawCurrentFrame(_videoManager->getCurrentFrameMat());
      
      cv::Mat currentFrame = *_videoManager->getCurrentFrameMat();
      //cv::Mat test;
      //currentFrame.copyTo(test);
      cv::Size paperSize = ConfigManager::getPaperSize();      
      //aDetector.detect(TheInputImage,detectMarkers,cameraParameters,markerSize,true);
      
      PaperDetector::getInstance().detect(currentFrame, paperList, camera, paperSize, true);

      //for(size_t i=0; i<paperList.size(); i++){ 
      //  DrawCV::draw3DAxis(test, paperList[i], camera);
      //  DrawCV::draw3DPaper(test, paperList[i], camera);
      // }
      //imshow("Test",test);
      //waitKey(1);
      
      for(unsigned int i=0; i<MAX_OBJECTS; i++) {
        if(i<paperList.size()) { 
          ogreNode[i]->setVisible(true);
          float position[3], orientation[4];
          paperList[i].OgreGetPoseParameters(position, orientation);
          ogreNode[i]->setPosition(position[0], position[1], position[2]);
          ogreNode[i]->setOrientation(orientation[0], orientation[1], orientation[2], orientation[3]);	    
          
          // Update animation and correct position
          baseAnim[i]->addTime(0.08);
          //topAnim[i]->addTime(0.08);
          
          Ogre::Real offset = ogreEntity[i]->getBoundingBox().getHalfSize().y;
          ogreNode[i]->translate(0,+offset*scale,0,Ogre::Node::TS_LOCAL);	 
        }
        else ogreNode[i]->setVisible(false);
      }
      //OgreFramework::getSingletonPtr()->updateOgre(timeSinceLastFrame);
      OgreFramework::getSingletonPtr()->_root->renderOneFrame();
      //timeSinceLastFrame = OgreFramework::getSingletonPtr()->_timer->getMillisecondsCPU() - startTime;
    }
    else{
      sleep(1);
    }
  }
  OgreFramework::getSingletonPtr()->_log->logMessage("Main loop quit");
  OgreFramework::getSingletonPtr()->_log->logMessage("Shutdown OGRE...");
}

bool arOgre::keyPressed(const OIS::KeyEvent &keyEventRef){
  OgreFramework::getSingletonPtr()->keyPressed(keyEventRef);
  if(OgreFramework::getSingletonPtr()->_keyboard->isKeyDown(OIS::KC_F)){
    //do something
  }
  return true;
}

bool arOgre::keyReleased(const OIS::KeyEvent &keyEventRef){
  OgreFramework::getSingletonPtr()->keyReleased(keyEventRef);
  
  return true;
}

void arOgre::DrawCurrentFrame(cv::Mat *frameMat){
  if(frameMat->rows==0) return;
  
  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().
    getByName("BackgroundTex",
	      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  
  Ogre::HardwarePixelBufferSharedPtr pBuffer = tex->getBuffer();
  
  pBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixelBox = pBuffer->getCurrentLock();
  Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);
  
  for(int j=0;j<frameMat->rows;j++){
    for(int i=0;i<frameMat->cols;i++) {
      int idx = ((j) * pixelBox.rowPitch + i )*4;
      pDest[idx] = frameMat->data[(j*frameMat->cols+i)*3];
      pDest[idx+1] = frameMat->data[(j*frameMat->cols+i)*3+1];
      pDest[idx+2] = frameMat->data[(j*frameMat->cols+i)*3+2];
      pDest[idx+3] = 255;
    }
  }
  pBuffer->unlock();
  
  //Ogre::Rectangle2D* rect = static_cast<Ogre::Rectangle2D*>(OgreFramework::getSingletonPtr()->_sceneManager->getRootSceneNode()->getSceneNode("BackgroundNode")->getAttachedObject(0));
}
