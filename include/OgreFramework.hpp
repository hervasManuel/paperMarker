#ifndef OGRE_FRAMEWORK_HPP
#define OGRE_FRAMEWORK_HPP

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreLogManager.h>

#include <OgreRoot.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>
#include <OgreOverlay.h>
#include <OgreOverlayElement.h>
#include <OgreOverlayManager.h>

#include <OIS/OISEvents.h>
#include <OIS/OISInputManager.h>
#include <OIS/OISKeyboard.h>
#include <OIS/OISMouse.h>

class OgreFramework:
      public Ogre::Singleton<OgreFramework>, OIS::KeyListener, OIS::MouseListener, 
      public Ogre::FrameListener, public Ogre::WindowEventListener {

public:	
  OgreFramework();		
  ~OgreFramework();
  
  bool initOgre(Ogre::String wndTitle, OIS::KeyListener *keyListener = 0, OIS::MouseListener *mouseListener = 0);
  void updateOgre(double timeSinceLastFrame);
  void moveCamera();
  void getInput();
  
  bool isOgreToBeShutDown()const{return _shutDownOgre;}  
  
  //bool frameStarted(const Ogre::FrameEvent &frameEvt);
  //bool frameRenderingQueued(const Ogre::FrameEvent &frameEvt);

  void notifyWindowMetrics(Ogre::RenderWindow* _renderWindow, int left, int top, int width, int height);
  void notifyWindowClosed(Ogre::RenderWindow* _renderWindow);

  void setCommandLine(const Ogre::String &commandLine);
  void setCommandLine(int argc, char *argv[]);

  inline Ogre::Root *getOgreRoot(void) const {return _root;}
  inline Ogre::RenderWindow *getRenderWindow(void) const {return _renderWindow;}
  inline OIS::Mouse *getMouse(void) const {return _mouse;}
  inline OIS::Keyboard *getKeyboard(void) const {return _keyboard;}
  
  void windowClosed(Ogre::RenderWindow* _renderWindow);
  bool windowClosing(Ogre::RenderWindow* _renderWindow);
  void windowResized(Ogre::RenderWindow* _renderWindow);
  //void windowMoved(Ogre::RenderWindow* _renderWindow);

  bool mouseMoved(const OIS::MouseEvent &evt);
  bool mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id); 
  bool mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id);
  
  bool keyPressed(const OIS::KeyEvent &keyEventRef);
  bool keyReleased(const OIS::KeyEvent &keyEventRef);

  Ogre::Root* _root;
  Ogre::SceneManager* _sceneManager;
  Ogre::RenderWindow* _renderWindow;
  Ogre::FrameListener* _frameListener;
  Ogre::Camera* _camera;
  Ogre::Viewport* _viewport;
  Ogre::Log* _log;
  Ogre::Timer* _timer;
  
  //OIS
  OIS::InputManager* _inputManager;
  OIS::Keyboard* _keyboard;
  OIS::Mouse* _mouse;
  
private:
  OgreFramework(const OgreFramework&);
  OgreFramework& operator= (const OgreFramework&);
  
  //OgreBites::SdkTrayManager* _trayManager;
  Ogre::FrameEvent _frameEvent;
  int _numScreenShots;
  bool _shutDownOgre;
  
  Ogre::Vector3	_translateVector;
  Ogre::Real _moveSpeed; 
  Ogre::Degree _rotateSpeed; 
  float	_moveScale; 
  Ogre::Degree _rotateScale;
};

#endif 
