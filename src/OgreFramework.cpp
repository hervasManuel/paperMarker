/**
 * @author Manuel Hervás Ortega
 * @file OgreFramework.cpp
 * @date 24-09-2012
 */

#include "OgreFramework.hpp"

#include <iostream>
#include <string>
#include <utility>

using std::cout;
using std::endl;
using std::string;
using std::ostringstream;

using namespace Ogre;

template<> OgreFramework* Ogre::Singleton<OgreFramework>::msSingleton = NULL;

OgreFramework::OgreFramework(){
  _moveSpeed = 0.1f;
  _rotateSpeed = 0.3f;
  
  _shutDownOgre = false;
  _numScreenShots = 0;
  
  _root = NULL;
  _sceneManager = NULL ;
  _renderWindow  = NULL;
  _camera = NULL;
  _viewport = NULL;
  _log = NULL;
  _timer = NULL;

  _inputManager = NULL;
  _keyboard = NULL;
  _mouse = NULL;
  
  //_trayManager = NULL;
  _frameEvent = Ogre::FrameEvent();
}

bool OgreFramework::initOgre(Ogre::String wndTitle, OIS::KeyListener *keyListener, OIS::MouseListener *mouseListener){
  
  // Creamos el objeto principal de Ogre, la raíz
  //---------------------------
  _root = new Ogre::Root();
  
  //Generamos fichero de LOG
  //------------------------
  _log = Ogre::LogManager::getSingleton().createLog("Ogre.log", true, true, false);
  _log->setDebugOutputEnabled(true);
  
  // Diálogo para configurar Ogre
  //-------------------------------
  if(!_root->showConfigDialog())
    return false;
    
  // Creamos la ventana
  //------------------------------------
  _renderWindow = _root->initialise(true, wndTitle);
    
  // Creamos un SceneManager genérico
  //------------------------------------
  _sceneManager = _root->createSceneManager(ST_GENERIC, "SceneManager");
  // Luz ambiente
  _sceneManager->setAmbientLight(Ogre::ColourValue(0.7f, 0.7f, 0.7f));
  // Elegimos el tipo de sombra
  _sceneManager->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
  
  // Creamos y configuramos la cámara
  //-------------------------------------
  _camera = _sceneManager->createCamera("Camera");
  _camera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
  //_camera->setPosition(Vector3(15, 15, 15));
  //_camera->lookAt(Vector3(0, 0, 0));
  //_camera->setNearClipDistance(0.01);
  //_camera->setFarClipDistance(10);
  //_camera->setCustomProjectionMatrix(true, PM);
  //_camera->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);
  //_camera->setFOVy(Degree(40)); //FOVy camera Ogre = 40°
  
  // Añadimos un Viewport
  //-------------------------------------
  _viewport = _renderWindow->addViewport(_camera);
  _viewport->setBackgroundColour(ColourValue(0.8f, 0.7f, 0.6f, 1.0f));
  
  // Adaptamos la proporción de la cámara para que encaje con el viewport
  //-------------------------------------
  _camera->setAspectRatio(Real(_viewport->getActualWidth()) / Real(_viewport->getActualHeight()));
  _viewport->setCamera(_camera);
  
  
  // Configuramos el listener OIS
  //-----------------------------------------
  unsigned long hWnd = 0;
  OIS::ParamList paramList;
  _renderWindow->getCustomAttribute("WINDOW", &hWnd);
  
  paramList.insert(OIS::ParamList::value_type("WINDOW", Ogre::StringConverter::toString(hWnd)));
  
  // Creamos el gestor de entrada (con teclado y ratón)
  _inputManager = OIS::InputManager::createInputSystem(paramList);
  
  _keyboard = static_cast<OIS::Keyboard*>(_inputManager->createInputObject(OIS::OISKeyboard, true));
  _mouse = static_cast<OIS::Mouse*>(_inputManager->createInputObject(OIS::OISMouse, true));
  
  //_mouse->getMouseState().height = _renderWindow->getHeight();
  //_mouse->getMouseState().width  = _renderWindow->getWidth();
  
  
  // Pedimos que nos envíen eventos de ratón teclado
  if(keyListener == 0)
    _keyboard->setEventCallback(this);
  else
    _keyboard->setEventCallback(keyListener);
  
  if(mouseListener == 0)
    _mouse->setEventCallback(this);
  else
    _mouse->setEventCallback(mouseListener);
  
  
  // Preparar Recursos
  //-------------------------------------
  Ogre::String secName, typeName, archName;
  Ogre::ConfigFile cf;
  cf.load("resources.cfg");
  
  Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();
  while (seci.hasMoreElements()){
    secName = seci.peekNextKey();
    Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
    Ogre::ConfigFile::SettingsMultiMap::iterator i;
    for (i = settings->begin(); i != settings->end(); ++i){
      typeName = i->first;
      archName = i->second;
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
    }
  }
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
  
  
  // Definir Gestor de texturas
  //-------------------------------------
  //Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);
  
  // Temporizador
  //-------------------------------------
  //m_pTimer = new Ogre::Timer();
  //m_pTimer->reset();
  
  // Tray Manager
  //-------------------------------
  //_trayManager = new OgreBites::SdkTrayManager("TrayMgr", _renderWindow, _mouse, this);
  //_trayManager->showFrameStats(OgreBites::TL_BOTTOMLEFT);
  //_trayManager->showLogo(OgreBites::TL_BOTTOMRIGHT);
  //_trayManager->hideCursor();
  
  _renderWindow->setActive(true);
  
  return true;
}

// Destructor
//------------------------------
OgreFramework::~OgreFramework(){
  // Destruimos OIS
  if(_inputManager) OIS::InputManager::destroyInputSystem(_inputManager);
  //if(_trayManager)  delete _trayManager;
  if(_root) delete _root;
}

/*
void OgreFramework::updateOgre(double timeSinceLastFrame){
  m_MoveScale = m_MoveSpeed   * (float)timeSinceLastFrame;
  m_RotScale  = m_RotateSpeed * (float)timeSinceLastFrame;
  
  m_TranslateVector = Vector3::ZERO;
  
  getInput();
  moveCamera();
  
  m_FrameEvent.timeSinceLastFrame = timeSinceLastFrame;
  m_pTrayMgr->frameRenderingQueued(m_FrameEvent);
}
*/

/*
void OgreFramework::moveCamera(){
  if(m_pKeyboard->isKeyDown(OIS::KC_LSHIFT))
    m_pCamera->moveRelative(m_TranslateVector);
  else
    m_pCamera->moveRelative(m_TranslateVector / 10);
}
*/

/*
void OgreFramework::getInput(){
    if(m_pKeyboard->isKeyDown(OIS::KC_A))
        m_TranslateVector.x = -m_MoveScale;

    if(m_pKeyboard->isKeyDown(OIS::KC_D))
        m_TranslateVector.x = m_MoveScale;

    if(m_pKeyboard->isKeyDown(OIS::KC_W))
        m_TranslateVector.z = -m_MoveScale;

    if(m_pKeyboard->isKeyDown(OIS::KC_S))
        m_TranslateVector.z = m_MoveScale;
}
*/

//****************************************************************************************
//                             Manejadores de Eventos
//****************************************************************************************

// Evento tecla pulsada
//------------------------------
bool OgreFramework::keyPressed(const OIS::KeyEvent &keyEventRef){
   
  if(_keyboard->isKeyDown(OIS::KC_ESCAPE)){
    _shutDownOgre = true;
    return true;
  }
  //cout << "Tecla presionada" << endl;
  
  return true;
}

// Evento Tecla liberada
//------------------------------
bool OgreFramework::keyReleased(const OIS::KeyEvent &keyEventRef){
  //cout << "Tecla liberada" << endl;
  return true;
}

// Evento movimiento de raton
//------------------------------
bool OgreFramework::mouseMoved(const OIS::MouseEvent &evt){
  //cout << "Ratón movido" << endl;
    return true;
}

// Evento click
//------------------------------
bool OgreFramework::mousePressed(const OIS::MouseEvent &evt, OIS::MouseButtonID id){
  //cout << "Botón del ratón presionado" << endl;
  return true;
}

// Evento soltar click
//------------------------------
bool OgreFramework::mouseReleased(const OIS::MouseEvent &evt, OIS::MouseButtonID id){
  //cout << "Botón del ratón liberado" << endl;
    return true;
}

// Cerrar ventana
//------------------------------
void OgreFramework::windowClosed(Ogre::RenderWindow* _renderWindow) {
  //cout << "Ventana cerrada" << endl;
}

// Cerrando ventana
//------------------------------
bool OgreFramework::windowClosing(Ogre::RenderWindow* _renderWindow) {
  _shutDownOgre = true;
  cout << "Cerrando ventana" << endl;
  return true;
}

// Redimensionar Ventana
//------------------------------
void OgreFramework::windowResized(Ogre::RenderWindow* _renderWindow) {
  //cout << "Ventana redimensionada" << endl;
}

//***********************************************************************************
