/*
 * main.cpp  
 * 
 */
/**
 *  @file main.cpp
 *  @author Manuel Hervás Ortega
 *  @date 24-09-2012
 */


#include "arOgre.hpp"

int main(int argc, char **argv){
  try{
    arOgre test;
    test.start();
  }catch(std::exception& e){
    fprintf(stderr, "An exception has occurred: %s\n", e.what());
  }
  
  return 0;
}
