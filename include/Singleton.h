#ifndef SINGLETON_H
#define SINGLETON_H

  /**
   * A Singleton templated class
   * Just inherits from it to make any class Singleton
   */
  template<typename T>
  class Singleton {
  public:
    /**
     * Retrieves the instanced class and creates it if does not exist
     * @return a reference to the instanced class
     */
    static T& getInstance() {
      if (instance == NULL) instance = new T;
      return *instance;
    }

    /**
     * Retrieves the instanced class and creates it if does not exist
     * @return a pointer to the instanced class
     */
    static T* getInstancePtr() {
      if (instance == NULL) instance = new T;
      return instance;
    }

    /**
     * Destroys the instanced class
     */
    static void destroy() {
      if(instance != NULL) delete instance;
    }

  private:
    static T* instance; ///< A pointer to the instanced class

    Singleton & operator=(const Singleton &);
  };

  template<typename T>
  T* Singleton<T>::instance = NULL;



#endif
