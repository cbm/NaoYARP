/**
 * @author Alexandros Paraschos
 *
 * Version : $Id$
 * This file was generated by Aldebaran Robotics ModuleGenerator
 */

#ifndef module_H
#define module_H
#include "naoyarp.h"
#include "alptr.h"

namespace AL
{
  class ALBroker;
}

/**
 * DESCRIBE YOUR CLASS HERE
 */
#define module_VERSION_MAJOR "0"
#define module_VERSION_MINOR "0"
class module : public AL::ALModule
{

  public:

    /**
     * Default Constructor.
     */
    module(AL::ALPtr<AL::ALBroker> pBroker, const std::string& pName );

    /**
     * Destructor.
     */
    virtual ~module();

    /**
     * version
     * @return The version number of module
     */
    std::string version();

    // **************************** BOUND METHODS ********************************** 
    /* dataChanged. Called by stm when subcription
     * has been modified.
     * @param pDataName Name of the suscribed data
     * @param pValue Value of the suscribed data
     * @param pMessage Message written by user during subscription
     */
    void dataChanged(const std::string& pDataName, const ALValue& pValue, const std::string& pMessage);


    /**
     * innerTest
     * @return True if all the tests passed
     */
    bool innerTest();


    /**
     * dummy Function. An autogenerated example method.
     * @param pMsg Message to show on screen
     * @param pFoo the function will return this parameter
     * @return an alvalue\n
     */
    ALValue dummyFunction(const std::string& pMsg, const ALValue& pFoo );


};
#endif // module_H
