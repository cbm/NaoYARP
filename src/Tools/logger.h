#ifndef _logger_h_
#define _logger_h_ 1

#include <iostream>
#include <fstream>
#include <set>
#include <cstdio>
#include "sys/stat.h"

#include "Tools/singleton.h"

//TODO mutex needed


class LoggerClass {

public:

    enum MsgType { FatalError=0, Error, Warning, Info, ExtraInfo, ExtraExtraInfo };

    ~LoggerClass () {
        ErrorLog.close();
    }

    template<class T>
    void WriteMsg ( std::string name, const T & msg, MsgType type ) {

        if ( type > VerbosityLevel )
            return;

        switch (type) {
        case FatalError:
        case Error:
            WriteMsgToBuffers ( name, msg, "red" );
            break;
        case Warning:
            WriteMsgToBuffers ( name, msg, "yellow" );
            break;
        case Info:
        case ExtraInfo:
            if ( ! ActivityFilterEnabled )
                WriteMsgToBuffers ( name, msg, "default" );
            else if ( ActivityFilter.find(name) != ActivityFilter.end() )
                WriteMsgToBuffers ( name, msg, "default" );
            break;

        case ExtraExtraInfo:
            if ( ! ActivityFilterEnabled )
                WriteMsgToBuffers ( name, msg, "blue" );
            else if ( ActivityFilter.find(name) != ActivityFilter.end() )
                WriteMsgToBuffers ( name, msg, "blue" );
        }
    }


    LoggerClass () {


        MsgLogFile = "MonasLog.txt";
        VerbosityLevel = 0;
        CerrEnabled = false;
        ColorEnabled = false;


        struct stat stFileInfo;
        int intStat = stat((MsgLogFile+".0").c_str(),&stFileInfo);
        if (intStat == 0 && stFileInfo.st_size > 512 )
            rename( (MsgLogFile + ".0").c_str() , (MsgLogFile + ".1").c_str() );


        ErrorLog.open( ( MsgLogFile+".0" ).c_str() );
        if ( ! ErrorLog.is_open() )
            std::cerr<<"Can't open MessageLog file: "<<MsgLogFile<<std::endl;

        ColorMap["red"]     = "\033[1;31m";
        ColorMap["blue"]    = "\033[1;34m";
        ColorMap["lBlue"]   = "\033[21;34m";
        ColorMap["green"]   = "\033[1;32m";
        ColorMap["yellow"]  = "\033[1;33m";
        ColorMap["default"] = "\033[0m";

    }

private:

    template< class T>
    void WriteMsgToBuffers ( std::string name, const T& msg, std::string color ) {
        if ( ErrorLog.is_open() )
            ErrorLog<<name<<" : "<<msg<<std::endl;
        if ( CerrEnabled ) {
            if ( ColorEnabled )
                std::cerr<<ColorMap[color]<<name<<" : "<<msg<<ColorMap["default"]<<std::endl;
            else
                std::cerr<<name<<" : "<<msg<<std::endl;
        }
    }



    int VerbosityLevel;

    std::string MsgLogFile;
    std::ofstream ErrorLog;

    std::set<std::string> ActivityFilter;
    bool ActivityFilterEnabled;

    bool CerrEnabled;

    bool ColorEnabled;

    std::map<std::string,std::string> ColorMap;

};

typedef Singleton<LoggerClass> Logger;

#endif // _logger_h_
