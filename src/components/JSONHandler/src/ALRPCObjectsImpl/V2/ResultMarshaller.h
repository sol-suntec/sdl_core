#ifndef NSAPPLINKRPCV2_RESULTMARSHALLER_INCLUDE
#define NSAPPLINKRPCV2_RESULTMARSHALLER_INCLUDE

#include <string>
#include <jsoncpp/json.h>

#include "PerfectHashTable.h"

#include "../include/JSONHandler/ALRPCObjects/V2/Result.h"


/*
  interface	Ford Sync RAPI
  version	2.0O
  date		2012-11-02
  generated at	Tue Dec  4 14:30:13 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

namespace NsAppLinkRPCV2
{

//! marshalling class for Result

  class ResultMarshaller
  {
  public:
  
    static std::string toName(const Result& e) 	{ return getName(e.mInternal) ?: ""; }
  
    static bool fromName(Result& e,const std::string& s)
    { 
      return (e.mInternal=getIndex(s.c_str()))!=Result::INVALID_ENUM;
    }
  
    static bool checkIntegrity(Result& e)		{ return e.mInternal!=Result::INVALID_ENUM; } 
    static bool checkIntegrityConst(const Result& e)	{ return e.mInternal!=Result::INVALID_ENUM; } 
  
    static bool fromString(const std::string& s,Result& e);
    static const std::string toString(const Result& e);
  
    static bool fromJSON(const Json::Value& s,Result& e);
    static Json::Value toJSON(const Result& e);
  
    static const char* getName(Result::ResultInternal e)
    {
       return (e>=0 && e<29) ? mHashTable[e].name : NULL;
    }
  
    static const Result::ResultInternal getIndex(const char* s);
  
    static const PerfectHashTable mHashTable[29];
  };
  
}

#endif
