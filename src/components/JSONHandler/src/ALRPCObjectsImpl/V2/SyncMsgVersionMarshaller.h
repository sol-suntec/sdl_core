#ifndef NSAPPLINKRPCV2_SYNCMSGVERSIONMARSHALLER_INCLUDE
#define NSAPPLINKRPCV2_SYNCMSGVERSIONMARSHALLER_INCLUDE

#include <string>
#include <jsoncpp/json.h>

#include "../include/JSONHandler/ALRPCObjects/V2/SyncMsgVersion.h"


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

  struct SyncMsgVersionMarshaller
  {
    static bool checkIntegrity(SyncMsgVersion& e);
    static bool checkIntegrityConst(const SyncMsgVersion& e);
  
    static bool fromString(const std::string& s,SyncMsgVersion& e);
    static const std::string toString(const SyncMsgVersion& e);
  
    static bool fromJSON(const Json::Value& s,SyncMsgVersion& e);
    static Json::Value toJSON(const SyncMsgVersion& e);
  };
}

#endif
