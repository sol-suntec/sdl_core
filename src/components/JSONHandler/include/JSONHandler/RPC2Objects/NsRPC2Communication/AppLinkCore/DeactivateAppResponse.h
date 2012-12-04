#ifndef NSRPC2COMMUNICATION_APPLINKCORE_DEACTIVATEAPPRESPONSE_INCLUDE
#define NSRPC2COMMUNICATION_APPLINKCORE_DEACTIVATEAPPRESPONSE_INCLUDE

#include "JSONHandler/RPC2Response.h"


/*
  interface	NsRPC2Communication::AppLinkCore
  version	1.2
  generated at	Tue Dec  4 15:06:30 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

namespace NsRPC2Communication
{
  namespace AppLinkCore
  {

    class DeactivateAppResponse : public ::NsRPC2Communication::RPC2Response
    {
    public:
    
      DeactivateAppResponse(const DeactivateAppResponse& c);
      DeactivateAppResponse(void);
    
      DeactivateAppResponse& operator =(const DeactivateAppResponse&);
    
      virtual ~DeactivateAppResponse(void);
    
      bool checkIntegrity(void);
    

    private:

      friend class DeactivateAppResponseMarshaller;


    };
  }
}

#endif
