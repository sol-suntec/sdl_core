#ifndef NSRPC2COMMUNICATION_UI_RESETGLOBALPROPERTIES_INCLUDE
#define NSRPC2COMMUNICATION_UI_RESETGLOBALPROPERTIES_INCLUDE

#include <vector>
#include "JSONHandler/RPC2Request.h"

#include "../include/JSONHandler/ALRPCObjects/V1/GlobalProperty.h"

/*
  interface	NsRPC2Communication::UI
  version	1.2
  generated at	Tue Dec  4 15:06:30 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

namespace NsRPC2Communication
{
  namespace UI
  {

    class ResetGlobalProperties : public ::NsRPC2Communication::RPC2Request
    {
    public:
    
      ResetGlobalProperties(const ResetGlobalProperties& c);
      ResetGlobalProperties(void);
    
      ResetGlobalProperties& operator =(const ResetGlobalProperties&);
    
      virtual ~ResetGlobalProperties(void);
    
      bool checkIntegrity(void);
    
// getters
      const std::vector< NsAppLinkRPC::GlobalProperty>& get_properties(void);

      int get_appId(void);


// setters
/// 1 <= size <= 100
      bool set_properties(const std::vector< NsAppLinkRPC::GlobalProperty>& properties);

      bool set_appId(int appId);


    private:

      friend class ResetGlobalPropertiesMarshaller;

      std::vector< NsAppLinkRPC::GlobalProperty> properties;
      int appId;

    };
  }
}

#endif
