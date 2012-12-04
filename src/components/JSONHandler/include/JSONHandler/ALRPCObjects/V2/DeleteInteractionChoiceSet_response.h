#ifndef NSAPPLINKRPCV2_DELETEINTERACTIONCHOICESET_RESPONSE_INCLUDE
#define NSAPPLINKRPCV2_DELETEINTERACTIONCHOICESET_RESPONSE_INCLUDE

#include <string>

#include "Result.h"
#include "JSONHandler/ALRPC2Message.hh"


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

  class DeleteInteractionChoiceSet_response : public ALRPC2Message
  {
  public:
  
    DeleteInteractionChoiceSet_response(const DeleteInteractionChoiceSet_response& c);
    DeleteInteractionChoiceSet_response(void);
    
    virtual ~DeleteInteractionChoiceSet_response(void);
  
    DeleteInteractionChoiceSet_response& operator =(const DeleteInteractionChoiceSet_response&);
  
    bool checkIntegrity(void);

    bool get_success(void) const;
    const Result& get_resultCode(void) const;
    const std::string* get_info(void) const;

    bool set_success(bool success_);
    bool set_resultCode(const Result& resultCode_);
    void reset_info(void);
    bool set_info(const std::string& info_);

  private:
  
    friend class DeleteInteractionChoiceSet_responseMarshaller;


/**
     true, if successful
     false, if failed
*/
      bool success;

///  See Result
      Result resultCode;

///  Provides additional human readable info regarding the result.
      std::string* info;	//!< (1000)
  };

}

#endif
