#include "../include/JSONHandler/ALRPCObjects/V2/EncodedSyncPData_request.h"
#include "EncodedSyncPData_requestMarshaller.h"
#include "../include/JSONHandler/ALRPCObjects/V2/Marshaller.h"

#define PROTOCOL_VERSION	2


/*
  interface	Ford Sync RAPI
  version	2.0O
  date		2012-11-02
  generated at	Tue Dec  4 14:30:13 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

using namespace NsAppLinkRPCV2;

EncodedSyncPData_request::~EncodedSyncPData_request(void)
{
}


EncodedSyncPData_request::EncodedSyncPData_request(const EncodedSyncPData_request& c) : ALRPC2Message(c)
{
  *this=c;
}


bool EncodedSyncPData_request::checkIntegrity(void)
{
  return EncodedSyncPData_requestMarshaller::checkIntegrity(*this);
}


EncodedSyncPData_request::EncodedSyncPData_request(void) : ALRPC2Message(PROTOCOL_VERSION)
{
}



bool EncodedSyncPData_request::set_data(const std::vector<std::string>& data_)
{
  unsigned int i=data_.size();
  if(i>100 || i<1)  return false;
  while(i--)
  {
    if(data_[i].length()>10000)  return false;
  }
  data=data_;
  return true;
}




const std::vector<std::string>& EncodedSyncPData_request::get_data(void) const 
{
  return data;
}

