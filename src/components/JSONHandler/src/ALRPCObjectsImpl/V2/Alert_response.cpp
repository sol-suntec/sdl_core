#include "../include/JSONHandler/ALRPCObjects/V2/Alert_response.h"
#include "Alert_responseMarshaller.h"
#include "../include/JSONHandler/ALRPCObjects/V2/Marshaller.h"
#include "ResultMarshaller.h"

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
Alert_response& Alert_response::operator =(const Alert_response& c)
{
  success= c.success;
  resultCode= c.resultCode;
  info= c.info ? new std::string(c.info[0]) : 0;
  tryAgainTime= c.tryAgainTime;

  return *this;
}


Alert_response::~Alert_response(void)
{
  if(info)
    delete info;
}


Alert_response::Alert_response(const Alert_response& c) : ALRPC2Message(c)
{
  *this=c;
}


bool Alert_response::checkIntegrity(void)
{
  return Alert_responseMarshaller::checkIntegrity(*this);
}


Alert_response::Alert_response(void) : ALRPC2Message(PROTOCOL_VERSION),
      info(0)
{
}



bool Alert_response::set_success(bool success_)
{
  success=success_;
  return true;
}

bool Alert_response::set_resultCode(const Result& resultCode_)
{
  if(!ResultMarshaller::checkIntegrityConst(resultCode_))   return false;
  resultCode=resultCode_;
  return true;
}

bool Alert_response::set_info(const std::string& info_)
{
  if(info_.length()>1000)  return false;
  delete info;
  info=0;

  info=new std::string(info_);
  return true;
}

void Alert_response::reset_info(void)
{
  if(info)
    delete info;
  info=0;
}

bool Alert_response::set_tryAgainTime(unsigned int tryAgainTime_)
{
  if(tryAgainTime_>2000000000)  return false;
  tryAgainTime=tryAgainTime_;
  return true;
}




bool Alert_response::get_success(void) const
{
  return success;
}

const Result& Alert_response::get_resultCode(void) const 
{
  return resultCode;
}

const std::string* Alert_response::get_info(void) const 
{
  return info;
}

unsigned int Alert_response::get_tryAgainTime(void) const
{
  return tryAgainTime;
}

