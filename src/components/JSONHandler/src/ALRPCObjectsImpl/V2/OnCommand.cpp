#include "../include/JSONHandler/ALRPCObjects/V2/OnCommand.h"
#include "OnCommandMarshaller.h"
#include "../include/JSONHandler/ALRPCObjects/V2/Marshaller.h"
#include "TriggerSourceMarshaller.h"

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

OnCommand::~OnCommand(void)
{
}


OnCommand::OnCommand(const OnCommand& c) : ALRPC2Message(c)
{
  *this=c;
}


bool OnCommand::checkIntegrity(void)
{
  return OnCommandMarshaller::checkIntegrity(*this);
}


OnCommand::OnCommand(void) : ALRPC2Message(PROTOCOL_VERSION)
{
}



bool OnCommand::set_cmdID(unsigned int cmdID_)
{
  if(cmdID_>2000000000)  return false;
  cmdID=cmdID_;
  return true;
}

bool OnCommand::set_triggerSource(const TriggerSource& triggerSource_)
{
  if(!TriggerSourceMarshaller::checkIntegrityConst(triggerSource_))   return false;
  triggerSource=triggerSource_;
  return true;
}




unsigned int OnCommand::get_cmdID(void) const
{
  return cmdID;
}

const TriggerSource& OnCommand::get_triggerSource(void) const 
{
  return triggerSource;
}

