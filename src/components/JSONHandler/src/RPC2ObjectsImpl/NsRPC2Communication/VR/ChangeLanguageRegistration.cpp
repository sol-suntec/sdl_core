#include "../src/../include/JSONHandler/RPC2Objects/NsRPC2Communication/VR/ChangeLanguageRegistration.h"
#include "../src/../include/JSONHandler/RPC2Objects/Marshaller.h"

/*
  interface	NsRPC2Communication::VR
  version	1.2
  generated at	Tue Dec  4 15:06:30 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

using namespace NsRPC2Communication::VR;


ChangeLanguageRegistration& ChangeLanguageRegistration::operator =(const ChangeLanguageRegistration& c)
{
  language=c.language;
  appId=c.appId;
  return *this;
}


ChangeLanguageRegistration::~ChangeLanguageRegistration(void)
{
}


ChangeLanguageRegistration::ChangeLanguageRegistration(void) : 
  RPC2Request(Marshaller::METHOD_NSRPC2COMMUNICATION_VR__CHANGELANGUAGEREGISTRATION)
{
}


ChangeLanguageRegistration::ChangeLanguageRegistration(const ChangeLanguageRegistration& c) : RPC2Request(Marshaller::METHOD_NSRPC2COMMUNICATION_VR__CHANGELANGUAGEREGISTRATION,c.getId())
{
  *this=c;
}


const NsAppLinkRPC::Language& ChangeLanguageRegistration::get_language(void)
{
  return language;
}

bool ChangeLanguageRegistration::set_language(const NsAppLinkRPC::Language& language_)
{
  language=language_;
  return true;
}

int ChangeLanguageRegistration::get_appId(void)
{
  return appId;
}

bool ChangeLanguageRegistration::set_appId(int appId_)
{
  appId=appId_;
  return true;
}

bool ChangeLanguageRegistration::checkIntegrity(void)
{
  return ChangeLanguageRegistrationMarshaller::checkIntegrity(*this);
}
