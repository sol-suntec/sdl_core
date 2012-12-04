#include "../include/JSONHandler/ALRPCObjects/V2/DeleteInteractionChoiceSet_request.h"


#include "DeleteInteractionChoiceSet_requestMarshaller.h"


/*
  interface	Ford Sync RAPI
  version	2.0O
  date		2012-11-02
  generated at	Tue Dec  4 14:30:13 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

using namespace NsAppLinkRPCV2;


bool DeleteInteractionChoiceSet_requestMarshaller::checkIntegrity(DeleteInteractionChoiceSet_request& s)
{
  return checkIntegrityConst(s);
}


bool DeleteInteractionChoiceSet_requestMarshaller::fromString(const std::string& s,DeleteInteractionChoiceSet_request& e)
{
  try
  {
    Json::Reader reader;
    Json::Value json;
    if(!reader.parse(s,json,false))  return false;
    if(!fromJSON(json,e))  return false;
  }
  catch(...)
  {
    return false;
  }
  return true;
}


const std::string DeleteInteractionChoiceSet_requestMarshaller::toString(const DeleteInteractionChoiceSet_request& e)
{
  Json::FastWriter writer;
  return checkIntegrityConst(e) ? writer.write(toJSON(e)) : "";
}


bool DeleteInteractionChoiceSet_requestMarshaller::checkIntegrityConst(const DeleteInteractionChoiceSet_request& s)
{
  if(s.interactionChoiceSetID>2000000000)  return false;
  return true;
}

Json::Value DeleteInteractionChoiceSet_requestMarshaller::toJSON(const DeleteInteractionChoiceSet_request& e)
{
  Json::Value json(Json::objectValue);
  if(!checkIntegrityConst(e))
    return Json::Value(Json::nullValue);

  json["interactionChoiceSetID"]=Json::Value(e.interactionChoiceSetID);

  return json;
}


bool DeleteInteractionChoiceSet_requestMarshaller::fromJSON(const Json::Value& json,DeleteInteractionChoiceSet_request& c)
{
  try
  {
    if(!json.isObject())  return false;

    if(!json.isMember("interactionChoiceSetID"))  return false;
    {
      const Json::Value& j=json["interactionChoiceSetID"];
      if(!j.isInt())  return false;
      c.interactionChoiceSetID=j.asInt();
    }

  }
  catch(...)
  {
    return false;
  }
  return checkIntegrity(c);
}

