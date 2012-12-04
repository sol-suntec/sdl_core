#include "../src/../include/JSONHandler/RPC2Objects/NsRPC2Communication/VehicleInfo/GetVehicleType.h"
#include "../src/ALRPCObjectsImpl/V1/ResultMarshaller.h"
#include "../src/../src/RPC2ObjectsImpl//NsRPC2Communication/VehicleInfo/GetVehicleTypeMarshaller.h"

/*
  interface	NsRPC2Communication::VehicleInfo
  version	1.2
  generated at	Tue Dec  4 15:06:30 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

using namespace NsRPC2Communication::VehicleInfo;

bool GetVehicleTypeMarshaller::checkIntegrity(GetVehicleType& s)
{
  return checkIntegrityConst(s);
}


bool GetVehicleTypeMarshaller::fromString(const std::string& s,GetVehicleType& e)
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


const std::string GetVehicleTypeMarshaller::toString(const GetVehicleType& e)
{
  Json::FastWriter writer;
  return checkIntegrityConst(e) ? writer.write(toJSON(e)) : "";
}


bool GetVehicleTypeMarshaller::checkIntegrityConst(const GetVehicleType& s)
{
  return true;
}


Json::Value GetVehicleTypeMarshaller::toJSON(const GetVehicleType& e)
{
  Json::Value json(Json::objectValue);
  if(!checkIntegrityConst(e))
    return Json::Value(Json::nullValue);

  json["jsonrpc"]=Json::Value("2.0");
  json["method"]=Json::Value("AppLinkCore.GetVehicleType");

  json["id"]=Json::Value(e.getId());
  return json;
}


bool GetVehicleTypeMarshaller::fromJSON(const Json::Value& json,GetVehicleType& c)
{
  try
  {
    if(!json.isObject())  return false;
    if(!json.isMember("jsonrpc") || !json["jsonrpc"].isString() || json["jsonrpc"].asString().compare("2.0"))  return false;
    if(!json.isMember("method") || !json["method"].isString() || json["method"].asString().compare("AppLinkCore.GetVehicleType"))  return false;
    if(!json.isMember("id") || !json["id"].isInt()) return false;
    c.setId(json["id"].asInt());

  }
  catch(...)
  {
    return false;
  }
  return checkIntegrity(c);
}
