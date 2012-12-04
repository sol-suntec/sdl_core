#include "../src/../include/JSONHandler/RPC2Objects/NsRPC2Communication/VehicleInfo/GetVehicleTypeResponse.h"
#include "../src/../include/JSONHandler/RPC2Objects/Marshaller.h"

/*
  interface	NsRPC2Communication::VehicleInfo
  version	1.2
  generated at	Tue Dec  4 15:06:30 2012
  source stamp	Tue Dec  4 14:21:32 2012
  author	robok0der
*/

using namespace NsRPC2Communication::VehicleInfo;


GetVehicleTypeResponse& GetVehicleTypeResponse::operator =(const GetVehicleTypeResponse& c)
{
  vehicleType=c.vehicleType;
  return *this;
}


GetVehicleTypeResponse::~GetVehicleTypeResponse(void)
{
}


GetVehicleTypeResponse::GetVehicleTypeResponse(void) : 
  RPC2Response(Marshaller::METHOD_NSRPC2COMMUNICATION_VEHICLEINFO__GETVEHICLETYPERESPONSE)
{
}


GetVehicleTypeResponse::GetVehicleTypeResponse(const GetVehicleTypeResponse& c) : RPC2Response(Marshaller::METHOD_NSRPC2COMMUNICATION_VEHICLEINFO__GETVEHICLETYPERESPONSE,c.getId(),c.getResult())
{
  *this=c;
}


const NsAppLinkRPC::VehicleType& GetVehicleTypeResponse::get_vehicleType(void)
{
  return vehicleType;
}

bool GetVehicleTypeResponse::set_vehicleType(const NsAppLinkRPC::VehicleType& vehicleType_)
{
  vehicleType=vehicleType_;
  return true;
}

bool GetVehicleTypeResponse::checkIntegrity(void)
{
  return GetVehicleTypeResponseMarshaller::checkIntegrity(*this);
}
