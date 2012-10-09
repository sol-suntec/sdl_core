#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iomanip>

#include "CMessage.hpp"
#include "../../transport/bt/Blob.hpp"

UInt8 CMessage::sVersion = 0;
UInt8 CMessage::sCompressedFlag = 0;
UInt8 CMessage::sFrameType = 0;
UInt8 CMessage::sServiceType = 0;
UInt8 CMessage::sFrameData = 0;
UInt8 CMessage::sSessionID = 0;
UInt32 CMessage::sDataSize = 0;
UInt32 CMessage::sMessageID = 0;
void*  CMessage::sPacketData = 0;

void CMessage::generate()
{
   sVersion        = 0x01; //rand() % (0x0F + 1); // 0x00 – 0x0F 4-Bits
   sCompressedFlag = 0; //rand() % (0x01 + 1); // 0x00 – 0x01 1-Bit
   //sFrameType    = rand() % (0x07 + 1); //0x00 – 0x07 3-Bit
   sFrameType      = 0x01; // SingleFrame

   //sServiceType    = rand() % (0xFF + 1); // 0x00 – 0xFF 8-Bits
   sServiceType    = 0x07; // 0x00 – 0xFF 8-Bits
   sFrameData      = rand() % (0xFF + 1); // 0x00 – 0xFF 8-Bits
   sSessionID      = 0; // 8-Bits //TODO: test multiple sessions
   sDataSize       = rand() % 0xFFFFFFFF + 1; //0x01-0xFFFFFFFF 32-Bits
   sMessageID      = rand() % 0xFFFFFFFF + 1; //0x01-0xFFFFFFFF 32-Bits

   std::cout << std::hex << std::setw(4) << (int)sVersion
                         << std::setw(4) << (int)sCompressedFlag
                         << std::setw(4) << (int)sFrameType
                         << std::setw(4) << (int)sServiceType
                         << std::setw(4) << (int)sFrameData
                         << std::setw(4) << (int)sSessionID << std::endl;

   std::cout << std::hex /*<< std::setw(18)*/ << (int)sDataSize
                         << std::setw(18) << (int)sMessageID << std::endl;
}

void CMessage::write()
{
   generate();

   sPacketData = malloc(12 + sDataSize);

   UInt8 firstByte = ( (sVersion << 4) & 0xF0 )
                       | ( (sCompressedFlag << 3) & 0x08)
                       | (sFrameType & 0x07);

   memcpy(sPacketData, &firstByte, 1);
   memcpy(sPacketData + 1, &sServiceType, 1);
   memcpy(sPacketData + 2, &sFrameData, 1);
   memcpy(sPacketData + 3, &sSessionID, 1);
   memcpy(sPacketData + 4, &sDataSize, 4);
   memcpy(sPacketData + 8, &sMessageID, 4);
}
