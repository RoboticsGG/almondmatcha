#ifndef _MSGS_IFACES_MSG_CHASSIS_IMU_H
#define _MSGS_IFACES_MSG_CHASSIS_IMU_H

#include <iostream>
#include <string>
#include <cstring>

using namespace std;

namespace msgs_ifaces
{
namespace msg
{
class ChassisIMU
{
public:
  uint32_t cntPub = 0;
  uint32_t cntSub = 0;
  uint32_t idxSerialized = 0;

  typedef std::pair<bool, uint32_t> FragCopyReturnType;

  template <class T>
  uint32_t copyPrimToFragBufLocal(uint8_t *&addrPtr,
                                  const uint32_t cntPub,
                                  const uint32_t size,
                                  const T &data)
  {
    uint32_t lenPad = (0 == (cntPub % sizeof(T))) ? 0 : (sizeof(T) - (cntPub % sizeof(T)));
    if (size < sizeof(T))
    {
      return 0;
    }
    for (int i = 0; i < lenPad; i++)
    {
      *addrPtr = 0;
      addrPtr += 1;
    }
    memcpy(addrPtr, &data, sizeof(T));
    addrPtr += sizeof(T);

    return sizeof(T) + lenPad;
  }

  template <class T>
  FragCopyReturnType copyArrayToFragBufLocal(uint8_t *&addrPtr,
                                             const uint32_t size,
                                             T &data,
                                             uint32_t &cntPubMemberLocal)
  {
    uint32_t pubDataSize = data.size();
    uint32_t cntLocalFrag = 0;

    if (cntPubMemberLocal < sizeof(uint32_t))
    {
      if (size < sizeof(uint32_t))
      {
        return {false, 0};
      }
      memcpy(addrPtr, &pubDataSize, sizeof(uint32_t));
      addrPtr += sizeof(uint32_t);
      cntPubMemberLocal += sizeof(uint32_t);
      cntLocalFrag += sizeof(uint32_t);
    }

    uint32_t cntFrag = (cntPubMemberLocal - sizeof(uint32_t));
    uint32_t tmp = std::min(pubDataSize - cntFrag, size - cntLocalFrag);
    if (0 < tmp)
    {
      memcpy(addrPtr, data.data() + cntFrag, tmp);
      addrPtr += tmp;
      cntPubMemberLocal += tmp;
      cntLocalFrag += tmp;
    }

    return {(cntPubMemberLocal - sizeof(uint32_t)) >= pubDataSize, cntLocalFrag};
  }

  
  int32_t accel_x;
  
  int32_t accel_y;
  
  int32_t accel_z;
  
  int32_t gyro_x;
  
  int32_t gyro_y;
  
  int32_t gyro_z;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    // accel_x (int32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &accel_x, 4);
    addrPtr += 4;
    cntPub += 4;

    // accel_y (int32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &accel_y, 4);
    addrPtr += 4;
    cntPub += 4;

    // accel_z (int32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &accel_z, 4);
    addrPtr += 4;
    cntPub += 4;

    // gyro_x (int32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &gyro_x, 4);
    addrPtr += 4;
    cntPub += 4;

    // gyro_y (int32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &gyro_y, 4);
    addrPtr += 4;
    cntPub += 4;

    // gyro_z (int32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &gyro_z, 4);
    addrPtr += 4;
    cntPub += 4;

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr)
  {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    // accel_x (int32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&accel_x, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    // accel_y (int32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&accel_y, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    // accel_z (int32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&accel_z, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    // gyro_x (int32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&gyro_x, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    // gyro_y (int32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&gyro_y, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    // gyro_z (int32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&gyro_z, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;

    return cntSub;
  }

  void memAlign(uint8_t *addrPtr)
  {
    if (cntPub % 4 > 0)
    {
      addrPtr += cntPub;
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    return;
  }

  uint32_t getTotalSize()
  {
    uint32_t tmpCntPub = cntPub;
    cntPub = 0;
    return tmpCntPub;
  }

  uint32_t getPubCnt()
  {
    return cntPub;
  }

  uint32_t calcRawTotalSize()
  {
    return 0;
  }

  uint32_t calcTotalSize()
  {
    uint32_t tmp;
    tmp = 4 + calcRawTotalSize();
    tmp += (0 == (tmp % 4) ? 0 : (4 - (tmp % 4)));
    return tmp;
  }

  void resetCount()
  {
    cntPub = 0;
    cntSub = 0;
    idxSerialized = 0;
    return;
  }

  FragCopyReturnType copyToFragBuf(uint8_t *addrPtr, uint32_t size)
  {
    return {false, 0};
  }

private:
  std::string type_name = "msgs_ifaces::msg::dds_::ChassisIMU";
};
};
}

namespace message_traits
{
template<>
struct TypeName<msgs_ifaces::msg::ChassisIMU*> {
  static const char* value()
  {
    return "msgs_ifaces::msg::dds_::ChassisIMU_";
  }
};
}

#endif
