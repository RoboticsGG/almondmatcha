#ifndef _MSGS_IFACES_MSG_CHASSIS_CTRL_H
#define _MSGS_IFACES_MSG_CHASSIS_CTRL_H

#include <iostream>
#include <string>
#include <cstring>

using namespace std;

namespace msgs_ifaces
{
namespace msg
{
class ChassisCtrl
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

  
  uint8_t fdr_msg;
  
  float ro_ctrl_msg;
  
  uint8_t spd_msg;
  
  uint8_t bdr_msg;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    // fdr_msg (uint8)
    memcpy(addrPtr, &fdr_msg, 1);
    addrPtr += 1;
    cntPub += 1;

    // ro_ctrl_msg (float32)
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    memcpy(addrPtr, &ro_ctrl_msg, 4);
    addrPtr += 4;
    cntPub += 4;

    // spd_msg (uint8)
    memcpy(addrPtr, &spd_msg, 1);
    addrPtr += 1;
    cntPub += 1;

    // bdr_msg (uint8)
    memcpy(addrPtr, &bdr_msg, 1);
    addrPtr += 1;
    cntPub += 1;

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr)
  {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    // fdr_msg (uint8)
    memcpy(&fdr_msg, addrPtr, 1);
    addrPtr += 1;
    cntSub += 1;
    
    // ro_ctrl_msg (float32)
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    memcpy(&ro_ctrl_msg, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    // spd_msg (uint8)
    memcpy(&spd_msg, addrPtr, 1);
    addrPtr += 1;
    cntSub += 1;
    
    // bdr_msg (uint8)
    memcpy(&bdr_msg, addrPtr, 1);
    addrPtr += 1;
    cntSub += 1;

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
  std::string type_name = "msgs_ifaces::msg::dds_::ChassisCtrl";
};
};
}

namespace message_traits
{
template<>
struct TypeName<msgs_ifaces::msg::ChassisCtrl*> {
  static const char* value()
  {
    return "msgs_ifaces::msg::dds_::ChassisCtrl_";
  }
};
}

#endif
