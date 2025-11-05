#ifndef _.._MSG_MSGS_IFACES_H
#define _.._MSG_MSGS_IFACES_H

#include <iostream>
#include <string>
#include <cstring>

using namespace std;

namespace ..
{
namespace msg
{
class msgs_ifaces
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
    uint32_t lenPad = (0 == (cntPub % sizeof(T))) ? 0 : (sizeof(T) - (cntPub % sizeof(T))); // this doesn't get along with float128.
    if (size < sizeof(T))
    {
      // There are no enough space.
      return 0;
    }
    // Put padding space
    for (int i = 0; i < lenPad; i++)
    {
      *addrPtr = 0;
      addrPtr += 1;
    }
    // Store serialzed value.
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

    uint32_t cntFrag = (cntPubMemberLocal - sizeof(uint32_t)); // cntPubMemberLocal > 4 here
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

  
   ;
  
   ;
  
  int32_t mt_lf_encode_msg;
  
  int32_t mt_rt_encode_msg;
  
  float sys_current_msg;
  
  float sys_volt_msg;
  

  uint32_t copyToBuf(uint8_t *addrPtr)
  {
    uint32_t tmpPub = 0;
    uint32_t arraySize;
    uint32_t stringSize;
    
    
    
    memcpy(addrPtr, &, );
    addrPtr += ;
    cntPub += ;

    
    
    
    
    memcpy(addrPtr, &, );
    addrPtr += ;
    cntPub += ;

    
    
    
    
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    
    memcpy(addrPtr, &mt_lf_encode_msg, 4);
    addrPtr += 4;
    cntPub += 4;

    
    
    
    
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    
    memcpy(addrPtr, &mt_rt_encode_msg, 4);
    addrPtr += 4;
    cntPub += 4;

    
    
    
    
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    
    memcpy(addrPtr, &sys_current_msg, 4);
    addrPtr += 4;
    cntPub += 4;

    
    
    
    
    if (cntPub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntPub % 4)); i++)
      {
        *addrPtr = 0;
        addrPtr += 1;
      }
      cntPub += 4 - (cntPub % 4);
    }
    
    memcpy(addrPtr, &sys_volt_msg, 4);
    addrPtr += 4;
    cntPub += 4;

    
    

    return cntPub;
  }

  uint32_t copyFromBuf(const uint8_t *addrPtr)
  {
    uint32_t tmpSub = 0;
    uint32_t arraySize;
    uint32_t stringSize;

    
    
    
    memcpy(&, addrPtr, );
    addrPtr += ;
    cntSub += ;
    
    
    
    
    memcpy(&, addrPtr, );
    addrPtr += ;
    cntSub += ;
    
    
    
    
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    
    memcpy(&mt_lf_encode_msg, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    
    
    
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    
    memcpy(&mt_rt_encode_msg, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    
    
    
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    
    memcpy(&sys_current_msg, addrPtr, 4);
    addrPtr += 4;
    cntSub += 4;
    
    
    
    
    if (cntSub % 4 > 0)
    {
      for (uint32_t i = 0; i < (4 - (cntSub % 4)); i++)
      {
        addrPtr += 1;
      }
      cntSub += 4 - (cntSub % 4);
    }
    
    memcpy(&sys_volt_msg, addrPtr, 4);
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
    // TODO: store template code here
    return 0;
  }

  uint32_t calcTotalSize()
  {
    uint32_t tmp;
    tmp = 4 + calcRawTotalSize();                  // CDR encoding version.
    tmp += (0 == (tmp % 4) ? 0 : (4 - (tmp % 4))); // Padding
    return tmp;
  }

  void resetCount()
  {
    cntPub = 0;
    cntSub = 0;
    idxSerialized = 0;
    // TODO: store template code here
    return;
  }

  FragCopyReturnType copyToFragBuf(uint8_t *addrPtr, uint32_t size)
  {
    // TODO: store template code here
    return {false, 0};
  }

private:
  std::string type_name = "..::msg::dds_::msgs_ifaces";
};
};
}

namespace message_traits
{
template<>
struct TypeName<..::msg::msgs_ifaces*> {
  static const char* value()
  {
    return "..::msg::dds_::msgs_ifaces_";
  }
};
}

#endif