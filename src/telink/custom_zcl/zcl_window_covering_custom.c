#pragma pack(push, 1)
#include "zcl_include.h"
#pragma pack(pop)

_CODE_ZCL_ status_t zcl_windowCovering_custom_register(
    u8 endpoint, u16 manuCode, u8 attrNum, const zclAttrInfo_t attrTbl[],
    cluster_forAppCb_t cb) {
  // Register with NULL command table so all commands go to application callback
  return (zcl_registerCluster(endpoint, ZCL_CLUSTER_CLOSURES_WINDOW_COVERING,
                              manuCode, attrNum, attrTbl, NULL, cb));
}
