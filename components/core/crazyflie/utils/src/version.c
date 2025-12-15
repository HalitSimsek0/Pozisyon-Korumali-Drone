#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "param.h"
const char * V_SLOCAL_REVISION="{local_revision}";
const char * V_SREVISION="{revision}";
const char * V_STAG="{tag}";
const char * V_BRANCH="{branch}";
const bool V_MODIFIED= 0 ;
const bool V_PRODUCTION_RELEASE= 0 ;
const uint32_t V_REVISION_0=0;
const uint16_t V_REVISION_1=1;
PARAM_GROUP_START(firmware)
PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, revision0, &V_REVISION_0)
PARAM_ADD(PARAM_UINT16 | PARAM_RONLY, revision1, &V_REVISION_1)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, modified, &V_MODIFIED)
PARAM_GROUP_STOP(firmware)