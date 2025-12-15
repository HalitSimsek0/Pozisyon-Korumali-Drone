#include "motors.h"
static const MotorPerifDef CONN_M1_BL = {};
static const MotorPerifDef CONN_M2_BL = {};
static const MotorPerifDef CONN_M3_BL = {};
static const MotorPerifDef CONN_M4_BL = {};
const MotorPerifDef *motorMapDefaultBrushless[NBR_OF_MOTORS] = {
    &CONN_M1_BL,
    &CONN_M2_BL,
    &CONN_M3_BL,
    &CONN_M4_BL
};
