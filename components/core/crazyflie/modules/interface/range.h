#pragma once
typedef enum {
    rangeFront=0,
    rangeBack,
    rangeLeft,
    rangeRight,
    rangeUp,
    rangeDown,
    RANGE_T_END,
} rangeDirection_t;
void rangeSet(rangeDirection_t direction, float range_m);
float rangeGet(rangeDirection_t direction);
bool rangeEnqueueDownRangeInEstimator(float distance, float stdDev, uint32_t timeStamp);