#include <stdint.h>
#include "log.h"
#include "range.h"
#include "stabilizer_types.h"
#include "estimator.h"
static uint16_t ranges[RANGE_T_END] = {0,};
void rangeSet(rangeDirection_t direction, float range_m)
{
  if (direction > (RANGE_T_END-1)) return;
  ranges[direction] = range_m * 1000;
}
float rangeGet(rangeDirection_t direction)
{
    if (direction > (RANGE_T_END-1)) return 0;
  return ranges[direction];
}
bool rangeEnqueueDownRangeInEstimator(float distance, float stdDev, uint32_t timeStamp) {
  tofMeasurement_t tofData;
  tofData.timestamp = timeStamp;
  tofData.distance = distance;
  tofData.stdDev = stdDev;
  return estimatorEnqueueTOF(&tofData);
}
LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, front, &ranges[rangeFront])
LOG_ADD(LOG_UINT16, back, &ranges[rangeBack])
LOG_ADD(LOG_UINT16, up, &ranges[rangeUp])
LOG_ADD(LOG_UINT16, left, &ranges[rangeLeft])
LOG_ADD(LOG_UINT16, right, &ranges[rangeRight])
LOG_ADD(LOG_UINT16, zrange, &ranges[rangeDown])
LOG_GROUP_STOP(range)