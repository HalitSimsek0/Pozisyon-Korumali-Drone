#include <math.h>
#include "outlierFilter.h"
#include "stabilizer_types.h"
#include "log.h"
#include "debug_cf.h"
#define BUCKET_ACCEPTANCE_LEVEL 3
#define MAX_BUCKET_FILL 10
#define FILTER_CLOSE_DELAY_COUNT 30
static float acceptanceLevel = 0.0;
static float errorDistance;
static int filterCloseDelayCounter __attribute__((unused)) = 0;
static int previousFilterIndex __attribute__((unused)) = 0;
typedef struct {
  float acceptanceLevel;
  int bucket;
} filterLevel_t;
#define FILTER_LEVELS 5
#define FILTER_NONE FILTER_LEVELS
filterLevel_t filterLevels[FILTER_LEVELS] = {
  {.acceptanceLevel = 0.4},
  {.acceptanceLevel = 0.8},
  {.acceptanceLevel = 1.2},
  {.acceptanceLevel = 1.6},
  {.acceptanceLevel = 2.0},
};
// Removed: TDOA outlier filter functions - TDOA hardware not available
static float sq(float a) {return a * a;}
// Removed: Lighthouse sweep outlier filter functions - Lighthouse hardware not available
static float __attribute__((unused)) distanceSq(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}
static void __attribute__((unused)) addToBucket(filterLevel_t* filter) {
  if (filter->bucket < MAX_BUCKET_FILL) {
    filter->bucket++;
  }
}
static void __attribute__((unused)) removeFromBucket(filterLevel_t* filter) {
  if (filter->bucket > 0) {
    filter->bucket--;
  }
}
static int __attribute__((unused)) updateBuckets(float errorDistance) {
  int filterIndex = FILTER_NONE;
  for (int i = FILTER_LEVELS - 1; i >= 0; i--) {
    filterLevel_t* filter = &filterLevels[i];
    if (errorDistance < filter->acceptanceLevel) {
      removeFromBucket(filter);
    } else {
      addToBucket(filter);
    }
    if (filter->bucket < BUCKET_ACCEPTANCE_LEVEL) {
      filterIndex = i;
    }
  }
  return filterIndex;
}
LOG_GROUP_START(outlierf)
  LOG_ADD(LOG_INT32, bucket0, &filterLevels[0].bucket)
  LOG_ADD(LOG_INT32, bucket1, &filterLevels[1].bucket)
  LOG_ADD(LOG_INT32, bucket2, &filterLevels[2].bucket)
  LOG_ADD(LOG_INT32, bucket3, &filterLevels[3].bucket)
  LOG_ADD(LOG_INT32, bucket4, &filterLevels[4].bucket)
  LOG_ADD(LOG_FLOAT, accLev, &acceptanceLevel)
  LOG_ADD(LOG_FLOAT, errD, &errorDistance)
LOG_GROUP_STOP(outlierf)
