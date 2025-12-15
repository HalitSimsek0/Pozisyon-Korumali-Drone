#include "kalman_supervisor.h"
#include "param.h"
float maxPosition = 100; 
float maxVelocity = 10; 
bool kalmanSupervisorIsStateWithinBounds(const kalmanCoreData_t* this) {
  for (int i = 0; i < 3; i++) {
    if (maxPosition > 0.0f) {
      if (this->S[KC_STATE_X + i] > maxPosition) {
        return false;
      } else if (this->S[KC_STATE_X + i] < -maxPosition) {
        return false;
      }
    }
    if (maxVelocity > 0.0f) {
      if (this->S[KC_STATE_PX + i] > maxVelocity) {
        return false;
      } else if (this->S[KC_STATE_PX + i] < -maxVelocity) {
        return false;
      }
    }
  }
  return true;
}
PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_FLOAT, maxPos, &maxPosition)
  PARAM_ADD(PARAM_FLOAT, maxVel, &maxVelocity)
PARAM_GROUP_STOP(kalman)
