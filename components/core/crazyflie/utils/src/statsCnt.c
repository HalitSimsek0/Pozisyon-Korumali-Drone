#include "statsCnt.h"
#include "debug_cf.h"
void statsCntRateCounterInit(statsCntRateCounter_t* counter, uint32_t averagingIntervalMs) {
    counter->intervalMs = averagingIntervalMs;
    counter->count = 0;
    counter->latestCount = 0;
    counter->latestAveragingMs = 0;
    counter->latestRate = 0.0f;
}
float statsCntRateCounterUpdate(statsCntRateCounter_t* counter, uint32_t now_ms) {
    uint32_t dt_ms = now_ms - counter->latestAveragingMs;
    if (dt_ms > counter->intervalMs) {
        float dt_s = dt_ms / 1000.0f;
        float dv = counter->count - counter->latestCount;
        counter->latestRate = dv / dt_s;
        counter->latestCount = counter->count;
        counter->latestAveragingMs = now_ms;
    }
    return counter->latestRate;
}
void statsCntRateLoggerInit(statsCntRateLogger_t* logger, uint32_t averagingIntervalMs) {
    statsCntRateCounterInit(&logger->rateCounter, averagingIntervalMs);
    logger->logByFunction.data = (void*)logger;
    logger->logByFunction.aquireFloat = statsCntRateLogHandler;
}
float statsCntRateLogHandler(uint32_t timestamp, void* data) {
    statsCntRateLogger_t* logger = (statsCntRateLogger_t*)data;
    return statsCntRateCounterUpdate(&logger->rateCounter, timestamp);
}
