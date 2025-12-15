#pragma once
#include <stdint.h>
#include "log.h"
typedef struct {
    uint32_t count;
    uint32_t latestCount;
    uint32_t latestAveragingMs;
    float latestRate;
    uint32_t intervalMs;
} statsCntRateCounter_t;
void statsCntRateCounterInit(statsCntRateCounter_t* counter, uint32_t averagingIntervalMs);
float statsCntRateCounterUpdate(statsCntRateCounter_t* counter, uint32_t now_ms);
typedef struct {
    logByFunction_t logByFunction;
    statsCntRateCounter_t rateCounter;
} statsCntRateLogger_t;
#define STATS_CNT_RATE_INIT(LOGGER, INTERVAL_MS) statsCntRateLoggerInit(LOGGER, INTERVAL_MS)
#define STATS_CNT_RATE_DEFINE(NAME, INTERVAL_MS) statsCntRateLogger_t NAME = {.logByFunction = {.data = &NAME, .aquireFloat = statsCntRateLogHandler}, .rateCounter = {.intervalMs = (INTERVAL_MS), .count = 0, .latestCount = 0, .latestAveragingMs = 0, .latestRate = 0}}
#define STATS_CNT_RATE_EVENT(LOGGER) ((LOGGER)->rateCounter.count++)
#define STATS_CNT_RATE_MULTI_EVENT(LOGGER, CNT) ((LOGGER)->rateCounter.count += CNT)
#define STATS_CNT_RATE_LOG_ADD(NAME, LOGGER) LOG_ADD_BY_FUNCTION(LOG_FLOAT, NAME, LOGGER)
void statsCntRateLoggerInit(statsCntRateLogger_t* logger, uint32_t averagingIntervalMs);
float statsCntRateLogHandler(uint32_t timestamp, void* data);