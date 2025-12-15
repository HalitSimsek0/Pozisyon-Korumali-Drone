#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint32_t count;
    uint32_t expectedMin;
    uint32_t expectedMax;
    uint32_t nextEvaluationTimeMs;
    uint32_t evaluationIntervalMs;
    uint32_t latestCount;
    uint8_t skip;
} rateSupervisor_t;
void rateSupervisorInit(rateSupervisor_t* context, const uint32_t osTimeMs, const uint32_t evaluationIntervalMs, const uint32_t minCount, const uint32_t maxCount, const uint8_t skip);
bool rateSupervisorValidate(rateSupervisor_t* context, const uint32_t osTimeMs);
uint32_t rateSupervisorLatestCount(rateSupervisor_t* context);