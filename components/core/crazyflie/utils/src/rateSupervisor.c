#include "rateSupervisor.h"
void rateSupervisorInit(rateSupervisor_t* context, const uint32_t osTimeMs, const uint32_t evaluationIntervalMs, const uint32_t minCount, const uint32_t maxCount, const uint8_t skip) {
    context->count = 0;
    context->evaluationIntervalMs = evaluationIntervalMs;
    context->expectedMin = minCount;
    context->expectedMax = maxCount;
    context->nextEvaluationTimeMs = osTimeMs + evaluationIntervalMs;
    context->latestCount = 0;
    context->skip = skip;
}
bool rateSupervisorValidate(rateSupervisor_t* context, const uint32_t osTimeMs) {
    bool result = true;
    context->count += 1;
    if (osTimeMs > context->nextEvaluationTimeMs) {
        uint32_t actual = context->count;
        if (actual < context->expectedMin || actual > context->expectedMax) {
            result = false;
        }
        context->latestCount = context->count;
        context->count = 0;
        context->nextEvaluationTimeMs = osTimeMs + context->evaluationIntervalMs;
        if (context->skip > 0) {
            result = true;
            context->skip -= 1;
        }
    }
    return result;
}
uint32_t rateSupervisorLatestCount(rateSupervisor_t* context) {
    return context->latestCount;
}