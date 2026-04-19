#include <unity.h>

#include <vector>

#include "../../../support/pulse_engine_rmt/PulseEngineRmtModel.h"
#include "../../../support/pulse_engine_rmt/PulseEngineRmtSequencer.h"

// Native env compiles tests only, so include implementation units directly.
#include "../../../support/pulse_engine_rmt/PulseEngineTrapezoid.cpp"
#include "../../../support/pulse_engine_rmt/PulseEngineRmtModel.cpp"
#include "../../../support/pulse_engine_rmt/PulseEngineRmtSequencer.cpp"

namespace {
struct Timeline
{
    uint64_t ticks = 0;
    uint32_t risingEdges = 0;
};

Timeline reconstruct_timeline(const std::vector<PulseEngineRmtSymbol> &symbols)
{
    Timeline timeline = {};
    bool level = false;

    for (const PulseEngineRmtSymbol &symbol : symbols)
    {
        const uint16_t d0 = symbol.duration0();
        if (d0 > 0)
        {
            const bool l0 = symbol.level0();
            if (!level && l0) timeline.risingEdges = timeline.risingEdges + 1;
            level = l0;
            timeline.ticks = timeline.ticks + static_cast<uint64_t>(d0);
        }

        const uint16_t d1 = symbol.duration1();
        if (d1 > 0)
        {
            const bool l1 = symbol.level1();
            if (!level && l1) timeline.risingEdges = timeline.risingEdges + 1;
            level = l1;
            timeline.ticks = timeline.ticks + static_cast<uint64_t>(d1);
        }
    }

    return timeline;
}

class FakeRmtTx final : public PulseEngineRmtSequencer::ITx
{
public:
    bool queueTransaction(const PulseEngineRmtSymbol *symbols, size_t symbolCount) override
    {
        if (symbols == nullptr || symbolCount == 0) return false;

        frameSizes.push_back(symbolCount);
        for (size_t i = 0; i < symbolCount; ++i)
        {
            stream.push_back(symbols[i]);
        }
        return true;
    }

    std::vector<size_t> frameSizes = {};
    std::vector<PulseEngineRmtSymbol> stream = {};
};
} // namespace

void test_model_splits_long_low_duration_without_losing_ticks(void)
{
    PulseEngineRmtModel model = {};

    PulseEngineRmtModel::BuildConfig config = {};
    config.resolutionHz = 1000000;
    config.pulseHighTicks = 2;
    config.pulseCount = 3;
    config.startSpeedHz = 1;
    config.endSpeedHz = 1;

    TEST_ASSERT_TRUE(model.begin(config));

    std::vector<PulseEngineRmtSymbol> allSymbols = {};
    uint32_t pulses = 0;
    uint64_t plannedTicks = 0;

    PulseEngineRmtFrame frame = {};
    while (model.buildNextFrame(&frame))
    {
        pulses = pulses + frame.pulseCount;
        plannedTicks = plannedTicks + frame.ticksTotal;
        for (uint16_t i = 0; i < frame.symbolCount; ++i)
        {
            allSymbols.push_back(frame.symbols[i]);
        }
    }

    const Timeline timeline = reconstruct_timeline(allSymbols);

    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, pulses);
    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, timeline.risingEdges);
    TEST_ASSERT_EQUAL_UINT64(plannedTicks, timeline.ticks);
    TEST_ASSERT_EQUAL_UINT64(3000000ULL, timeline.ticks);
    TEST_ASSERT_TRUE(allSymbols.size() > static_cast<size_t>(config.pulseCount));
}

void test_sequencer_chains_frames_and_finishes_exact_pulse_count(void)
{
    FakeRmtTx tx = {};
    PulseEngineRmtSequencer sequencer = {};

    PulseEngineRmtModel::BuildConfig config = {};
    config.resolutionHz = 1000000;
    config.pulseHighTicks = 2;
    config.pulseCount = 1000;
    config.startSpeedHz = 5000;
    config.endSpeedHz = 5000;

    TEST_ASSERT_TRUE(sequencer.start(&tx, config));

    uint32_t callbacks = 0;
    while (sequencer.isRunning())
    {
        sequencer.onTransactionDone();
        callbacks = callbacks + 1;
        TEST_ASSERT_TRUE(callbacks < 2000);
    }

    TEST_ASSERT_TRUE(sequencer.completionPending());
    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, sequencer.takeCompletionPulses());
    TEST_ASSERT_FALSE(sequencer.hadTxError());
    TEST_ASSERT_TRUE(tx.frameSizes.size() > 1);

    const Timeline timeline = reconstruct_timeline(tx.stream);
    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, timeline.risingEdges);
    TEST_ASSERT_EQUAL_UINT64(200000ULL, timeline.ticks);
}

void test_trapezoid_model_reports_all_segments_and_exact_step_count(void)
{
    PulseEngineRmtModel model = {};

    PulseEngineRmtModel::BuildConfig config = {};
    config.resolutionHz = 1000000;
    config.pulseHighTicks = 2;
    config.pulseCount = 200;
    config.startSpeedHz = 100;
    config.cruiseSpeedHz = 400;
    config.endSpeedHz = 100;
    config.accelHzPerSec2 = 2000;
    config.useTrapezoid = true;

    TEST_ASSERT_TRUE(model.begin(config));

    uint32_t pulses = 0;
    uint32_t accelSteps = 0;
    uint32_t cruiseSteps = 0;
    uint32_t decelSteps = 0;

    std::vector<PulseEngineRmtSymbol> symbols = {};

    PulseEngineRmtFrame frame = {};
    while (model.buildNextFrame(&frame))
    {
        pulses = pulses + frame.pulseCount;
        accelSteps = accelSteps + frame.accelSteps;
        cruiseSteps = cruiseSteps + frame.cruiseSteps;
        decelSteps = decelSteps + frame.decelSteps;

        for (uint16_t i = 0; i < frame.symbolCount; ++i)
        {
            symbols.push_back(frame.symbols[i]);
        }
    }

    const Timeline timeline = reconstruct_timeline(symbols);

    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, pulses);
    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, timeline.risingEdges);
    TEST_ASSERT_TRUE(accelSteps > 0);
    TEST_ASSERT_TRUE(cruiseSteps > 0);
    TEST_ASSERT_TRUE(decelSteps > 0);
}

void test_model_respects_frame_max_pulses_limit(void)
{
    PulseEngineRmtModel model = {};

    PulseEngineRmtModel::BuildConfig config = {};
    config.resolutionHz = 1000000;
    config.pulseHighTicks = 2;
    config.pulseCount = 103;
    config.startSpeedHz = 10000;
    config.endSpeedHz = 10000;
    config.frameMaxPulses = 20;

    TEST_ASSERT_TRUE(model.begin(config));

    uint32_t pulses = 0;
    uint32_t frameCount = 0;

    PulseEngineRmtFrame frame = {};
    while (model.buildNextFrame(&frame))
    {
        TEST_ASSERT_TRUE(frame.pulseCount <= config.frameMaxPulses);
        pulses = pulses + frame.pulseCount;
        frameCount = frameCount + 1;
    }

    TEST_ASSERT_EQUAL_UINT32(config.pulseCount, pulses);
    TEST_ASSERT_TRUE(frameCount >= 6);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_model_splits_long_low_duration_without_losing_ticks);
    RUN_TEST(test_sequencer_chains_frames_and_finishes_exact_pulse_count);
    RUN_TEST(test_trapezoid_model_reports_all_segments_and_exact_step_count);
    RUN_TEST(test_model_respects_frame_max_pulses_limit);
    return UNITY_END();
}
