#include <unity.h>

#include <vector>

#include "../../../support/pulse_engine_rmt/PulseEngineRmtModel.h"
#include "../../../support/pulse_engine_rmt/PulseEngineRmtSequencer.h"

#include "../../../support/pulse_engine_rmt/PulseEngineTrapezoid.cpp"
#include "../../../support/pulse_engine_rmt/PulseEngineRmtModel.cpp"
#include "../../../support/pulse_engine_rmt/PulseEngineRmtSequencer.cpp"

namespace {
class FakeRmtTx final : public PulseEngineRmtSequencer::ITx
{
public:
    bool queueTransaction(const PulseEngineRmtSymbol *symbols, size_t symbolCount) override
    {
        if (symbols == nullptr || symbolCount == 0) return false;
        queuedFrames.push_back(symbolCount);
        return true;
    }

    std::vector<size_t> queuedFrames = {};
};
} // namespace

void test_sequencer_recovers_from_abort_and_restarts_immediately(void)
{
    FakeRmtTx tx = {};
    PulseEngineRmtSequencer sequencer = {};

    PulseEngineRmtModel::BuildConfig config = {};
    config.resolutionHz = 1000000;
    config.pulseHighTicks = 2;
    config.pulseCount = 600;
    config.startSpeedHz = 8000;
    config.endSpeedHz = 8000;

    for (uint32_t cycle = 0; cycle < 100; ++cycle)
    {
        TEST_ASSERT_TRUE(sequencer.start(&tx, config));
        sequencer.onTransactionDone();
        sequencer.onTransactionDone();

        const uint32_t aborted = sequencer.abort();
        TEST_ASSERT_TRUE(aborted <= config.pulseCount);
        TEST_ASSERT_FALSE(sequencer.isRunning());

        TEST_ASSERT_TRUE(sequencer.start(&tx, config));
        while (sequencer.isRunning())
        {
            sequencer.onTransactionDone();
        }
        TEST_ASSERT_TRUE(sequencer.completionPending());
        TEST_ASSERT_EQUAL_UINT32(config.pulseCount, sequencer.takeCompletionPulses());
        TEST_ASSERT_FALSE(sequencer.hadTxError());
    }
}

void test_sequencer_stays_stable_with_bursty_tx_done_delivery(void)
{
    FakeRmtTx tx = {};
    PulseEngineRmtSequencer sequencer = {};

    PulseEngineRmtModel::BuildConfig config = {};
    config.resolutionHz = 1000000;
    config.pulseHighTicks = 2;
    config.pulseCount = 240;
    config.startSpeedHz = 10000;
    config.endSpeedHz = 10000;
    config.frameMaxPulses = 20;

    for (uint32_t cycle = 0; cycle < 50; ++cycle)
    {
        TEST_ASSERT_TRUE(sequencer.start(&tx, config));

        uint32_t callbacks = 0;
        while (sequencer.isRunning())
        {
            const uint32_t burst = (cycle % 3U) + 1U;
            for (uint32_t i = 0; i < burst && sequencer.isRunning(); ++i)
            {
                sequencer.onTransactionDone();
                callbacks = callbacks + 1U;
                TEST_ASSERT_TRUE(callbacks < 200U);
            }
        }

        TEST_ASSERT_TRUE(sequencer.completionPending());
        TEST_ASSERT_EQUAL_UINT32(config.pulseCount, sequencer.takeCompletionPulses());
        TEST_ASSERT_FALSE(sequencer.hadTxError());
    }
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_sequencer_recovers_from_abort_and_restarts_immediately);
    RUN_TEST(test_sequencer_stays_stable_with_bursty_tx_done_delivery);
    return UNITY_END();
}
