#include <stddef.h>
#include <stdint.h>

#include <unity.h>

#include "channel_scheduler.h"

void setUp() {}
void tearDown() {}

void test_24g_schedule_keeps_adjacent_channels_non_overlapping() {
  TEST_ASSERT_EQUAL_UINT32(14, channelSchedule24gCount());

  const uint8_t expected[] = {1, 6, 11, 2, 7, 12, 3, 8, 13, 4, 9, 14, 5, 10};
  for (size_t i = 0; i < channelSchedule24gCount(); i++) {
    TEST_ASSERT_EQUAL_UINT8(expected[i], channelSchedule24gAt(i));
    if ((i + 1u) < channelSchedule24gCount()) {
      const int delta = (int)channelSchedule24gAt(i + 1u) - (int)channelSchedule24gAt(i);
      TEST_ASSERT_TRUE((delta >= 5) || (delta <= -5));
    }
  }
}

void test_schedule_dispatches_24g_then_two_5g_channels() {
  ChannelScheduleState state = {};

  const ChannelScheduleEntry a = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_24_GHZ, a.band);
  TEST_ASSERT_EQUAL_UINT8(1, a.channel);

  TEST_ASSERT_FALSE(channelScheduleAdvance(state));
  const ChannelScheduleEntry b = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, b.band);
  TEST_ASSERT_EQUAL_UINT8(36, b.channel);

  TEST_ASSERT_FALSE(channelScheduleAdvance(state));
  const ChannelScheduleEntry c = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, c.band);
  TEST_ASSERT_EQUAL_UINT8(40, c.channel);

  TEST_ASSERT_FALSE(channelScheduleAdvance(state));
  const ChannelScheduleEntry d = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_24_GHZ, d.band);
  TEST_ASSERT_EQUAL_UINT8(6, d.channel);

  TEST_ASSERT_FALSE(channelScheduleAdvance(state));
  const ChannelScheduleEntry e = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, e.band);
  TEST_ASSERT_EQUAL_UINT8(44, e.channel);
}

void test_channel_schedule_current_uses_phase_and_indices() {
  ChannelScheduleState state = {};
  state.next_24g_index = 3;
  state.next_5g_index = 4;

  state.dispatch_phase = CHANNEL_SCHEDULE_PHASE_24G;
  ChannelScheduleEntry entry = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_24_GHZ, entry.band);
  TEST_ASSERT_EQUAL_UINT8(channelSchedule24gAt(3), entry.channel);

  state.dispatch_phase = CHANNEL_SCHEDULE_PHASE_5G_A;
  entry = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, entry.band);
  TEST_ASSERT_EQUAL_UINT8(channelSchedule5gAt(4), entry.channel);

  state.dispatch_phase = CHANNEL_SCHEDULE_PHASE_5G_B;
  entry = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, entry.band);
  TEST_ASSERT_EQUAL_UINT8(channelSchedule5gAt(4), entry.channel);
}

void test_full_coverage_cycle_completes_after_both_bands_wrap() {
  ChannelScheduleState state = {};
  bool cycle_complete = false;
  uint32_t dispatches = 0;

  while (!cycle_complete && dispatches < 128) {
    cycle_complete = channelScheduleAdvance(state);
    dispatches++;
  }

  TEST_ASSERT_TRUE(cycle_complete);
  TEST_ASSERT_EQUAL_UINT32(40, dispatches);
  TEST_ASSERT_EQUAL_UINT8(CHANNEL_SCHEDULE_PHASE_5G_A, state.dispatch_phase);
  TEST_ASSERT_EQUAL_UINT32(0, state.next_24g_index);
  TEST_ASSERT_EQUAL_UINT32(1, state.next_5g_index);
}

void test_full_coverage_cycle_repeats_only_first_5g_channel() {
  ChannelScheduleState state = {};
  bool cycle_complete = false;
  uint32_t dispatches = 0;
  uint8_t hits_24g[15] = {};
  uint8_t hits_5g[166] = {};

  while (!cycle_complete && dispatches < 128) {
    const ChannelScheduleEntry entry = channelScheduleCurrent(state);
    if (entry.band == WIFI_BAND_24_GHZ) {
      hits_24g[entry.channel]++;
    } else {
      hits_5g[entry.channel]++;
    }
    cycle_complete = channelScheduleAdvance(state);
    dispatches++;
  }

  TEST_ASSERT_TRUE(cycle_complete);
  TEST_ASSERT_EQUAL_UINT32(40, dispatches);
  for (size_t i = 0; i < channelSchedule24gCount(); i++) {
    TEST_ASSERT_EQUAL_UINT8(1, hits_24g[channelSchedule24gAt(i)]);
  }
  for (size_t i = 0; i < channelSchedule5gCount(); i++) {
    const uint8_t channel = channelSchedule5gAt(i);
    const uint8_t expected_hits = (channel == 36) ? 2 : 1;
    TEST_ASSERT_EQUAL_UINT8(expected_hits, hits_5g[channel]);
  }
}

void test_next_cycle_continues_from_post_wrap_state() {
  ChannelScheduleState state = {};
  bool cycle_complete = false;
  uint32_t dispatches = 0;

  while (!cycle_complete && dispatches < 128) {
    cycle_complete = channelScheduleAdvance(state);
    dispatches++;
  }

  TEST_ASSERT_TRUE(cycle_complete);
  TEST_ASSERT_FALSE(state.wrapped_24g);
  TEST_ASSERT_FALSE(state.wrapped_5g);

  ChannelScheduleEntry entry = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, entry.band);
  TEST_ASSERT_EQUAL_UINT8(40, entry.channel);

  TEST_ASSERT_FALSE(channelScheduleAdvance(state));
  entry = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_5_GHZ, entry.band);
  TEST_ASSERT_EQUAL_UINT8(44, entry.channel);

  TEST_ASSERT_FALSE(channelScheduleAdvance(state));
  entry = channelScheduleCurrent(state);
  TEST_ASSERT_EQUAL_UINT8(WIFI_BAND_24_GHZ, entry.band);
  TEST_ASSERT_EQUAL_UINT8(1, entry.channel);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_24g_schedule_keeps_adjacent_channels_non_overlapping);
  RUN_TEST(test_schedule_dispatches_24g_then_two_5g_channels);
  RUN_TEST(test_channel_schedule_current_uses_phase_and_indices);
  RUN_TEST(test_full_coverage_cycle_completes_after_both_bands_wrap);
  RUN_TEST(test_full_coverage_cycle_repeats_only_first_5g_channel);
  RUN_TEST(test_next_cycle_continues_from_post_wrap_state);
  return UNITY_END();
}
