#pragma once

#include <stddef.h>
#include <stdint.h>

#include "wifi_result_utils.h"

struct ChannelScheduleEntry {
  uint8_t band;
  uint8_t channel;
};

struct ChannelScheduleState {
  size_t next_24g_index;
  size_t next_5g_index;
  uint8_t dispatch_phase;
  bool wrapped_24g;
  bool wrapped_5g;
};

static constexpr uint8_t CHANNEL_SCHEDULE_PHASE_24G = 0;
static constexpr uint8_t CHANNEL_SCHEDULE_PHASE_5G_A = 1;
static constexpr uint8_t CHANNEL_SCHEDULE_PHASE_5G_B = 2;
static constexpr uint8_t CHANNEL_SCHEDULE_PHASE_COUNT = 3;

// Keep adjacent 2.4GHz assignments at least five channel numbers apart so
// sequential 20MHz scans do not land on overlapping 2.4GHz channels.
static constexpr uint8_t CHANNEL_SCHEDULE_24G[] = {
  1, 6, 11, 2, 7, 12, 3, 8, 13, 4, 9, 14, 5, 10
};

static constexpr uint8_t CHANNEL_SCHEDULE_5G[] = {
  36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112, 116, 120, 124,
  128, 132, 136, 140, 144, 149, 153, 157, 161, 165
};

static inline size_t channelSchedule24gCount() {
  return sizeof(CHANNEL_SCHEDULE_24G) / sizeof(CHANNEL_SCHEDULE_24G[0]);
}

static inline size_t channelSchedule5gCount() {
  return sizeof(CHANNEL_SCHEDULE_5G) / sizeof(CHANNEL_SCHEDULE_5G[0]);
}

static inline uint8_t channelSchedule24gAt(size_t index) {
  return CHANNEL_SCHEDULE_24G[index];
}

static inline uint8_t channelSchedule5gAt(size_t index) {
  return CHANNEL_SCHEDULE_5G[index];
}

static inline bool channelScheduleCurrentUses24g(const ChannelScheduleState& state) {
  return state.dispatch_phase == CHANNEL_SCHEDULE_PHASE_24G;
}

static inline ChannelScheduleEntry channelScheduleCurrent(const ChannelScheduleState& state) {
  ChannelScheduleEntry entry = {};
  if (channelScheduleCurrentUses24g(state)) {
    entry.band = WIFI_BAND_24_GHZ;
    entry.channel = channelSchedule24gAt(state.next_24g_index);
    return entry;
  }

  entry.band = WIFI_BAND_5_GHZ;
  entry.channel = channelSchedule5gAt(state.next_5g_index);
  return entry;
}

static inline bool channelScheduleAdvance(ChannelScheduleState& state) {
  if (channelScheduleCurrentUses24g(state)) {
    state.next_24g_index++;
    if (state.next_24g_index >= channelSchedule24gCount()) {
      state.next_24g_index = 0;
      state.wrapped_24g = true;
    }
  } else {
    state.next_5g_index++;
    if (state.next_5g_index >= channelSchedule5gCount()) {
      state.next_5g_index = 0;
      state.wrapped_5g = true;
    }
  }

  state.dispatch_phase = (uint8_t)((state.dispatch_phase + 1u) % CHANNEL_SCHEDULE_PHASE_COUNT);
  if (state.wrapped_24g && state.wrapped_5g) {
    state.wrapped_24g = false;
    state.wrapped_5g = false;
    return true;
  }
  return false;
}
