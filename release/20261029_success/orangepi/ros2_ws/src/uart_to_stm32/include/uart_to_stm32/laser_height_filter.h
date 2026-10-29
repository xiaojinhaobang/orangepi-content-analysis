#ifndef UART_TO_STM32_LASER_HEIGHT_FILTER_H
#define UART_TO_STM32_LASER_HEIGHT_FILTER_H
#include <algorithm>
#include <array>
#include <cstdint>

// Median of three fresh 50 Hz samples; isolated spikes do not reach /height.
// A gap over 200 ms restarts warmup, without replaying an old height.
class LaserHeightFilter {
 public:
  bool update(int16_t value, int64_t now_ns, int16_t &output) {
    if (count_ && (now_ns <= last_ns_ || now_ns - last_ns_ > 200000000LL)) {
      count_ = 0;
      next_ = 0;
    }
    last_ns_ = now_ns;
    samples_[next_] = value;
    next_ = (next_ + 1) % 3;
    if (count_ < 3) ++count_;
    if (count_ < 3) return false;
    auto sorted = samples_;
    std::sort(sorted.begin(), sorted.end());
    output = sorted[1];
    return true;
  }
 private:
  std::array<int16_t, 3> samples_{};
  unsigned count_{0}, next_{0};
  int64_t last_ns_{0};
};
#endif
