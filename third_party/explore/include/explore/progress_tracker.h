// Progress policy for the mapping integration of explore_lite.
#ifndef EXPLORE_PROGRESS_TRACKER_H
#define EXPLORE_PROGRESS_TRACKER_H
#include <algorithm>
#include <cmath>
#include <limits>

namespace explore {
// High-water marks reject oscillation and require actual translation before a
// shortened/replanned Nav2 path can count as progress. All times are ROS time.
class ProgressTracker {
public:
  void reset(double now, double x, double y, double distance) {
    started = last_progress = now;
    reference_x = x; reference_y = y; best_distance = distance;
    best_remaining = std::numeric_limits<double>::infinity();
  }
  bool update(double now, double x, double y, double distance,
              double remaining, double threshold) {
    if (now < last_progress) { reset(now, x, y, distance); return false; }
    if (std::isfinite(remaining) && !std::isfinite(best_remaining))
      best_remaining = remaining;
    const bool moved = std::hypot(x - reference_x, y - reference_y) >= threshold;
    const bool closer = best_distance - distance >= threshold;
    const bool path_progress = std::isfinite(remaining) && best_remaining - remaining >= threshold;
    if (!moved || (!closer && !path_progress)) return false;
    best_distance = std::min(best_distance, distance);
    if (std::isfinite(remaining)) best_remaining = std::min(best_remaining, remaining);
    reference_x = x; reference_y = y; last_progress = now;
    return true;
  }
  bool stalled(double now, double timeout) const { return now - last_progress >= timeout; }
  bool expired(double now, double timeout) const { return timeout > 0 && now - started >= timeout; }
  double started{0}, last_progress{0};
private:
  double reference_x{0}, reference_y{0}, best_distance{0};
  double best_remaining{std::numeric_limits<double>::infinity()};
};
}
#endif
