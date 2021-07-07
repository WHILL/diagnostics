#pragma once

#include <diagnostic_updater/update_functions.h>

#include <algorithm>
#include <mutex>

#include "diagnostic_updater_ext/headerless_topic_diag.h"

namespace diagnostic_updater_ext
{
struct TimeStampStatusParam
{
  TimeStampStatusParam(
    double min_acceptable = -1., double max_acceptable = 5., std::string keycode = "")
  : max_acceptable_(max_acceptable), min_acceptable_(min_acceptable), keycode_(keycode)
  {
  }

  double max_acceptable_;
  double min_acceptable_;
  std::string keycode_;
};

/**
 * \brief Diagnostic task to monitor the interval between events.
 *
 * This diagnostic task monitors the difference between consecutive events,
 * and creates corresponding diagnostics. An error occurs if the interval
 * between consecutive events is too large or too small. An error condition
 * will only be reported during a single diagnostic report unless it
 * persists. Tallies of errors are also maintained to keep track of errors
 * in a more persistent way.
 */

class TimeStampStatus : public diagnostic_updater::DiagnosticTask
{
public:
  /**
   * \brief Constructs the TimeStampStatus with the given parameters.
   */

  TimeStampStatus(const TimeStampStatusParam & params, std::string name = "Timestamp Status")
  : DiagnosticTask(name),
    params_(params),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR),
    early_count_(0),
    late_count_(0),
    zero_count_(0),
    zero_seen_(false),
    max_delta_(0),
    min_delta_(0),
    deltas_valid_(false)
  {
  }

  int get_status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_status_;
  }

  /**
   * \brief Signals an event. Timestamp stored as a double.
   *
   * \param stamp The timestamp of the event that will be used in computing
   * intervals.
   */
  void tick(double stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stamp == 0) {
      zero_seen_ = true;
    } else {
      double delta = ros::Time::now().toSec() - stamp;

      if (!deltas_valid_ || delta > max_delta_) max_delta_ = delta;

      if (!deltas_valid_ || delta < min_delta_) min_delta_ = delta;

      deltas_valid_ = true;
    }
  }

  /**
   * \brief Signals an event.
   *
   * \param t The timestamp of the event that will be used in computing
   * intervals.
   */

  void tick(const ros::Time t) { tick(t.toSec()); }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
    stat.summary(latest_status_, "Timestamps are reasonable.");
    if (!deltas_valid_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "No data since last update.");
    } else {
      if (min_delta_ < params_.min_acceptable_) {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Timestamps too far in future seen.");
        early_count_++;
      }

      if (max_delta_ > params_.max_acceptable_) {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Timestamps too far in past seen.");
        late_count_++;
      }

      if (zero_seen_) {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Zero timestamp seen.");
        zero_count_++;
      }
    }

    stat.addf("Earliest timestamp delay:", "%f", min_delta_);
    stat.addf("Latest timestamp delay:", "%f", max_delta_);
    stat.addf("Earliest acceptable timestamp delay:", "%f", params_.min_acceptable_);
    stat.addf("Latest acceptable timestamp delay:", "%f", params_.max_acceptable_);
    stat.add("Late diagnostic update count:", late_count_);
    stat.add("Early diagnostic update count:", early_count_);
    stat.add("Zero seen diagnostic update count:", zero_count_);

    deltas_valid_ = false;
    min_delta_ = 0;
    max_delta_ = 0;
    zero_seen_ = false;
  }

private:
  const TimeStampStatusParam params_;
  mutable std::mutex mutex_;
  int latest_status_;
  int early_count_;
  int late_count_;
  int zero_count_;
  bool zero_seen_;
  double max_delta_;
  double min_delta_;
  bool deltas_valid_;
};

/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus and TimeStampStatus.
 */

class TopicDiagnostic : public HeaderlessTopicDiagnostic
{
public:
  /**
   * \brief Constructs a TopicDiagnostic.
   *
   * \param name The name of the topic that is being diagnosed.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   *
   * \param stamp The parameters for the TimeStampStatus class that will be
   * computing statistics.
   */
  TopicDiagnostic(
    std::string name, diagnostic_updater::Updater & diag, const FrequencyStatusParam & freq,
    const TimeStampStatusParam & stamp)
  : HeaderlessTopicDiagnostic(name, diag, freq), stamp_(stamp)
  {
    this->addTask(&stamp_);
  }

  virtual ~TopicDiagnostic() {}

  /**
   * This method should never be called on a TopicDiagnostic as a timestamp
   * is needed to collect the timestamp diagnostics. It is defined here to
   * prevent the inherited tick method from being used accidentally.
   */
  virtual void tick()
  {
    ROS_FATAL(
      "tick(void) has been called on a TopicDiagnostic. This is never correct. Use tick(ros::Time "
      "&) instead.");
  }

  /**
   * \brief Collects statistics and publishes the message.
   *
   * \param stamp Timestamp to use for interval computation by the
   * TimeStampStatus class.
   */
  virtual void tick(const ros::Time & stamp)
  {
    stamp_.tick(stamp);
    HeaderlessTopicDiagnostic::tick();
  }

  int get_status()
  {
    int status_freq = HeaderlessTopicDiagnostic::get_status();
    int status_stamp = stamp_.get_status();
    return std::max(status_freq, status_stamp);
  }

private:
  TimeStampStatus stamp_;
};

}  // namespace diagnostic_updater_ext
