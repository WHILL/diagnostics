/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <diagnostic_updater/update_functions.h>

#include <cmath>

#include "diagnostic_updater_ext/diagnostic_updater_wrapper.h"

namespace diagnostic_updater_ext
{
/**
 * \brief A structure that holds the custom field parameters.
 */
struct CustomField
{
  std::string key;
  std::string value;
  int level;  // OK:1, WARN:2, ERROR:4 and sum of the error level you want to show
};

static void addCustomFields(
  DiagnosticStatusWrapper & stat, const int & status, const std::vector<CustomField> & fields)
{
  for (const auto & field : fields) {
    const bool disp_ok = (field.level) % 2 == 1;
    const bool disp_warn = (field.level / 2) % 2 == 1;
    const bool disp_err = (field.level / 4) % 2 == 1;
    if (
      (status == diagnostic_msgs::DiagnosticStatus::OK && disp_ok) ||
      (status == diagnostic_msgs::DiagnosticStatus::WARN && disp_warn) ||
      (status == diagnostic_msgs::DiagnosticStatus::ERROR && disp_err)) {
      stat.add(field.key, field.value);
    }
  }
  return;
}

using diagnostic_updater::FrequencyStatusParam;

/**
 * \brief A diagnostic task that monitors the frequency of an event.
 *
 * This diagnostic task monitors the frequency of calls to its tick method,
 * and creates corresponding diagnostics. It will report a warning if the frequency is
 * outside acceptable bounds, and report an error if there have been no events in the latest
 * window.
 */

class FrequencyStatus : public DiagnosticTask
{
private:
  const FrequencyStatusParam params_;
  std::vector<CustomField> custom_fields_;
  int latest_status_;
  int count_;
  std::vector<ros::Time> times_;
  std::vector<int> seq_nums_;
  int hist_indx_;
  boost::mutex lock_;

public:
  /**
   * \brief Constructs a FrequencyStatus class with the given parameters.
   */

  FrequencyStatus(const FrequencyStatusParam & params, std::string name)
  : DiagnosticTask(name),
    params_(params),
    times_(params_.window_size_),
    seq_nums_(params_.window_size_),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  FrequencyStatus(
    const FrequencyStatusParam & params, const std::vector<CustomField> & custom_fields)
  : DiagnosticTask("Frequency Status"),
    params_(params),
    times_(params_.window_size_),
    seq_nums_(params_.window_size_),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
    custom_fields_ = custom_fields;
  }

  /**
   * \brief Constructs a FrequencyStatus class with the given parameters.
   *        Uses a default diagnostic task name of "Frequency Status".
   */

  FrequencyStatus(const FrequencyStatusParam & params)
  : DiagnosticTask("Frequency Status"),
    params_(params),
    times_(params_.window_size_),
    seq_nums_(params_.window_size_),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  /**
   * \brief Resets the statistics.
   */

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    ros::Time curtime = ros::Time::now();
    count_ = 0;

    for (int i = 0; i < params_.window_size_; i++) {
      times_[i] = curtime;
      seq_nums_[i] = count_;
    }

    hist_indx_ = 0;
    custom_fields_.clear();
  }

  /**
   * \brief Signals that an event has occurred.
   */
  void tick()
  {
    boost::mutex::scoped_lock lock(lock_);
    // ROS_DEBUG("TICK %i", count_);
    count_++;
  }

  virtual void run(DiagnosticStatusWrapper & stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    ros::Time curtime = ros::Time::now();
    int curseq = count_;
    int events = curseq - seq_nums_[hist_indx_];
    double window = (curtime - times_[hist_indx_]).toSec();
    double freq = events / window;
    seq_nums_[hist_indx_] = curseq;
    times_[hist_indx_] = curtime;
    hist_indx_ = (hist_indx_ + 1) % params_.window_size_;

    if (events == 0) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "No events recorded.");
    } else if (freq < *params_.min_freq_ * (1 - params_.tolerance_)) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Frequency too low to continue operation.");
    } else if (freq > *params_.max_freq_ * (1 + params_.tolerance_)) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Frequency too high to continue operation.");
    } else if (freq < *params_.min_freq_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Frequency is lower than desired.");
    } else if (freq > *params_.max_freq_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Frequency is higher than desired.");
    } else {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Desired frequency met");
    }

    addCustomFields(stat, latest_status_, custom_fields_);

    stat.addf("Events in window", "%d", events);
    stat.addf("Events since startup", "%d", count_);
    stat.addf("Duration of window (s)", "%f", window);
    stat.addf("Actual frequency (Hz)", "%f", freq);
    if (*params_.min_freq_ == *params_.max_freq_)
      stat.addf("Target frequency (Hz)", "%f", *params_.min_freq_);
    if (*params_.min_freq_ > 0) {
      stat.addf(
        "Minimum acceptable frequency (Hz)", "%f", *params_.min_freq_ * (1 - params_.tolerance_));
      stat.addf("Minimum desired frequency (Hz)", "%f", *params_.min_freq_);
    }

    if (std::isfinite(*params_.max_freq_)) {
      stat.addf(
        "Maximum acceptable frequency (Hz)", "%f", *params_.max_freq_ * (1 + params_.tolerance_));
      stat.addf("Maximum desired frequency (Hz)", "%f", *params_.max_freq_);
    }
  }
};

using diagnostic_updater::DefaultTimeStampStatusParam;
using diagnostic_updater::TimeStampStatusParam;

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

class TimeStampStatus : public DiagnosticTask
{
private:
  void init()
  {
    early_count_ = 0;
    late_count_ = 0;
    zero_count_ = 0;
    zero_seen_ = false;
    max_delta_ = 0;
    min_delta_ = 0;
    deltas_valid_ = false;
    latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
  }

public:
  /**
   * \brief Constructs the TimeStampStatus with the given parameters.
   */

  TimeStampStatus(const TimeStampStatusParam & params, std::string name)
  : DiagnosticTask(name), params_(params)
  {
    init();
  }

  /**
   * \brief Constructs the TimeStampStatus with the given parameters.
   *        Uses a default diagnostic task name of "Timestamp Status".
   */

  TimeStampStatus(const TimeStampStatusParam & params)
  : DiagnosticTask("Timestamp Status"), params_(params)
  {
    init();
  }

  /**
   * \brief Constructs the TimeStampStatus with the default parameters.
   *        Uses a default diagnostic task name of "Timestamp Status".
   */

  TimeStampStatus() : DiagnosticTask("Timestamp Status") { init(); }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
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
    boost::mutex::scoped_lock lock(lock_);

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

  virtual void run(DiagnosticStatusWrapper & stat)
  {
    boost::mutex::scoped_lock lock(lock_);

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
  TimeStampStatusParam params_;
  int early_count_;
  int late_count_;
  int zero_count_;
  bool zero_seen_;
  double max_delta_;
  double min_delta_;
  bool deltas_valid_;
  boost::mutex lock_;
  int latest_status_;
};

struct BoolStatusParam
{
  BoolStatusParam(bool publish_error = true, bool invert = false)
  : publish_error_(publish_error), invert_(invert)
  {
  }

  /**
   * \brief Publish ERROR if true, otherwise publish WARN
   */
  bool publish_error_;
  bool invert_;
};

class BoolStatus : public DiagnosticTask
{
private:
  const BoolStatusParam params_;
  std::vector<CustomField> custom_fields_;
  int latest_status_;
  bool is_success_;
  int success_count_;
  int fail_count_;
  boost::mutex lock_;

public:
  /**
   * \brief Constructs a Bool Checker
   */
  BoolStatus(const BoolStatusParam & params, std::string name = "Bool Status")
  : DiagnosticTask(name),
    params_(params),
    is_success_(false),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
  }

  BoolStatus(
    const BoolStatusParam & params, const std::vector<CustomField> & custom_fields,
    std::string name = "Bool Status")
  : DiagnosticTask(name),
    params_(params),
    is_success_(false),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    clear();
    custom_fields_ = custom_fields;
  }

  void clear()
  {
    boost::mutex::scoped_lock lock(lock_);
    success_count_ = 0;
    fail_count_ = 0;
    custom_fields_.clear();
  }

  void set(bool is_success)
  {
    boost::mutex::scoped_lock lock(lock_);
    is_success_ = params_.invert_ ? !is_success : is_success;
    if (is_success_) {
      success_count_++;
      fail_count_ = 0;
    } else {
      success_count_ = 0;
      fail_count_++;
    }
  }

  int get_status()
  {
    boost::mutex::scoped_lock lock(lock_);
    return latest_status_;
  }

  virtual void run(DiagnosticStatusWrapper & stat)
  {
    boost::mutex::scoped_lock lock(lock_);
    if (is_success_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Latest process was successfully completed.");
      stat.add("Successfull update count:", success_count_);
    } else {
      if (params_.publish_error_) {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
        stat.summary(latest_status_, "Latest process failed.");
      } else {
        latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
        stat.summary(latest_status_, "Latest process failed.");
      }
      stat.add("Failed update count:", fail_count_);
    }
    addCustomFields(stat, latest_status_, custom_fields_);
  }
};
};  // namespace diagnostic_updater_ext
