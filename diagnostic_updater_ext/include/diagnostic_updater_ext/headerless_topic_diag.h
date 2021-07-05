#pragma once

#include <diagnostic_updater/diagnostic_updater.h>

#include <mutex>

namespace diagnostic_updater_ext
{
/**
 * \brief A structure that holds the constructor parameters for the
 * FrequencyStatus class.
 */
struct FrequencyStatusParam
{
  FrequencyStatusParam(
    double min_freq = 0., double max_freq = 0., double tolerance = 0.1, int window_size = 5,
    std::string keycode = "")
  : min_freq_(min_freq),
    max_freq_(max_freq),
    tolerance_(tolerance),
    window_size_(window_size),
    keycode_(keycode)
  {
  }

  double min_freq_;  //< Minimum acceptable frequency.
  double max_freq_;  //< Maximum acceptable frequency.

  /**
   * \brief Tolerance with which bounds must be satisfied.
   *
   * Acceptable values are from min_freq_ * (1 - torelance_) to *max_freq_ *
   * (1 + tolerance_).
   *
   * Common use cases are to set tolerance_ to zero, or to assign the same
   * value to max_freq_ and min_freq_.
   */
  double tolerance_;

  int window_size_;  //< Number of events to consider in the statistics.
  std::string keycode_;
};

/**
 * \brief A diagnostic task that monitors the frequency of an event.
 *
 * This diagnostic task monitors the frequency of calls to its tick method,
 * and creates corresponding diagnostics. It will report a warning if the frequency is
 * outside acceptable bounds, and report an error if there have been no events in the latest
 * window.
 */
class FrequencyStatus : public diagnostic_updater::DiagnosticTask
{
public:
  /**
   * \brief Constructs a FrequencyStatus class with the given parameters.
   */

  FrequencyStatus(const FrequencyStatusParam & params, std::string name = "Frequency Status")
  : DiagnosticTask(name),
    params_(params),
    times_(params_.window_size_),
    seq_nums_(params_.window_size_),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR)
  {
    this->clear();
  }

  void setMinMaxFreq(double min_freq, double max_freq)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    params_.min_freq_ = min_freq;
    params_.max_freq_ = max_freq;
  }

  /**
   * \brief Resets the statistics.
   */

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ros::Time curtime = ros::Time::now();
    count_ = 0;

    for (int i = 0; i < params_.window_size_; i++) {
      times_[i] = curtime;
      seq_nums_[i] = count_;
    }

    hist_index_ = 0;
  }

  int get_status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_status_;
  }

  /**
   * \brief Signals that an event has occurred.
   */
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count_++;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ros::Time curtime = ros::Time::now();
    int events = count_ - seq_nums_[hist_index_];
    double window = (curtime - times_[hist_index_]).toSec();
    double freq = events / window;
    seq_nums_[hist_index_] = count_;
    times_[hist_index_] = curtime;
    hist_index_ = (hist_index_ + 1) % params_.window_size_;

    if (events == 0) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "No events recorded.");
    } else if (freq < params_.min_freq_ * (1 - params_.tolerance_)) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Frequency too low to continue operation.");
    } else if (freq > params_.max_freq_ * (1 + params_.tolerance_)) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Frequency too high to continue operation.");
    } else if (freq < params_.min_freq_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Frequency is lower than desired.");
    } else if (freq > params_.max_freq_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Frequency is higher than desired.");
    } else {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Desired frequency met");
    }

    if (!params_.keycode_.empty() && latest_status_ == diagnostic_msgs::DiagnosticStatus::ERROR) {
      stat.add("KEYCODE", params_.keycode_);
    }

    stat.addf("Events in window", "%d", events);
    stat.addf("Events since startup", "%d", count_);
    stat.addf("Duration of window (s)", "%f", window);
    stat.addf("Actual frequency (Hz)", "%f", freq);
    if (params_.min_freq_ == params_.max_freq_)
      stat.addf("Target frequency (Hz)", "%f", params_.min_freq_);
    if (params_.min_freq_ > 0) {
      stat.addf(
        "Minimum acceptable frequency (Hz)", "%f", params_.min_freq_ * (1 - params_.tolerance_));
      stat.addf("Minimum desired frequency (Hz)", "%f", params_.min_freq_);
    }

    if (std::isfinite(params_.max_freq_)) {
      stat.addf(
        "Maximum acceptable frequency (Hz)", "%f", params_.max_freq_ * (1 + params_.tolerance_));
      stat.addf("Maximum desired frequency (Hz)", "%f", params_.max_freq_);
    }
  }

private:
  FrequencyStatusParam params_;
  mutable std::mutex mutex_;
  int latest_status_;
  int count_;
  std::vector<ros::Time> times_;
  std::vector<int> seq_nums_;
  int hist_index_;
};

/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus.
 *
 * The word "headerless" in the class name refers to the fact that it is
 * mainly designed for use with messages that do not have a header, and
 * that cannot therefore be checked using a TimeStampStatus.
 */

class HeaderlessTopicDiagnostic : public diagnostic_updater::CompositeDiagnosticTask
{
public:
  /**
   * \brief Constructs a HeaderlessTopicDiagnostic.
   *
   * \param name The name of the topic that is being diagnosed.
   *
   * \param diag The diagnostic_updater that the CompositeDiagnosticTask
   * should add itself to.
   *
   * \param freq The parameters for the FrequencyStatus class that will be
   * computing statistics.
   */
  HeaderlessTopicDiagnostic(
    std::string name, diagnostic_updater::Updater & diag, const FrequencyStatusParam & freq_param)
  : CompositeDiagnosticTask(name + " topic status"), freq_(freq_param)
  {
    this->addTask(&freq_);
    diag.add(*this);
  }

  virtual ~HeaderlessTopicDiagnostic() {}

  /**
   * \brief Signals that a publication has occurred.
   */
  virtual void tick() { freq_.tick(); }

  /**
   * \brief Clears the frequency statistics.
   */
  virtual void clear_window() { freq_.clear(); }

  virtual void setMinMaxFreq(double min_freq, double max_freq)
  {
    freq_.setMinMaxFreq(min_freq, max_freq);
  }

  int get_status() { return freq_.get_status(); }

private:
  FrequencyStatus freq_;
};

}  // namespace diagnostic_updater_ext
