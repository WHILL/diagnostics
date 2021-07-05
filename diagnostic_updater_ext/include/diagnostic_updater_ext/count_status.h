#pragma once

#include <diagnostic_updater/diagnostic_updater.h>

#include <mutex>

namespace diagnostic_updater_ext
{
struct CountStatusParam
{
  CountStatusParam(int warn_threshold, int error_threshold)
  : warn_threshold_(warn_threshold), error_threshold_(error_threshold)
  {
    if (warn_threshold_ > error_threshold_) {
      ROS_WARN("Error threshold must be grater than or equal to warn threshold.");
      error_threshold_ = warn_threshold_;
    }
  }

  int warn_threshold_;
  int error_threshold_;
};

class CountStatus : public diagnostic_updater::DiagnosticTask
{
private:
  const CountStatusParam params_;
  std::mutex mutex_;
  int latest_status_;
  int count_;

public:
  CountStatus(const CountStatusParam & params, const std::string & name = "Count Status")
  : DiagnosticTask(name),
    params_(params),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR),
    count_(0)
  {
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    count_ = 0;
  }

  int get_status()
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
    if (count_ >= params_.error_threshold_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::ERROR;
      stat.summary(latest_status_, "Count exceeds error limit.");
    } else if (count_ >= params_.warn_threshold_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Count exceeds warn limit.");
    } else {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Count is in nominal range.");
    }
    stat.add("Count:", count_);
    stat.add("Warn Threshold:", params_.warn_threshold_);
    stat.add("Error Threshold:", params_.error_threshold_);
  }
};

class CountDiagnostic : public diagnostic_updater::CompositeDiagnosticTask
{
public:
  CountDiagnostic(
    std::string name, diagnostic_updater::Updater & diag, const CountStatusParam & count_param)
  : CompositeDiagnosticTask(name + " count status"), count_(count_param)
  {
    this->addTask(&count_);
    diag.add(*this);
  }

  virtual ~CountDiagnostic() {}

  virtual void tick() { count_.tick(); }

  virtual void clear() { count_.clear(); }

  virtual int get_status() { return count_.get_status(); }

private:
  CountStatus count_;
};

}  // namespace diagnostic_updater_ext
