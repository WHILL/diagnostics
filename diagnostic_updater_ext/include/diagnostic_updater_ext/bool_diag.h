#pragma once

#include <diagnostic_updater/diagnostic_updater.h>

#include <mutex>

namespace diagnostic_updater_ext
{
struct BoolStatusParam
{
  BoolStatusParam(bool publish_error = true, bool invert = false, std::string keycode = "")
  : publish_error_(publish_error), invert_(invert), keycode_(keycode)
  {
  }

  /**
   * \brief Publish ERROR if true, otherwise publish WARN
   */
  bool publish_error_;
  bool invert_;
  std::string keycode_;
};

class BoolStatus : public diagnostic_updater::DiagnosticTask
{
private:
  const BoolStatusParam params_;
  mutable std::mutex mutex_;
  int latest_status_;
  bool is_success_;
  int success_count_;
  int fail_count_;

public:
  /**
   * \brief Constructs a Bool Checker
   */
  BoolStatus(const BoolStatusParam & params, std::string name = "Bool Status")
  : DiagnosticTask(name),
    params_(params),
    latest_status_(diagnostic_msgs::DiagnosticStatus::ERROR),
    is_success_(false),
    success_count_(0),
    fail_count_(0)
  {
  }

  void set(bool is_success)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_success_ = params_.invert_ ? !is_success : is_success;
    if (is_success_) {
      success_count_++;
      fail_count_ = 0;
    } else {
      success_count_ = 0;
      fail_count_++;
    }
  }

  int get_status() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_status_;
  }

  virtual void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (is_success_) {
      latest_status_ = diagnostic_msgs::DiagnosticStatus::OK;
      stat.summary(latest_status_, "Latest process was successfully completed.");
      stat.add("Successfull update count:", success_count_);
    } else {
      latest_status_ = (params_.publish_error_) ? diagnostic_msgs::DiagnosticStatus::ERROR
                                                : diagnostic_msgs::DiagnosticStatus::WARN;
      stat.summary(latest_status_, "Latest process failed.");
      stat.add("Failed update count:", fail_count_);
    }

    if (!params_.keycode_.empty() && latest_status_ == diagnostic_msgs::DiagnosticStatus::ERROR) {
      stat.add("KEYCODE", params_.keycode_);
    }
  }
};

class BoolDiagnostic : public diagnostic_updater::CompositeDiagnosticTask
{
public:
  BoolDiagnostic(
    std::string name, diagnostic_updater::Updater & diag, const BoolStatusParam & bool_diag,
    std::string keycode = "")
  : CompositeDiagnosticTask(name + " bool_diag status"), bool_diag_(bool_diag, keycode)
  {
    this->addTask(&bool_diag_);
    diag.add(*this);
  }

  virtual ~BoolDiagnostic() {}

  void set(bool result) { bool_diag_.set(result); }

  int get_status() { return bool_diag_.get_status(); }

private:
  BoolStatus bool_diag_;
};
}  // namespace diagnostic_updater_ext
