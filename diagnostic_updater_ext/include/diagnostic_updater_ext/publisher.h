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

#include <ros/publisher.h>
#include <ros/subscription.h>

#include "diagnostic_updater_ext/update_functions.h"

namespace diagnostic_updater_ext
{
class CountDiagnostic : public CompositeDiagnosticTask
{
public:
  CountDiagnostic(std::string name, Updater & diag, const CountStatusParam & count)
  : CompositeDiagnosticTask(name + " count status"), count_(count)
  {
    addTask(&count_);
    diag.add(*this);
  }

  virtual ~CountDiagnostic() {}

  virtual void tick() { count_.tick(); }

  virtual void clear() { count_.clear(); }

  virtual int get_status() { return count_.get_status(); }

private:
  CountStatus count_;
};

class BoundDiagnostic : public CompositeDiagnosticTask
{
public:
  BoundDiagnostic(std::string name, Updater & diag, const BoundStatusParam & bound)
  : CompositeDiagnosticTask(name + " bound status"), bound_(bound)
  {
    addTask(&bound_);
    diag.add(*this);
  }

  virtual ~BoundDiagnostic() {}

  virtual void set(int value) { bound_.set(value); }

  virtual void clear() { bound_.clear(); }

  virtual int get_status() { return bound_.get_status(); }

private:
  BoundStatus bound_;
};

class BoolDiagnostic : public CompositeDiagnosticTask
{
public:
  BoolDiagnostic(
    std::string name, Updater & diag, const BoolStatusParam & bool_diag,
    const std::vector<CustomField> & fields)
  : CompositeDiagnosticTask(name + " bool_diag status"), bool_diag_(bool_diag, fields)
  {
    addTask(&bool_diag_);
    diag.add(*this);
  }

  BoolDiagnostic(std::string name, Updater & diag, const BoolStatusParam & bool_diag)
  : CompositeDiagnosticTask(name + " bool_diag status"), bool_diag_(bool_diag)
  {
    addTask(&bool_diag_);
    diag.add(*this);
  }

  virtual ~BoolDiagnostic() {}

  virtual void set(bool result) { bool_diag_.set(result); }

  virtual void clear() { bool_diag_.clear(); }

  virtual int get_status() { return bool_diag_.get_status(); }

private:
  BoolStatus bool_diag_;
};
/**
 * \brief A class to facilitate making diagnostics for a topic using a
 * FrequencyStatus.
 *
 * The word "headerless" in the class name refers to the fact that it is
 * mainly designed for use with messages that do not have a header, and
 * that cannot therefore be checked using a TimeStampStatus.
 */

class HeaderlessTopicDiagnostic : public CompositeDiagnosticTask
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
    std::string name, Updater & diag, const FrequencyStatusParam & freq,
    const std::vector<CustomField> & fields)
  : CompositeDiagnosticTask(name + " topic status"), freq_(freq, fields)
  {
    addTask(&freq_);
    diag.add(*this);
  }

  HeaderlessTopicDiagnostic(std::string name, Updater & diag, const FrequencyStatusParam & freq)
  : CompositeDiagnosticTask(name + " topic status"), freq_(freq)
  {
    addTask(&freq_);
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

  int get_status() { return freq_.get_status(); }

private:
  FrequencyStatus freq_;
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
    std::string name, Updater & diag, const FrequencyStatusParam & freq,
    const TimeStampStatusParam & stamp, const std::vector<CustomField> & fields)
  : HeaderlessTopicDiagnostic(name, diag, freq, fields), stamp_(stamp)
  {
    addTask(&stamp_);
  }

  TopicDiagnostic(
    std::string name, Updater & diag, const FrequencyStatusParam & freq,
    const TimeStampStatusParam & stamp)
  : HeaderlessTopicDiagnostic(name, diag, freq), stamp_(stamp)
  {
    addTask(&stamp_);
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
    return (status_freq > status_stamp ? status_freq : status_stamp);
  }

private:
  TimeStampStatus stamp_;
};

};  // namespace diagnostic_updater_ext
