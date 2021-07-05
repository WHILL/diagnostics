#pragma once

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <unordered_map>

#include "diagnostic_updater_ext/headerless_topic_diag.h"

namespace diagnostic_generic_diagnostics
{
struct TopicStatusParam
{
  TopicStatusParam() : topic(""), hardware_id("anonymous"), fparam() {}

  std::string topic;
  std::string hardware_id;
  diagnostic_updater_ext::FrequencyStatusParam fparam;
};

bool parseTopicStatus(XmlRpc::XmlRpcValue & values, TopicStatusParam & param)
{
  ROS_DEBUG("parsing param...");
  if (values.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    if (values["topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.topic = static_cast<std::string>(values["topic"]);
    else
      return false;  // topic name is required

    if (values["hardware_id"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.hardware_id = static_cast<std::string>(values["hardware_id"]);

    if (values["min_freq"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.fparam.min_freq_ = static_cast<double>(values["min_freq"]);
    if (values["max_freq"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.fparam.max_freq_ = static_cast<double>(values["max_freq"]);
    if (values["tolerance"].getType() == XmlRpc::XmlRpcValue::TypeDouble)
      param.fparam.tolerance_ = static_cast<double>(values["tolerance"]);
    if (values["window_size"].getType() == XmlRpc::XmlRpcValue::TypeInt)
      param.fparam.window_size_ = static_cast<int>(values["window_size"]);
    if (values["keycode"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.fparam.keycode_ = static_cast<std::string>(values["keycode"]);

    return true;
  }
  return false;
}

using UpdaterPtr = std::shared_ptr<diagnostic_updater::Updater>;
using TopicStatusParamPtr = std::shared_ptr<TopicStatusParam>;
class TopicMonitor
{
private:
  std::vector<ros::Subscriber> subs_;

  void callback(
    const ros::MessageEvent<topic_tools::ShapeShifter> & msg,
    std::shared_ptr<diagnostic_updater::Updater> updater,
    std::shared_ptr<diagnostic_updater_ext::HeaderlessTopicDiagnostic> task)
  {
    task->tick();
    updater->update();
  }

public:
  TopicMonitor(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  {
    ROS_DEBUG("Starting TopicMonitor...");

    XmlRpc::XmlRpcValue topics;
    priv_nh.getParam("topics", topics);
    ROS_ASSERT(topics.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(topics.size() > 0);
    for (int i = 0; i < topics.size(); ++i) {
      ROS_DEBUG("Reading %dth topic...", i);
      TopicStatusParam param{};
      if (parseTopicStatus(topics[i], param)) {
        auto updater = std::make_shared<diagnostic_updater::Updater>();
        updater->setHardwareID(param.hardware_id);

        auto watcher = std::make_shared<diagnostic_updater_ext::HeaderlessTopicDiagnostic>(
          param.topic, *updater, param.fparam);
        auto sub = nh.subscribe<topic_tools::ShapeShifter>(
          param.topic, 50,
          boost::bind(
            &diagnostic_generic_diagnostics::TopicMonitor::callback, this, _1, updater, watcher));

        ROS_DEBUG("Setup sub for %s", param.topic.c_str());
        subs_.push_back(sub);
      }
    }
  }
};
};  // namespace diagnostic_generic_diagnostics
