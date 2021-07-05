#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <topic_tools/shape_shifter.h>

#include "diagnostic_updater_ext/bool_diag.h"

namespace diagnostic_generic_diagnostics
{
struct BoolStatusParam
{
  BoolStatusParam() : topic(""), hardware_id(""), bparam() {}

  std::string topic;
  std::string hardware_id;
  diagnostic_updater_ext::BoolStatusParam bparam;
};

bool parseTopicStatus(XmlRpc::XmlRpcValue & values, BoolStatusParam & param)
{
  ROS_DEBUG("parsing param...");
  if (values.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    if (values["topic"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.topic = static_cast<std::string>(values["topic"]);
    else
      return false;  // topic name is required

    if (values["hardware_id"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.hardware_id = static_cast<std::string>(values["hardware_id"]);

    if (values["publish_error"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      param.bparam.publish_error_ = static_cast<bool>(values["publish_error"]);
    if (values["invert"].getType() == XmlRpc::XmlRpcValue::TypeBoolean)
      param.bparam.invert_ = static_cast<bool>(values["invert"]);
    if (values["keycode"].getType() == XmlRpc::XmlRpcValue::TypeString)
      param.bparam.keycode_ = static_cast<std::string>(values["keycode"]);

    return true;
  }
  return false;
}

class BoolMonitor
{
private:
  ros::NodeHandle nh_, pnh_;
  std::vector<ros::Subscriber> subs_;
  std::vector<ros::Timer> timers_;

  void callback(
    const std_msgs::BoolConstPtr & msg, std::shared_ptr<diagnostic_updater::Updater> updater,
    std::shared_ptr<diagnostic_updater_ext::BoolDiagnostic> task)
  {
    task->set(msg->data);
    updater->update();
  }

public:
  BoolMonitor(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
    ROS_DEBUG("Starting BoolMonitor...");

    XmlRpc::XmlRpcValue topics;
    pnh_.getParam("topics", topics);
    ROS_ASSERT(topics.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(topics.size() > 0);
    for (int i = 0; i < topics.size(); ++i) {
      ROS_DEBUG("Reading %dth topic...", i);
      auto param = std::make_shared<BoolStatusParam>();
      if (parseTopicStatus(topics[i], *param)) {
        auto updater = std::make_shared<diagnostic_updater::Updater>();
        updater->setHardwareID(param->hardware_id);

        auto watcher = std::make_shared<diagnostic_updater_ext::BoolDiagnostic>(
          param->topic, *updater, param->bparam);
        auto sub = nh_.subscribe<std_msgs::Bool>(
          param->topic, 1,
          boost::bind(
            &diagnostic_generic_diagnostics::BoolMonitor::callback, this, _1, updater, watcher));

        ROS_DEBUG("Setup sub for %s", param->topic.c_str());
        subs_.push_back(sub);
      }
    }
  }
};

};  // namespace diagnostic_generic_diagnostics
