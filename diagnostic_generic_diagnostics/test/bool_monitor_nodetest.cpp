#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace
{
};  // namespace

class BoolMonitorTest : public ::testing::Test
{
public:
  BoolMonitorTest() : nh(""), pnh("")
  {
    pub_hoge = nh.advertise<std_msgs::Bool>("/test/hoge", 1);
    pub_fuga = nh.advertise<std_msgs::Bool>("/test/fuga", 1);

    sub = nh.subscribe("/diagnostics", 10, &BoolMonitorTest::callback, this);
  }

  void publish(const bool & value, ros::Publisher & pub)
  {
    const int freq = 10;
    std_msgs::Bool msg;
    msg.data = value;
    for (int i = 0; i < 2 * freq; ++i) {
      pub.publish(msg);
      ros::Duration(1.0 / static_cast<double>(freq)).sleep();  // Error range is above 30Hz.
    }
  }

  bool existKeycode(const diagnostic_msgs::DiagnosticStatus & stat)
  {
    for (const auto & value : stat.values) {
      if (value.key == "KEYCODE") {
        return true;
      }
    }
  }

  bool matchKeycode(const diagnostic_msgs::DiagnosticStatus & stat, const std::string & keycode_ans)
  {
    for (const auto & value : stat.values) {
      if (value.key == "KEYCODE" && value.value == keycode_ans) {
        return true;
      }
    }
  }

private:
  void callback(const diagnostic_msgs::DiagnosticArrayConstPtr & msgs)
  {
    // std::cerr << "callback invoked: " << msgs->status.size() << std::endl;

    for (const auto & msg : msgs->status) {
      // std::cerr << msg.hardware_id << std::endl;
      if (msg.hardware_id == "hoge-hw") {
        stat_hoge.hardware_id = msg.hardware_id;
        stat_hoge.level = msg.level;
        stat_hoge.message = msg.message;
        stat_hoge.name = msg.name;
        stat_hoge.values = msg.values;
      } else if (msg.hardware_id == "fuga-hw") {
        stat_fuga.hardware_id = msg.hardware_id;
        stat_fuga.level = msg.level;
        stat_fuga.message = msg.message;
        stat_fuga.name = msg.name;
        stat_fuga.values = msg.values;
      } else {
      }
    }
  }

protected:
  virtual void SetUp()
  {
    stat_hoge.clear();
    stat_fuga.clear();
  }

  ros::NodeHandle nh, pnh;
  ros::Publisher pub_hoge, pub_fuga;
  ros::Subscriber sub;
  diagnostic_updater::DiagnosticStatusWrapper stat_hoge, stat_fuga;
};

TEST_F(BoolMonitorTest, subPubInitTest)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>(
    "/diagnostics", nh, ros::Duration(10));

  EXPECT_STREQ("/test/hoge", pub_hoge.getTopic().c_str());
  EXPECT_EQ(1, pub_hoge.getNumSubscribers());

  EXPECT_STREQ("/test/fuga", pub_fuga.getTopic().c_str());
  EXPECT_EQ(1, pub_fuga.getNumSubscribers());

  EXPECT_STREQ("/diagnostics", sub.getTopic().c_str());
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST_F(BoolMonitorTest, normalError)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>(
    "/diagnostics", nh, ros::Duration(10));

  publish(false, pub_hoge);

  EXPECT_EQ(stat_hoge.ERROR, stat_hoge.level);

  EXPECT_TRUE(matchKeycode(stat_hoge, "HOGE001"));
}

TEST_F(BoolMonitorTest, normalOk)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>(
    "/diagnostics", nh, ros::Duration(10));

  publish(true, pub_hoge);

  EXPECT_EQ(stat_hoge.OK, stat_hoge.level);

  EXPECT_FALSE(existKeycode(stat_hoge));
}

TEST_F(BoolMonitorTest, invertedWarn)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>(
    "/diagnostics", nh, ros::Duration(10));

  publish(true, pub_fuga);

  EXPECT_EQ(stat_fuga.WARN, stat_fuga.level);

  EXPECT_FALSE(existKeycode(stat_fuga));
}

TEST_F(BoolMonitorTest, invertedOk)
{
  ros::topic::waitForMessage<diagnostic_msgs::DiagnosticArray>(
    "/diagnostics", nh, ros::Duration(10));

  publish(false, pub_fuga);

  EXPECT_EQ(stat_fuga.OK, stat_fuga.level);

  EXPECT_FALSE(existKeycode(stat_fuga));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "BoolMonitorTest");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
