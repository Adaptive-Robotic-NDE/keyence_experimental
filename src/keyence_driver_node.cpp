//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_msgs/msg/header.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <pcl/point_types.h>
// #include <pcl_ros/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <utility>
#include <vector>

#include <keyence/impl/keyence_exception.h>
#include <keyence/impl/keyence_tcp_client.h>
#include <keyence/impl/messages/high_speed_single_profile.h>
// #include <keyence/impl/messages/change_program.h>
#include <keyence/impl/messages/get_setting.h>
// When each profile was MEASURED, as opposed to when this process got round
// to publishing it. Read that header before changing anything below that
// touches a stamp.
#include <keyence/impl/profile_timing.h>
#include <keyence/impl/settings_defs.h>

#include <boost/bind/bind.hpp>

#include "boost/ref.hpp"
#include "boost/make_shared.hpp"

// #include "keyence_interfaces/srv/change_program.hpp"

// keyence protocol / profile related defines
static const std::string KEYENCE_DEFAULT_TCP_PORT = "24691";
static const std::string KEYENCE_DEFAULT_TCP_PORT_HS = "24692";

// values LJ Navigator uses for out-of-range points (in meters)
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI = -999.9990 / 1e3;
const static double KEYENCE_INFINITE_DISTANCE_VALUE_SI2 = -999.9970 / 1e3;

// default values for parameters
const static std::string DEFAULT_FRAME_ID = "lj_v7080_optical_frame";

// local types
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

// Prototype for function that converts a given profile to
// a PCL point cloud.
//
// `stamp_us` is the profile's ACQUISITION time in microseconds, which is what
// a pcl::PCLHeader stamp is measured in. It is passed in rather than read off
// a clock here: this function converts data, and a conversion routine reading
// the wall clock is exactly how the stamp came to mean "when the driver got
// here" instead of "when the sensor measured". See keyence/impl/profile_timing.h.
int unpackProfileToPointCloud(const keyence::ProfileInformation& info,
                              const std::vector<int>& points,
                              pcl::PointCloud<pcl::PointXYZ>& msg,
                              bool cnv_inf_pts,
                              uint64_t stamp_us);

namespace
{

using Clock = std::chrono::steady_clock;
using Diagnostics = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

/// The state words published on the status topic. A consumer branches on
/// these; the human-readable `message` beside them is free to change.
///
/// The distinction that matters, and the reason this topic exists: `no_data`
/// and `disconnected` look identical from outside -- /profiles is silent --
/// and mean opposite things. With the laser switched off, /profiles goes quiet
/// while the driver stays connected and answered; without this topic every
/// consumer reads that as a dead driver.
const char* const STATE_CONNECTING = "connecting";
const char* const STATE_STREAMING = "streaming";
const char* const STATE_NO_DATA = "connected_no_data";
const char* const STATE_CONTROLLER_ERROR = "controller_error";
const char* const STATE_UNSUBSCRIBED = "connected_unsubscribed";
const char* const STATE_NOT_TRIGGERING = "connected_not_triggering";
const char* const STATE_DISCONNECTED = "disconnected";

double seconds_since(Clock::time_point then, Clock::time_point now)
{
  return std::chrono::duration<double>(now - then).count();
}

std::string to_string(double value, int precision = 2)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(precision);
  os << value;
  return os.str();
}

/**
 * @brief "Say this at most once every N seconds", on the steady clock.
 *
 * NOT RCLCPP_*_THROTTLE: that throttles on a node clock, which stalls
 * completely when use_sim_time is on and nothing publishes /clock -- and a
 * throttle that never fires is indistinguishable from the bug this is here to
 * fix. The first call after a reset() always fires, so entering a state logs
 * immediately and then settles into the period.
 */
class Throttle
{
public:
  explicit Throttle(double period_seconds) : period_(period_seconds) {}

  bool due(Clock::time_point now)
  {
    if (!armed_ || seconds_since(last_, now) >= period_)
    {
      last_ = now;
      armed_ = true;
      return true;
    }
    return false;
  }

  void reset() { armed_ = false; }

private:
  double period_;
  bool armed_ = false;
  Clock::time_point last_{};
};

/// Everything the status topic reports. Plain data so that building a message
/// never needs the acquisition loop's locals.
struct DriverState
{
  std::string state = STATE_CONNECTING;
  std::string message;
  std::string controller;
  std::string frame_id;
  int active_program = -1;
  double sampling_rate_hz = 0.0;
  bool continuous_trigger = false;
  double achieved_rate_hz = 0.0;
  uint64_t profiles_published = 0;
  uint64_t no_data_replies = 0;
  uint64_t consecutive_no_data = 0;
  uint64_t controller_errors = 0;
  int last_return_code = 0;
  uint64_t reconnects = 0;
  size_t subscribers = 0;
  double seconds_since_profile = -1.0;
  /// How far before the reply the last profile was stamped, in milliseconds.
  /// Publishing it makes the timing model visible to anyone auditing a scan
  /// rather than leaving it to be inferred from the source.
  double acquisition_backdate_ms = 0.0;
  /// Replies carrying a profile the controller had already given us, told
  /// apart by an unchanged trigger_count. See the acquisition loop.
  uint64_t repeated_profiles = 0;
};

/**
 * @brief Declare a TCP port parameter that accepts `"24691"` or `24691`.
 *
 * The port is a string because libsocket takes a service name. But every
 * natural way to pass one -- `-p controller_port:=24691` on the command line,
 * `{"controller_port": 24691}` in a launch file -- produces an INTEGER, and
 * declaring a string parameter against an integer override is a fatal
 * InvalidParameterTypeException: the node aborts before it has logged a single
 * line, which is a worse failure than the silently-ignored override that
 * declaring it properly was meant to fix. So take either spelling.
 */
std::string declarePort(const rclcpp::Node::SharedPtr& node, const std::string& name,
                        const std::string& fallback)
{
  try
  {
    node->declare_parameter<std::string>(name, fallback);
    return node->get_parameter(name).as_string();
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException&)
  {
    // declare_parameter validates before it stores, so `name` is still
    // undeclared here and can be declared again with the type it was given.
  }
  try
  {
    node->declare_parameter<int64_t>(name, std::stoll(fallback));
    return std::to_string(node->get_parameter(name).as_int());
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(node->get_logger(),
                "Parameter '%s' is neither a string nor an integer (%s); using %s.",
                name.c_str(), ex.what(), fallback.c_str());
    return fallback;
  }
}

unsigned char level_of(const std::string& state)
{
  if (state == STATE_STREAMING || state == STATE_UNSUBSCRIBED)
  {
    return DiagnosticStatus::OK;
  }
  if (state == STATE_DISCONNECTED)
  {
    return DiagnosticStatus::ERROR;
  }
  return DiagnosticStatus::WARN;   // no data, controller error, connecting
}

/**
 * @brief Publishes what the driver is doing, so that "silent" can be told
 *        from "dead" without reading a launch log.
 *
 * diagnostic_msgs/DiagnosticArray on `keyence_status` (remappable). The QoS is
 * transient-local depth 1, so a tool that starts later is told the current
 * state at once rather than waiting for the next heartbeat; it is republished
 * on every state change and at least once a second regardless, because the
 * bring-up's topic audit subscribes with volatile durability and would
 * otherwise see nothing.
 */
class StatusPublisher
{
public:
  StatusPublisher(rclcpp::Node::SharedPtr node, double heartbeat_seconds)
    : node_(std::move(node)), heartbeat_(heartbeat_seconds)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable().transient_local();
    pub_ = node_->create_publisher<Diagnostics>("keyence_status", qos);
  }

  /// Publish if the state word or message changed, or the heartbeat is due.
  void report(const DriverState& state, Clock::time_point now)
  {
    const bool changed = state.state != last_state_ || state.message != last_message_;
    if (!changed && armed_ && seconds_since(last_published_, now) < heartbeat_)
    {
      return;
    }
    last_state_ = state.state;
    last_message_ = state.message;
    last_published_ = now;
    armed_ = true;

    DiagnosticStatus status;
    status.level = level_of(state.state);
    status.name = "keyence_lj_driver: profiles";
    status.hardware_id = state.controller;
    status.message = state.message;
    add(status, "state", state.state);
    add(status, "controller", state.controller);
    add(status, "frame_id", state.frame_id);
    add(status, "active_program", std::to_string(state.active_program));
    add(status, "program_sampling_rate_hz", to_string(state.sampling_rate_hz, 1));
    add(status, "continuous_trigger", state.continuous_trigger ? "true" : "false");
    add(status, "achieved_rate_hz", to_string(state.achieved_rate_hz, 1));
    add(status, "profiles_published", std::to_string(state.profiles_published));
    add(status, "no_data_replies", std::to_string(state.no_data_replies));
    add(status, "consecutive_no_data", std::to_string(state.consecutive_no_data));
    add(status, "controller_errors", std::to_string(state.controller_errors));
    add(status, "last_return_code", std::to_string(state.last_return_code));
    add(status, "reconnects", std::to_string(state.reconnects));
    add(status, "subscribers", std::to_string(state.subscribers));
    add(status, "seconds_since_profile",
        state.seconds_since_profile < 0.0 ? "never" : to_string(state.seconds_since_profile, 1));
    add(status, "acquisition_backdate_ms", to_string(state.acquisition_backdate_ms, 2));
    add(status, "repeated_profiles", std::to_string(state.repeated_profiles));

    Diagnostics msg;
    msg.header.stamp = node_->now();
    msg.status.push_back(status);
    pub_->publish(msg);
  }

private:
  static void add(DiagnosticStatus& status, const std::string& key, const std::string& value)
  {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<Diagnostics>::SharedPtr pub_;
  double heartbeat_;
  std::string last_state_;
  std::string last_message_;
  Clock::time_point last_published_{};
  bool armed_ = false;
};

}  // namespace

/**
 * @brief Given a @e client, makes a request to figure out if the given @e program
 * is continuously triggered or not.
 */
bool isProgramContinuouslyTriggered(keyence::TcpClient& client, uint8_t program)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(program);
  uint8_t category = keyence::setting::program::TriggerMode::category;
  uint8_t item = keyence::setting::program::TriggerMode::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  // has_data(), not good(): a setting read can come back well formed with an
  // empty payload, and every line below reads data[0]. See ResponseStatus.
  if (!resp.has_data())
  {
    throw keyence::KeyenceException(
        std::string("Reading the trigger mode of program ") + std::to_string(program) +
        " gave no answer (" + keyence::describe(resp.status) + ", return code " +
        std::to_string(static_cast<int>(resp.header.return_code)) + ")");
  }

  uint8_t result_id = resp.body.data[0];

  if (result_id == keyence::setting::program::TriggerMode::continuous_trigger)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Get's the active program number by making a settings-read request and pulling
 * the number out of the response header.
 */
uint8_t getActiveProgramNumber(keyence::TcpClient& client)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(0);
  uint8_t category = keyence::setting::program::TriggerMode::category;
  uint8_t item = keyence::setting::program::TriggerMode::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  // Deliberately NOT checked for a payload: this reads the program number out
  // of the response HEADER, which is present on every reply including an error
  // one, and it is the first request made after connecting.
  return resp.header.active_program_no;
}

/**
 * @brief Get's the commanded sampling frequency of the auto-trigger setting
 * for the given program. Ranges from 10 Hz - 64 kHz.
 *
 * THIS IS THE CONTROLLER'S RATE, NOT THE DRIVER'S. One profile per TCP round
 * trip cannot reach it: against a program set to 1000 Hz on a real LJ-V7080
 * the measured range is 136 to 218.9 Hz. The driver reports the rate it
 * actually achieves instead of this one -- see the acquisition loop's periodic
 * report.
 */
double getProgramSamplingRate(keyence::TcpClient& client, uint8_t program)
{
  uint8_t level = keyence::setting::write_area;
  uint8_t type = keyence::setting::program::programType(program);
  uint8_t category = keyence::setting::program::SamplingPeriod::category;
  uint8_t item = keyence::setting::program::SamplingPeriod::item;

  keyence::command::GetSetting::Request req (level, type, category, item, 0, 0, 0, 0);
  auto resp = client.sendReceive(req);

  if (!resp.has_data())
  {
    throw keyence::KeyenceException(
        std::string("Reading the sampling rate of program ") + std::to_string(program) +
        " gave no answer (" + keyence::describe(resp.status) + ", return code " +
        std::to_string(static_cast<int>(resp.header.return_code)) + ")");
  }

  uint8_t result_id = resp.body.data[0];

  switch (result_id)
  {
  case keyence::setting::program::SamplingPeriod::freq_10hz:
    return 10.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_20hz:
    return 20.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_50hz:
    return 50.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_100hz:
    return 100.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_200hz:
    return 200.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_500hz:
    return 500.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_1000hz:
    return 1000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_2000hz:
    return 2000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_4000hz:
    return 4000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_4130hz:
    return 4130.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_8000hz:
    return 8000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_16000hz:
    return 16000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_32000hz:
    return 32000.0;
    break;
  case keyence::setting::program::SamplingPeriod::freq_64000hz:
    return 64000.0;
    break;
  default:
    throw keyence::KeyenceException("Received unrecognized frequency code: " + std::to_string(result_id));
  }
}

/**
 * @brief Services external ROS requests to change the active program. Will reset
 * activity flag and sampling rate according to the settings of the new program.
 */
// bool changeProgramCallback(keyence_interfaces::srv::ChangeProgram::Request& req,
//                            keyence_interfaces::srv::ChangeProgram::Response& res,
//                            keyence::TcpClient& client, bool& active_flag,
//                            rclcpp::Rate& rate)
// {
//   // ... (implementation removed) ...
// }


int main(int argc, char** argv)
{
  //ros::init(argc, argv, "keyence_lj_driver");
  //ros::NodeHandle nh, pnh("~");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("keyence_lj_driver");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // The node's own logger, not rclcpp::get_logger("rclcpp"). Every line this
  // driver has ever written went out under the name "rclcpp", which in a
  // bring-up log of thirty nodes says nothing about who wrote it.
  auto logger = node->get_logger();

  // ros parameters
  std::string sensor_host;
  std::string frame_id;
  std::string sensor_port;
  double sample_rate;

  // DECLARE THEM. controller_port and frame_id used to be read with
  // get_parameter_or() on names that were never declared, which in ROS 2 means
  // the override a launch file passes is not just ignored, it cannot even be
  // seen (`ros2 param list` does not show them). Anything that wanted a second
  // head on another port, or the frame the robot's URDF actually uses, was
  // silently getting 24691 and lj_v7080_optical_frame.
  node->declare_parameter<std::string>("controller_ip", "192.168.1.153");
  node->declare_parameter<std::string>("frame_id", DEFAULT_FRAME_ID);
  sensor_port = declarePort(node, "controller_port", KEYENCE_DEFAULT_TCP_PORT);

  // How fast to poll while profiles are arriving. 0 means "as fast as the
  // controller's program samples", which is what this has always asked for and
  // is a ceiling the TCP round trip sets long before the sleep does.
  node->declare_parameter<double>("poll_rate", 0.0);
  // ... and while there are none. A controller with its laser off answers
  // every request, so polling it at full speed is a TCP round trip per poll
  // for as long as the laser stays off, which can be a whole shift. Back off
  // instead. 20 Hz costs at most 50 ms of latency when the laser comes back
  // on, long before an arm has moved anywhere.
  node->declare_parameter<double>("idle_poll_rate", 20.0);
  node->declare_parameter<double>("idle_after", 2.0);
  // One line every this many seconds while there is nothing to read.
  node->declare_parameter<double>("no_data_log_period", 5.0);
  // How often to report the rate actually achieved.
  node->declare_parameter<double>("rate_report_period", 10.0);
  node->declare_parameter<double>("reconnect_pause", 1.0);
  node->declare_parameter<double>("status_period", 1.0);

  // --- Profile timestamps. See keyence/impl/profile_timing.h. ---
  // How long before the reply arrived the profile was actually measured.
  // Negative (the default) means "model it from the round trip and the
  // program's sampling period", which is all this driver can know without a
  // synchronised clock. A cell that has MEASURED the latency should pin it
  // here instead, in seconds.
  node->declare_parameter<double>("acquisition_latency", -1.0);
  // Hard limit on that backdating, in seconds. A round trip that stalled must
  // not be allowed to place a profile arbitrarily far in the past: half of a
  // one-second stall is half a second of arm travel in the wrong direction,
  // and TF would happily interpolate to it.
  node->declare_parameter<double>("max_stamp_backdate", 0.05);
  // A controller answers a profile request with its most recent profile. Poll
  // it faster than its program samples and the SAME measurement comes back
  // twice, which downstream places at two different arm poses -- the same
  // wrong-geometry failure this file's timestamps are about. trigger_count
  // increments once per controller sample, so an unchanged one identifies a
  // repeat exactly. Dropping is armed only after the counter has been SEEN to
  // advance, so a program that does not maintain it cannot silence the topic.
  node->declare_parameter<bool>("drop_repeated_profiles", true);

  // check required parameters
  if (!node->has_parameter("controller_ip"))
  {
    RCLCPP_FATAL(logger, "Parameter 'controller_ip' missing. Cannot continue.");
    return -1;
  }

  node->get_parameter("controller_ip", sensor_host);
  node->get_parameter("frame_id", frame_id);

  const double poll_rate_param = node->get_parameter("poll_rate").as_double();
  const double idle_poll_rate = node->get_parameter("idle_poll_rate").as_double();
  const double idle_after = node->get_parameter("idle_after").as_double();
  const double no_data_log_period = node->get_parameter("no_data_log_period").as_double();
  const double rate_report_period = node->get_parameter("rate_report_period").as_double();
  const double reconnect_pause = node->get_parameter("reconnect_pause").as_double();
  const double status_period = node->get_parameter("status_period").as_double();
  const double acquisition_latency = node->get_parameter("acquisition_latency").as_double();
  const double max_stamp_backdate = node->get_parameter("max_stamp_backdate").as_double();
  const bool drop_repeated_profiles = node->get_parameter("drop_repeated_profiles").as_bool();
  const int64_t max_backdate_ns = static_cast<int64_t>(max_stamp_backdate * 1e9);
  const int64_t fixed_latency_ns =
      acquisition_latency >= 0.0 ? static_cast<int64_t>(acquisition_latency * 1e9) : -1;

  RCLCPP_INFO(logger, "Attempting to connect to %s (TCP %s); expecting a single head attached to port A.",
           sensor_host.c_str(), sensor_port.c_str());

  // setup point cloud message (we reuse single one)
  // TODO: this won't work with nodelets
  Cloud::Ptr pc_msg(new Cloud);
  pc_msg->header.frame_id = frame_id;
  pc_msg->is_dense = false; // cloud could have NaNs
  // message is essentially a line-strip of points
  pc_msg->height = 1;

  // set up profile cloud publisher
  //ros::Publisher pub = nh.advertise<Cloud>("profiles", 100);
  //auto pub = node->create_publisher<Cloud>("profiles", 100);
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("profiles", 100);
  //auto pub = node->create_publisher<pcl::PointCloud<pcl::PointXYZ>>("profiles", 100);

  StatusPublisher status(node, status_period);

  DriverState state;
  state.controller = sensor_host + ":" + sensor_port;
  state.frame_id = frame_id;

  // Outside the reconnect loop: "when did a profile last arrive" is a fact
  // about the sensor, and it does not reset because a socket was rebuilt.
  Clock::time_point last_profile{};

  bool active_flag = true;

  while (rclcpp::ok())
  {
    try
    {
      state.state = STATE_CONNECTING;
      state.message = "connecting to " + state.controller;
      status.report(state, Clock::now());

      // Establish communications
      keyence::TcpClient keyence(sensor_host, sensor_port);

      auto active_program = getActiveProgramNumber(keyence);
      active_flag = isProgramContinuouslyTriggered(keyence, active_program);
      sample_rate = getProgramSamplingRate(keyence, active_program);

      state.active_program = static_cast<int>(active_program);
      state.sampling_rate_hz = sample_rate;
      state.continuous_trigger = active_flag;

      RCLCPP_INFO(logger, "Connection established to Keyence controller at %s.",
                  state.controller.c_str());
      RCLCPP_INFO(logger, "Beginning with program %hhu with a sampling rate of %.0f Hz",
               active_program, sample_rate);
      if (!active_flag)
      {
        RCLCPP_INFO(logger, "Note that program %hhu is not configured for continuous sampling",
                 active_program);
      }

      // Announce the poll cap, not the controller's program setting. One
      // profile per TCP round trip does not come within a factor of four of
      // that setting (136 to 218.9 Hz against a program set to 1000 Hz), and
      // announcing a number the driver cannot reach makes every rate check
      // downstream look broken. The achieved rate is reported as it is
      // measured.
      const double poll_rate = poll_rate_param > 0.0 ? poll_rate_param : sample_rate;
      RCLCPP_INFO(logger,
                  "Polling one profile per TCP round trip, capped at %.0f Hz. That cap is "
                  "not the rate to expect: the round trip sets it, and this driver measured "
                  "137-219 Hz at the cell. The achieved rate follows every %.0f s.",
                  poll_rate, rate_report_period);

      const std::chrono::duration<double> poll_period(poll_rate > 0.0 ? 1.0 / poll_rate : 0.0);
      const std::chrono::duration<double> idle_period(
          idle_poll_rate > 0.0 ? 1.0 / idle_poll_rate : 0.0);

      // Half of this is the "the profile finished some time in the last
      // sampling period" half of the backdate. It is the CONTROLLER's rate,
      // read from the program, not the rate this driver achieves.
      const int64_t sampling_period_ns =
          sample_rate > 0.0 ? static_cast<int64_t>(1e9 / sample_rate) : 0;
      if (fixed_latency_ns >= 0)
      {
        RCLCPP_INFO(logger,
                    "Profiles are stamped %.2f ms before the reply arrives (acquisition_latency "
                    "was set explicitly), clamped to %.0f ms.",
                    fixed_latency_ns / 1e6, max_stamp_backdate * 1e3);
      }
      else
      {
        RCLCPP_INFO(logger,
                    "Profiles are stamped at their estimated ACQUISITION time: half the "
                    "measured round trip plus half the %.0f Hz sampling period before the "
                    "reply, clamped to %.0f ms. Set acquisition_latency to pin it to a "
                    "measured value instead.",
                    sample_rate, max_stamp_backdate * 1e3);
      }

      // trigger_count is the controller's own sample counter. Two consecutive
      // replies carrying the same one are the same measurement handed out
      // twice -- see the drop_repeated_profiles parameter.
      uint32_t last_trigger_count = 0;
      bool have_trigger_count = false;
      bool trigger_count_advances = false;
      Throttle repeated_profile_log(no_data_log_period);

      // rclcpp::ServiceBase program_server =
      //     node.advertiseService<keyence::command::ChangeProgram::Request,
      //                         keyence::command::ChangeProgram::Response>(
      //         "change_program", boost::bind(changeProgramCallback, boost::placeholders::_1, boost::placeholders::_2, boost::ref(keyence),
      //                                       boost::ref(active_flag), boost::ref(sleeper)));

      Throttle no_data_log(no_data_log_period);
      Throttle controller_error_log(no_data_log_period);
      Throttle not_triggering_log(no_data_log_period);

      auto last_report = Clock::now();
      uint64_t published_at_report = state.profiles_published;
      uint64_t no_data_at_report = state.no_data_replies;
      Clock::time_point no_data_since{};
      bool idle = false;

      // Main loop
      while (rclcpp::ok())
      {
        const auto started = Clock::now();
        executor.spin_some();

        state.subscribers = pub->get_subscription_count();

        // avoid interacting with sensor if there are no publishers
        // TODO: maybe we should actually always poll sensor, but just not
        //       publish anything (although unpacking + publishing is cheap)
        if (state.subscribers == 0)
        {
          state.state = STATE_UNSUBSCRIBED;
          state.message = "connected and idle: nothing subscribes to profiles, so the "
                          "controller is not being polled";
          state.consecutive_no_data = 0;
          no_data_since = Clock::time_point{};
          no_data_log.reset();
          idle = true;
        }
        else if (!active_flag)
        {
          // Once every no_data_log_period, not once per poll: this used to be
          // an unthrottled INFO inside a loop asking for 1000 Hz.
          state.state = STATE_NOT_TRIGGERING;
          state.message = "connected, but program " + std::to_string(state.active_program) +
                          " is not configured for continuous sampling, so nothing will "
                          "be acquired";
          if (not_triggering_log.due(started))
          {
            RCLCPP_WARN(logger, "%s.", state.message.c_str());
          }
          idle = true;
        }
        else
        {
          // unpack profile data
          keyence::command::SingleProfile::Request req;
          // BRACKET THE ROUND TRIP. These two clock reads are the only
          // evidence this process has about when the measurement happened;
          // everything after sendReceive() returns is this driver's own
          // latency and must not end up in the stamp.
          const rclcpp::Time t_request = node->now();
          auto resp = keyence.sendReceive(req);
          const rclcpp::Time t_reply = node->now();

          if (resp.status == keyence::ResponseStatus::no_data)
          {
            // A controller with nothing to give answers normally and
            // attaches no profile. Decoding it instead throws out of the
            // decoder, unwinds past the socket declared inside this try block,
            // and rebuilds the whole connection once a second forever. Treat
            // it as a fact about the world: report it and wait it out. The
            // socket is not touched and nothing is published.
            state.no_data_replies++;
            state.consecutive_no_data++;
            if (no_data_since == Clock::time_point{})
            {
              no_data_since = started;
            }
            state.state = STATE_NO_DATA;
            state.message = "connected, and the controller has no profile to give: the "
                            "laser is off (a UR digital output switches it -- output 1 on "
                            "ex-ur10e-1) or nothing is within range of the head";
            if (no_data_log.due(started))
            {
              RCLCPP_WARN(logger,
                          "The Keyence controller is answering and has NO PROFILE to give "
                          "(%lu replies, %.0f s so far). The laser is off, or nothing is "
                          "within range of the head. This is not a fault and the connection "
                          "is being kept: profiles resume by themselves when the laser comes "
                          "back on. Reply was return code %d, %lu bytes.",
                          static_cast<unsigned long>(state.consecutive_no_data),
                          seconds_since(no_data_since, started),
                          static_cast<int>(resp.header.return_code),
                          static_cast<unsigned long>(resp.received));
              RCLCPP_DEBUG(logger, "No-profile reply: %s", resp.dump.c_str());
            }
            idle = seconds_since(no_data_since, started) >= idle_after;
          }
          else if (resp.status == keyence::ResponseStatus::controller_error)
          {
            // A non-zero return code: the controller understood and refused.
            // Nothing a new socket would fix, so do not reconnect.
            state.controller_errors++;
            state.last_return_code = static_cast<int>(resp.header.return_code);
            state.state = STATE_CONTROLLER_ERROR;
            state.message = "connected, and the controller is refusing profile requests "
                            "with return code " + std::to_string(state.last_return_code);
            if (controller_error_log.due(started))
            {
              RCLCPP_ERROR(logger,
                           "The Keyence controller refused the profile request: return code "
                           "%d, controller status %d, program %d (%lu such replies). The "
                           "connection is healthy, so this is the program or the request, "
                           "not the network.",
                           state.last_return_code,
                           static_cast<int>(resp.header.controller_status),
                           static_cast<int>(resp.header.active_program_no),
                           static_cast<unsigned long>(state.controller_errors));
              RCLCPP_DEBUG(logger, "Refused-request reply: %s", resp.dump.c_str());
            }
            idle = true;
          }
          else
          {
            // SAME MEASUREMENT AGAIN? The controller hands back its most
            // recent profile, so polling faster than it samples repeats one.
            // Republishing it gives one measurement two stamps and two arm
            // poses, which is precisely the failure the stamp below exists to
            // avoid, so drop it -- but only once trigger_count has been seen
            // to move at least once, so that a program which leaves the field
            // at a constant cannot silence the topic.
            const uint32_t trigger_count = resp.body.trigger_count;
            const bool repeat = have_trigger_count && trigger_count == last_trigger_count;
            if (have_trigger_count && trigger_count != last_trigger_count)
            {
              trigger_count_advances = true;
            }
            last_trigger_count = trigger_count;
            have_trigger_count = true;

            if (repeat)
            {
              state.repeated_profiles++;
              if (repeated_profile_log.due(started))
              {
                RCLCPP_WARN(logger,
                            "The controller returned the profile it had already given us "
                            "(trigger_count %u unchanged, %lu times so far). This driver is "
                            "polling faster than program %d samples (%.0f Hz); each repeat "
                            "would be placed at a second arm pose. %s",
                            trigger_count,
                            static_cast<unsigned long>(state.repeated_profiles),
                            state.active_program, sample_rate,
                            (drop_repeated_profiles && trigger_count_advances)
                                ? "They are being dropped."
                                : "They are being PUBLISHED: set drop_repeated_profiles, or "
                                  "raise the program's sampling rate.");
              }
            }

            const bool drop = repeat && drop_repeated_profiles && trigger_count_advances;
            if (!drop)
            {
              // WHEN THIS PROFILE WAS MEASURED, not when it was published.
              const int64_t stamp_ns =
                  fixed_latency_ns >= 0
                      ? keyence::profileAcquisitionTimeNsFixed(t_reply.nanoseconds(),
                                                               fixed_latency_ns, max_backdate_ns)
                      : keyence::profileAcquisitionTimeNs(t_request.nanoseconds(),
                                                          t_reply.nanoseconds(),
                                                          sampling_period_ns, max_backdate_ns);
              state.acquisition_backdate_ms = (t_reply.nanoseconds() - stamp_ns) / 1e6;

              // convert to pointcloud
              pc_msg->points.clear();
              unpackProfileToPointCloud(resp.body.profile_info, resp.body.profile_points, *pc_msg,
                                        true, static_cast<uint64_t>(stamp_ns / 1000));

              sensor_msgs::msg::PointCloud2 ros_cloud;
              pcl::toROSMsg(*pc_msg, ros_cloud);
              // pcl::toROSMsg() derives the ROS stamp from the PCL one, which
              // is microseconds; write the nanosecond value straight in so
              // nothing downstream inherits a rounding step it cannot see.
              // This is the stamp TF is asked about, so it is the one that has
              // to be right.
              ros_cloud.header.stamp = rclcpp::Time(stamp_ns, node->get_clock()->get_clock_type());
              // publish pointcloud
              // pub->publish(*pc_msg);
              pub->publish(ros_cloud);
              state.profiles_published++;
            }

            if (state.consecutive_no_data > 0)
            {
              RCLCPP_INFO(logger,
                          "Profiles have resumed after %lu empty replies over %.0f s -- the "
                          "connection was never lost.",
                          static_cast<unsigned long>(state.consecutive_no_data),
                          seconds_since(no_data_since, started));
            }
            state.consecutive_no_data = 0;
            state.state = STATE_STREAMING;
            state.message = "publishing profiles";
            state.seconds_since_profile = 0.0;
            last_profile = started;
            no_data_since = Clock::time_point{};
            no_data_log.reset();
            controller_error_log.reset();
            idle = false;
          }
        }

        if (last_profile != Clock::time_point{})
        {
          state.seconds_since_profile = seconds_since(last_profile, started);
        }

        // The achieved rate, periodically. What the old "Attempting to publish
        // at %.2f Hz" claimed once and never revisited.
        if (seconds_since(last_report, started) >= rate_report_period)
        {
          const double window = seconds_since(last_report, started);
          const uint64_t published = state.profiles_published - published_at_report;
          const uint64_t empty = state.no_data_replies - no_data_at_report;
          state.achieved_rate_hz = published / window;
          if (published > 0)
          {
            const std::string empties =
                empty ? ", " + std::to_string(empty) + " empty replies" : std::string();
            RCLCPP_INFO(logger, "Publishing %.1f profiles/s (%lu in %.0f s%s).",
                        state.achieved_rate_hz, static_cast<unsigned long>(published), window,
                        empties.c_str());
          }
          last_report = started;
          published_at_report = state.profiles_published;
          no_data_at_report = state.no_data_replies;
        }

        status.report(state, started);

        // Sleep to the poll period -- the idle one when there is nothing to
        // read, so that a laser left off does not cost a TCP round trip every
        // few milliseconds for a whole shift.
        const auto target = idle ? idle_period : poll_period;
        const auto elapsed = Clock::now() - started;
        if (target > elapsed)
        {
          std::this_thread::sleep_for(target - elapsed);
        }
      } // end main loop
    }
    // LOCAL PATCH. Two things these handlers get right that the originals did
    // not:
    //  * they log through rclcpp. PCL_ERROR_STREAM goes to PCL's own console
    //    and never reaches ~/.ros/log, so a driver that retries leaves no
    //    record of having retried -- only of having died.
    //  * they catch std::exception as well as KeyenceException. Anything else
    //    -- std::bad_alloc from a bogus length, a libsocket exception thrown
    //    outside TcpClient's own try/catch, std::system_error from the
    //    executor -- escapes main() and becomes SIGABRT. A driver whose whole
    //    job is to survive a sensor that is not answering yet has to retry on
    //    any failure.
    //
    // What reaches here is a SOCKET failure or a genuinely malformed message,
    // and nothing else. A controller that is answering and has nothing to say
    // is handled in the loop above without dropping the connection.
    catch (const keyence::KeyenceException& ex)
    {
      state.reconnects++;
      state.state = STATE_DISCONNECTED;
      state.message = std::string("disconnected: ") + ex.what();
      status.report(state, Clock::now());
      RCLCPP_ERROR(logger,
                   "Keyence exception caught: %s. Reconnecting after a %.0f s pause.",
                   ex.what(), reconnect_pause);
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_pause));
    }
    catch (const std::exception& ex)
    {
      state.reconnects++;
      state.state = STATE_DISCONNECTED;
      state.message = std::string("disconnected: ") + ex.what();
      status.report(state, Clock::now());
      RCLCPP_ERROR(logger,
                   "Unexpected %s: %s. Reconnecting after a %.0f s pause.",
                   typeid(ex).name(), ex.what(), reconnect_pause);
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_pause));
    }
  }

  rclcpp::shutdown();
  return 0;
}

int unpackProfileToPointCloud(const keyence::ProfileInformation& info,
                              const std::vector<int>& points,
                              pcl::PointCloud<pcl::PointXYZ>& msg,
                              bool cnv_inf_pts,
                              uint64_t stamp_us)

{
  // The acquisition instant, computed by the caller from the round trip that
  // fetched this profile. This used to be node->now() with a "TODO: get proper
  // timestamp from somewhere" beside it, i.e. the moment the driver reached
  // this line. pcl header stamps are in microseconds.
  msg.header.stamp = stamp_us;
  // NOT a batch count, despite the name: num_profiles is the number of points
  // ACROSS THE LASER LINE in this one profile (800 on an LJ-V7080), which is
  // why it is the cloud's width and why the loop below walks it as
  // x = x_start + i * x_increment. One profile per reply.
  msg.width = info.num_profiles;
  cnv_inf_pts = true;

  double x = 0., y = 0., z = 0.;

  msg.points.reserve(info.num_profiles);

  // add points
  for (int i = 0; i < static_cast<int>(points.size()); ++i)
  {
    // convert profile points to SI units (meters)
    x = keyence::unitsToMeters(info.x_start + i * info.x_increment);
    y = 0.0;
    z = keyence::unitsToMeters(static_cast<int>(info.data_unit) * points[i]);

    // filter out 'infinite distance' points
    // REP-117: http://www.ros.org/reps/rep-0117.html
    //  "out of range detections will be represented by +Inf."
    if (points[i] == KEYENCE_INVALID_DATA_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI;
    }

    // device returns two different values that are supposed to be interpreted
    // as out-of-range or 'infinite'. This is the second
    if (points[i] == KEYENCE_DEAD_ZONE_DATA_VALUE)
    {
      if (cnv_inf_pts)
        z = std::numeric_limits<double>::infinity();
      else
        z = KEYENCE_INFINITE_DISTANCE_VALUE_SI2;
    }

    msg.points.push_back(pcl::PointXYZ(x, y, z));
  }

  return 0;
}
