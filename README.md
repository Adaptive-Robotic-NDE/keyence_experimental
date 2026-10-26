[![Build Status](https://travis-ci.com/ros-industrial/keyence_experimental.svg?branch=kinetic-devel)](https://travis-ci.com/ros-industrial/keyence_experimental)

# keyence_experimental
A library for communicating with Keyence LJ-V series sensors over TCP/IP. 

It has been tested on the Keyence LJ-V7080 at Southwest Research Institute.

## Organization
This package is divided into two logical components:
 - A cmake-based library for communication with the Keyence (only dependencies are C++-11 and libsocket)
 - A ROS node that uses the library to publish point clouds

## Build
The keyence library requires:
 - [wstool](http://wiki.ros.org/wstool)

Download wstool and dependencies:
```
mkdir -p my_workspace/src
cd my_workspace/src
wstool init .
wstool merge https://github.com/ros-industrial/keyence_experimental/raw/kinetic-devel/keyence_experimental.rosinstall
wstool update
cd ..
catkin build
```

## Capability
Currently, laser program configuration must be done through the Windows-based Keyence utility. This API supports:
  - Changing active programs
  - Sampling individual profiles via the service interface

## Examples

See the [keyence_library/src](keyence_library/src) directory for C++ examples of the above capabilities.

## ROS Node
To run the ros node:
```
rosrun keyence_experimental keyence_driver_node _controller_ip:=xxx.xxx.xxx.xxx
```

Or set the `controller_ip` parameter as a private param in a launch file.

The following additional parameters are supported:
  - `controller_port`, defaults to 24691, the service-port of the Keyence controller. A
    string or an integer; both spellings are accepted.
  - `frame_id`, defaults to `lj_v7080_optical_frame`, the frame of reference included in the published point cloud
  - `poll_rate`, Hz, 0 (the default) meaning "the controller program's own sampling rate".
    This is a CEILING and not a rate to expect -- see "What rate to expect" below.
  - `idle_poll_rate`, Hz, default 20: how fast to poll once the controller has been giving
    nothing for `idle_after` seconds (default 2). A controller with its laser off answers
    every request, so polling at full speed costs a TCP round trip every few milliseconds
    for as long as the laser stays off.
  - `no_data_log_period`, seconds, default 5: at most one "nothing to give" line per period.
  - `rate_report_period`, seconds, default 10: how often the ACHIEVED rate is logged.
  - `reconnect_pause`, seconds, default 1, and `status_period`, seconds, default 1.

By default the node publishes XYZ point clouds on the `profiles` topic. Note that nothing will be published if
no one subscribes that topic.

The active program can be set via the `keyence_experimental::ChangeProgram` service call. See the [srv/](srv/) directory for more info.

## What the driver says about itself: `keyence_status`

`diagnostic_msgs/DiagnosticArray`, one status, latched (transient local, depth 1) and
republished once a second. It exists because a silent `profiles` topic has two causes that
look identical from outside and mean opposite things. The `state` key/value is the one to
branch on:

| state | level | means |
|---|---|---|
| `streaming` | OK | profiles are being published |
| `connected_unsubscribed` | OK | connected; nothing subscribes, so the controller is not polled |
| `connected_no_data` | WARN | connected, and the controller has nothing to measure: **the laser is off**, or nothing is in range |
| `connected_not_triggering` | WARN | connected; the active program is not set to sample continuously |
| `controller_error` | WARN | connected; the controller is refusing the request (see `last_return_code`) |
| `connecting` / `disconnected` | WARN / ERROR | the socket. `message` carries the reason |

The other key/values are `controller`, `frame_id`, `active_program`,
`program_sampling_rate_hz`, `continuous_trigger`, `achieved_rate_hz`, `profiles_published`,
`no_data_replies`, `consecutive_no_data`, `controller_errors`, `last_return_code`,
`reconnects`, `subscribers` and `seconds_since_profile`.

**Switching the laser off does not disconnect the driver.** The controller answers a profile
request it cannot satisfy with a well-formed 48-byte "no profile" message; that is
`connected_no_data`, it is throttled to one log line per `no_data_log_period`, and profiles
resume by themselves when the laser comes back on. Decoding that message as a truncated
profile instead throws, destroys the socket and rebuilds the connection once a second
forever, which makes the driver look dead.

## What rate to expect

Not the controller's program setting. This driver makes one TCP round trip per profile, so
the program's sampling rate is only a ceiling: against a program set to 1000 Hz on a real
LJ-V7080, the measured range is **136 to 218.9 Hz**. The node logs the rate it actually
achieves every `rate_report_period` seconds and publishes it as `achieved_rate_hz`.

## Tests

`colcon test --packages-select keyence_experimental`, or run the two gtest binaries in
`build/keyence_experimental/` directly. They need no controller and no network:

  - `test_profile_decode` -- the profile decoder's bounds checks, which exist because
    `num_profiles` comes off the wire; trusting it segfaults the node.
  - `test_response_classification` -- what `Client::sendReceive()` decides a reply IS before
    any decoder sees it, driven by a scripted `keyence::Client`: the no-profile reply, a
    truncated one, an error return code, a desynchronised length prefix, and a full profile.
