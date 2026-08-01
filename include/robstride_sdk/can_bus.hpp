// Copyright 2026
// Apache License, Version 2.0

#ifndef ROBSTRIDE_SDK__CAN_BUS_HPP_
#define ROBSTRIDE_SDK__CAN_BUS_HPP_

#include <linux/can.h>
#include <atomic>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "robstride_sdk/robstride_motor.hpp"

namespace robstride_sdk
{

/// @brief One SocketCAN raw socket bound to a single interface (e.g. "can0"),
/// used non-blocking with an epoll-driven background read thread and
/// sendmmsg()/recvmmsg() batched I/O. All command encode/decode happens in
/// RobstrideMotor; this class only moves frames between the socket and the
/// registered motors, so read()/write() calls in the hardware interface
/// never block on CAN I/O.
class CanBus
{
public:
  explicit CanBus(std::string ifname);
  ~CanBus();

  CanBus(const CanBus &) = delete;
  CanBus & operator=(const CanBus &) = delete;

  /// @brief Opens and configures the socket (non-blocking, extended-frame,
  /// tuned send/receive buffers). Must be called before Start().
  /// @return false with errno set on failure.
  bool Open();
  void Close();

  /// @brief Associates a motor with its CAN id for feedback dispatch.
  /// Must be called before Start(); not safe to call concurrently with the
  /// read thread.
  void RegisterMotor(RobstrideMotor * motor);

  /// @brief Starts the background epoll/recvmmsg read thread.
  void Start();
  /// @brief Stops and joins the background read thread.
  void Stop();

  /// @brief Sends a batch of frames in a single sendmmsg() syscall.
  /// @return number of frames successfully queued to the kernel.
  int SendFrames(const std::vector<can_frame> & frames);

  /// @brief Convenience wrapper for a single frame.
  bool SendFrame(const can_frame & frame);

  const std::string & interface_name() const {return ifname_;}

private:
  void ReadThreadMain();
  void DispatchFrame(const can_frame & frame);

  std::string ifname_;
  int fd_ = -1;
  int epoll_fd_ = -1;
  std::atomic<bool> running_{false};
  std::thread read_thread_;
  std::unordered_map<uint8_t, RobstrideMotor *> motors_;
};

}  // namespace robstride_sdk

#endif  // ROBSTRIDE_SDK__CAN_BUS_HPP_
