/*
 * hunter_base.hpp
 *
 * Created on: Apr 01, 2020 09:43
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef HUNTER_BASE_HPP
#define HUNTER_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>

#include "ugv_sdk/mobile_base.hpp"

//#include "ugv_sdk/hunter/hunter_protocol.h"
#include "ugv_sdk/proto/agx_msg_parser.h"
#include "ugv_sdk/hunter/hunter_types.hpp"

namespace westonrobot {
class HunterBase : public MobileBase {
 public:
  HunterBase() : MobileBase(){};
  ~HunterBase() = default;

  // get robot state
  HunterState GetHunterState();

  // motion control
  void SetMotionCommand(double linear_vel, double angular_vel,
                        double steering_angle,
                        HunterMotionCmd::FaultClearFlag fault_clr_flag =
                            HunterMotionCmd::FaultClearFlag::NO_FAULT);

 private:
  // cmd/status update related variables
  std::mutex hunter_state_mutex_;
  std::mutex motion_cmd_mutex_;
  std::mutex mode_cmd_mutex_;
  std::mutex pack_mode_cmd_mutex_;

  HunterState hunter_state_;
  HunterMotionCmd current_motion_cmd_;

  // internal functions
  void SendRobotCmd() override;
  void ParseCANFrame(can_frame *rx_frame) override;
  void ParseUARTBuffer(uint8_t *buf, const size_t bufsize,
                       size_t bytes_received) override{};

  void SendMotionCmd(uint8_t count);
  void SendModeCtl();
  void SetParkMode();
  void NewStatusMsgReceivedCallback(const AgxMessage &msg);

 public:
  static void UpdateHunterState(const AgxMessage &status_msg,
                                HunterState &state);
};
}  // namespace westonrobot

#endif /* HUNTER_BASE_HPP */
