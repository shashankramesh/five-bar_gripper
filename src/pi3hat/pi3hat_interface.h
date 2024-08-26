/*
 * file: pi3hat_interface.h
 *
 * Created: 23 Sep, 2021
 * Author : Aditya Sagi
 */

#ifndef __PI3HAT_INTERFACE_H__
#define __PI3HAT_INTERFACE_H__

#include <iostream>
#include <map>
#include <future>

#include "pi3hat/pi3hat.h"
#include "pi3hat/pi3hat_moteus_interface.h"

using MoteusInterface = mjbots::moteus::Pi3HatMoteusInterface;

using namespace mjbots;


typedef struct {
  uint8_t id;
  uint8_t mode;
  double position;  // rad
  double velocity;  // rad/sec
  double feedforward_torque; // Nm
  double kp_scale;
  double kd_scale;
  double watchdog_timeout; // sec
} MoteusCommand ;

typedef struct  {
  uint8_t id;
  uint8_t mode;
  double position; // rad
  double velocity; // rad/sec
  double torque;  // Nm
  double voltage; // V
  double temperature; // deg C
} MoteusResponse;

class Pi3HatInterface {
  public:

    Pi3HatInterface();
    ~Pi3HatInterface();

    void initialize(std::map<int, int> servo_bus_map);
    void initialize(void);
    std::vector<MoteusResponse> read(void);
    void write(std::vector<MoteusCommand> cmds);
    void stop(void);

  private:

    const int MAIN_CPU = 1;
    const int CAN_CPU  = 2;

    std::unique_ptr<MoteusInterface> moteus_interface_;

    MoteusInterface::Options moteus_options_;

    std::vector<MoteusInterface::ServoCommand> commands_;
    std::vector<MoteusInterface::ServoReply> saved_replies_;
    std::vector<MoteusInterface::ServoReply> replies_; 
    MoteusInterface::Data moteus_data_;

    std::future<MoteusInterface::Output> can_result_;

    std::map<int, int> servoBusMap();
};

#endif // __PI3HAT_INTERFACE_H__
