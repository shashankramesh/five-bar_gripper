/*
 * file: pi3hat_interface.cpp
 * 
 * Created: 23 Sep, 2021
 * Author : Aditya Sagi
 */

#include <iostream>
#include <algorithm>
#include <vector>
#include <limits>

#include "pi3hat/pi3hat_interface.h"


Pi3HatInterface::Pi3HatInterface()
{}

Pi3HatInterface::~Pi3HatInterface()
{}

std::map<int,int> Pi3HatInterface::servoBusMap(void)
{
  return {
    {1, 1}
  };
}

void Pi3HatInterface::initialize(void)
{
  initialize(servoBusMap());
}


void Pi3HatInterface::initialize(std::map<int, int> servo_bus_map)
{
  // Start the Moteus Interface
  moteus_options_.cpu = CAN_CPU;
  moteus_options_.servo_bus_map = servo_bus_map;
  moteus_interface_.reset(new MoteusInterface(moteus_options_));

  for (const auto &pair : moteus_options_.servo_bus_map)
  {
    commands_.push_back({});
    commands_.back().id = pair.first;
  }
  replies_.resize(commands_.size());

  // Set the resolution for all the commands being sent to the moteus
  // controllers
  moteus::PositionResolution res;
  res.position = moteus::Resolution::kInt16;
  res.velocity = moteus::Resolution::kInt16;
  res.feedforward_torque = moteus::Resolution::kFloat;
  res.kp_scale = moteus::Resolution::kFloat;
  res.kd_scale = moteus::Resolution::kFloat;
  res.maximum_torque = moteus::Resolution::kIgnore;
  res.stop_position = moteus::Resolution::kIgnore;
  res.watchdog_timeout = moteus::Resolution::kIgnore;
  for (auto &cmd : commands_)
  {
    cmd.resolution = res;
  }

  moteus_data_.commands = {commands_.data(), commands_.size()};
  moteus_data_.replies = {replies_.data(), replies_.size()};
}


std::vector<MoteusResponse> Pi3HatInterface::read(void)
{
  std::vector<MoteusResponse> resp;

  if (can_result_.valid())
  {
    // Now we get the result of our last query
    const auto current_values = can_result_.get();

    // We copy out the results we just got.
    for(auto it = replies_.begin(); it!=replies_.end(); it++)
    {
      MoteusResponse mresp;
      mresp.id          = it->id;
      mresp.mode        = (uint8_t) it->result.mode;
      mresp.position    = it->result.position * (-2*M_PI); // convert to radians
      mresp.velocity    = it->result.velocity * (-2*M_PI); // convert to radians/sec
      mresp.torque      = it->result.torque * (-1);
      mresp.temperature = it->result.temperature;
      mresp.voltage     = it->result.voltage;

      resp.push_back(mresp);
    } 
    return resp;
  }
  return {};
}

void Pi3HatInterface::stop(void)
{

  // Update the commands
  for (auto scmd = commands_.begin(); scmd != commands_.end(); scmd++)
  {
    scmd->mode                        = moteus::Mode::kStopped;
    scmd->position.position           = std::numeric_limits<double>::quiet_NaN();
    scmd->position.velocity           = std::numeric_limits<double>::quiet_NaN();
    scmd->position.feedforward_torque = std::numeric_limits<double>::quiet_NaN();
    scmd->position.kp_scale           = std::numeric_limits<double>::quiet_NaN();
    scmd->position.kd_scale           = std::numeric_limits<double>::quiet_NaN();
    scmd->position.watchdog_timeout   = std::numeric_limits<double>::quiet_NaN();
  }

  // Sends the commands over to the Pi3Hat interface thread
  // with the promise of returning with a result
  auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
  moteus_interface_->Cycle(
      moteus_data_,
      [promise](const MoteusInterface::Output &output)
      {
      // This is called from an arbitrary thread, so we just set
      // the promise value here.
      promise->set_value(output);
      });
  can_result_ = promise->get_future();

  return;
}


void Pi3HatInterface::write(std::vector<MoteusCommand> cmds)
{

  // Update the commands
  for (auto &cmd : cmds)
  {
    auto find_id = [cmd](MoteusInterface::ServoCommand scmd) {return cmd.id == scmd.id;};
    auto scmd = std::find_if(commands_.begin(), commands_.end(), find_id); 
    scmd->mode                        = (mjbots::moteus::Mode) cmd.mode;
    scmd->position.position           = -cmd.position / (2*M_PI); // convert to number of rotations
    scmd->position.velocity           = -cmd.velocity / (2*M_PI); // conver to number of rotations per second.
    scmd->position.feedforward_torque = -cmd.feedforward_torque;
    scmd->position.kp_scale           = cmd.kp_scale;
    scmd->position.kd_scale           = cmd.kd_scale;
    scmd->position.watchdog_timeout   = cmd.watchdog_timeout;
  }

  // Sends the commands over to the Pi3Hat interface thread
  // with the promise of returning with a result
  auto promise = std::make_shared<std::promise<MoteusInterface::Output>>();
  moteus_interface_->Cycle(
      moteus_data_,
      [promise](const MoteusInterface::Output &output)
      {
      // This is called from an arbitrary thread, so we just set
      // the promise value here.
      promise->set_value(output);
      });
  can_result_ = promise->get_future();

  return;
}
