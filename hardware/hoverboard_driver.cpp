// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hoverboard_driver/hoverboard_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hoverboard_driver
{

  hoverboard_driver_node::hoverboard_driver_node(std::string _prefix) : Node("hoverboard_driver_node")
  {
    // These publishers are only for debugging purposes
    std::string vel_pub_left_node_name = _prefix + "hoverboard/left_wheel/velocity";
    vel_pub[left_wheel] = this->create_publisher<std_msgs::msg::Float64>(vel_pub_left_node_name, 3);

    std::string vel_pub_right_node_name = _prefix + "hoverboard/right_wheel/velocity";
    vel_pub[right_wheel] = this->create_publisher<std_msgs::msg::Float64>(vel_pub_right_node_name, 3);

    std::string pos_pub_left_node_name = _prefix + "hoverboard/left_wheel/position";
    pos_pub[left_wheel] = this->create_publisher<std_msgs::msg::Float64>(pos_pub_left_node_name, 3);

    std::string pos_pub_right_node_name = _prefix + "hoverboard/right_wheel/position";
    pos_pub[right_wheel] = this->create_publisher<std_msgs::msg::Float64>(pos_pub_right_node_name, 3);

    std::string cmd_pub_left_node_name = _prefix + "hoverboard/left_wheel/cmd";
    cmd_pub[left_wheel] = this->create_publisher<std_msgs::msg::Float64>(cmd_pub_left_node_name, 3);

    std::string cmd_pub_right_node_name = _prefix + "hoverboard/right_wheel/cmd";
    cmd_pub[right_wheel] = this->create_publisher<std_msgs::msg::Float64>(cmd_pub_right_node_name, 3);

    std::string voltage_pub_node_name = _prefix + "hoverboard/battery_voltage";
    voltage_pub = this->create_publisher<std_msgs::msg::Float64>(voltage_pub_node_name, 3);

    std::string temp_pub_node_name = _prefix + "hoverboard/temperature";
    temp_pub = this->create_publisher<std_msgs::msg::Float64>(temp_pub_node_name, 3);

    std::string connected_pub_node_name = _prefix + "hoverboard/connected";
    connected_pub = this->create_publisher<std_msgs::msg::Bool>(connected_pub_node_name, 3);

    declare_parameter("f",0.2);
    declare_parameter("p", 1.5);
    declare_parameter("i", 0.0);
    declare_parameter("d", 0.05);
    declare_parameter("i_clamp_min", -5.0);
    declare_parameter("i_clamp_max", 5.0);
    declare_parameter("antiwindup", false);
    get_parameter("f", pid_config.f);
    get_parameter("p", pid_config.p);
    get_parameter("i", pid_config.i);
    get_parameter("d", pid_config.d);
    get_parameter("i_clamp_min", pid_config.i_clamp_min);
    get_parameter("i_clamp_max", pid_config.i_clamp_max);
    get_parameter("antiwindup", pid_config.antiwindup);

    // register parameter change callback handle
    callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&hoverboard_driver_node::parametersCallback, this, std::placeholders::_1));
  }

  void hoverboard_driver_node::publish_vel(int wheel, double message)
  {
    std_msgs::msg::Float64 f;
    f.data = message;
    vel_pub[wheel]->publish(f);
  }

  void hoverboard_driver_node::publish_pos(int wheel, double message)
  {
    std_msgs::msg::Float64 f;
    f.data = message;
    pos_pub[wheel]->publish(f);
  }

  void hoverboard_driver_node::publish_cmd(int wheel, double message)
  {
    std_msgs::msg::Float64 f;
    f.data = message;
    cmd_pub[wheel]->publish(f);
  }

  void hoverboard_driver_node::publish_curr(int wheel, double message)
  {
    std_msgs::msg::Float64 f;
    f.data = message;
    curr_pub[wheel]->publish(f);
  }

  void hoverboard_driver_node::publish_voltage(double message)
  {
    std_msgs::msg::Float64 f;
    f.data = message;
    voltage_pub->publish(f);
  }

  void hoverboard_driver_node::publish_temp(double message)
  {
    std_msgs::msg::Float64 f;
    f.data = message;
    temp_pub->publish(f);
  }

  void hoverboard_driver_node::publish_connected(bool message)
  {
    std_msgs::msg::Bool f;
    f.data = message;
    connected_pub->publish(f);
  }

  rcl_interfaces::msg::SetParametersResult hoverboard_driver_node::parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Here update class attributes, do some actions, etc.
    for (const auto &param : parameters)
    {
      if (param.get_name() == "p")
      {
        pid_config.p = param.as_double();
        RCLCPP_INFO(get_logger(), "new value for PID P: %f", pid_config.p);
      }
      if (param.get_name() == "i")
      {
        pid_config.i = param.as_double();
        RCLCPP_INFO(get_logger(), "new value for PID I: %f", pid_config.i);
      }
      if (param.get_name() == "d")
      {
        pid_config.d = param.as_double();
        RCLCPP_INFO(get_logger(), "new value for PID D: %f", pid_config.d);
      }
      if (param.get_name() == "f")
      {
        pid_config.p = param.as_double();
        RCLCPP_INFO(get_logger(), "new value for PID F: %f", pid_config.f);
      }
      if (param.get_name() == "i_clamp_min")
      {
        pid_config.i_clamp_min = param.as_double();
        RCLCPP_INFO(get_logger(), "new value for PID I_CLAMP_MIN: %f", pid_config.i_clamp_min);
      }
      if (param.get_name() == "i_clamp_max")
      {
        pid_config.i_clamp_max = param.as_double();
        RCLCPP_INFO(get_logger(), "new value for PID I_CLAMP_MAX: %f", pid_config.i_clamp_max);
      }
      if (param.get_name() == "antiwindup")
      {
        pid_config.antiwindup = param.as_bool();
        RCLCPP_INFO(get_logger(), "new value for PID ANTIWINDUP: %i", pid_config.antiwindup);
      }

      // HINT: the PID configuration itself will be set by timer callback of hoverboard_driver class
    }

    return result;
  }

  hardware_interface::CallbackReturn hoverboard_driver::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    // read parameter from hoverboard_driver.ros2_control.xacro file
    wheel_radius = std::stod(info_.hardware_parameters["wheel_radius"]);
    max_velocity = std::stod(info_.hardware_parameters["max_velocity"]);
    wheel_base = std::stod(info_.hardware_parameters["wheel_base"]);
    port = info_.hardware_parameters["device"];
    prefix= info_.hardware_parameters["prefix"];
    l_wheel_name = info_.joints[0].name;
    r_wheel_name = info_.joints[1].name;

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("HoverBoardSystemHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    hardware_publisher = std::make_shared<hoverboard_driver_node>(prefix); // fire up the publisher node

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> hoverboard_driver::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // RCLCPP_INFO(rclcpp::get_logger("HoverBoardSystemHardware"), " ---------------------------------- joints size %d ",info_.joints.size());
      
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          l_wheel_name, hardware_interface::HW_IF_POSITION, &hw_positions_[left_wheel]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          r_wheel_name, hardware_interface::HW_IF_POSITION, &hw_positions_[right_wheel]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          l_wheel_name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[left_wheel]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          r_wheel_name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[right_wheel]));          
    // RCLCPP_INFO(rclcpp::get_logger("HoverBoardSystemHardware"), " ---------------------------------- joints name %s ",l_wheel_name.c_str());
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> hoverboard_driver::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          l_wheel_name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[left_wheel]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          r_wheel_name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[right_wheel]));

    return command_interfaces;
  }

  hardware_interface::CallbackReturn hoverboard_driver::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {

    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"), "Using port %s", port.c_str());
    std::cout << wheel_radius << std::endl; 
    // Convert m/s to rad/s
    max_velocity /= wheel_radius;

    low_wrap = ENCODER_LOW_WRAP_FACTOR * (ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    high_wrap = ENCODER_HIGH_WRAP_FACTOR * (ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
    last_wheelcountR = last_wheelcountL = 0;
    multR = multL = 0;

    first_read_pass_ = true;

    //  Init PID controller
    pids[0].init(hardware_publisher->pid_config.f, hardware_publisher->pid_config.p,
                 hardware_publisher->pid_config.i, hardware_publisher->pid_config.d,
                 hardware_publisher->pid_config.i_clamp_max, hardware_publisher->pid_config.i_clamp_min,
                 hardware_publisher->pid_config.antiwindup, max_velocity, -max_velocity);
    pids[0].setOutputLimits(-max_velocity, max_velocity);
    pids[1].init(hardware_publisher->pid_config.f, hardware_publisher->pid_config.p,
                 hardware_publisher->pid_config.i, hardware_publisher->pid_config.d,
                 hardware_publisher->pid_config.i_clamp_max, hardware_publisher->pid_config.i_clamp_min,
                 hardware_publisher->pid_config.antiwindup, max_velocity, -max_velocity);
    pids[1].setOutputLimits(-max_velocity, max_velocity);

    if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("hoverboard_driver"), "Cannot open serial port to hoverboard");
      exit(-1);
    }

    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B38400 | CS8 | CLOCAL | CREAD; //<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);

    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn hoverboard_driver::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    if (port_fd != -1)
      close(port_fd);

    RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type hoverboard_driver::read(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // to be able to compare times, we need to set last_read time to a correct time source
    // set the actual time as last_read, when it hasn't been set before (first attempt to read from harware)
    if (first_read_pass_ == true)
    {
      last_read = time;
      first_read_pass_ = false;
    }

    if (port_fd != -1)
    {
      unsigned char c;
      int i = 0, r = 0;
      // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
      //             "%s connect read ",prefix.c_str());
      while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024)
        protocol_recv(time, c);
      if (i > 0)
        last_read = time;

      if (r < 0 && errno != EAGAIN)
        RCLCPP_ERROR(rclcpp::get_logger("hoverboard_driver"),"Reading from serial %s failed: %d", port.c_str(), r);
    }

    if ((time - last_read).seconds() > 1)
    {
      //   ROS_FATAL("Timeout reading from serial %s failed", port.c_str());

      // publish false when not receiving serial data
      hardware_publisher->publish_connected(false);
    }
    else
    {
      // we must be connected - publish true
      hardware_publisher->publish_connected(true);
    }
    return hardware_interface::return_type::OK;
  }

  void hoverboard_driver::protocol_recv(const rclcpp::Time &time, char byte)
  {
    // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
    //           "%s byte: %d",prefix.c_str(),byte);      
    start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;
           
    // Read the start frame
    if (start_frame == START_FRAME)
    {
      p = (char *)&msg;
      *p++ = prev_byte;
      *p++ = byte;
      msg_len = 2;
    }
    else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback))
    {
      // Otherwise just read the message content until the end
      *p++ = byte;
      msg_len++;
    }
    if (msg_len == 18)
    {

      uint16_t checksum = (uint16_t)(msg.start ^
                                     msg.cmd1 ^
                                     msg.cmd2 ^
                                     msg.batVoltage ^
                                     msg.cmdLed ^
                                     msg.wheelR_cnt ^
                                     msg.wheelL_cnt ^
                                     msg.boardTemp 
                                     );

      // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
      //               "%s start: 0X%04X, cmd1: 0X%04X, cmd2: 0X%04X, voltage: 0X%04X, cmdLed: 0X%04X, wheelR: 0X%04X, wheelL: 0X%04X, Temp: 0X%04X",
      //               prefix.c_str(),msg.start,msg.cmd1,msg.cmd2,msg.batVoltage,msg.cmdLed,msg.wheelR_cnt,msg.wheelL_cnt, msg.boardTemp);

      if (msg.start == START_FRAME && msg.checksum == checksum)
      {
        // hardware_publisher->publish_voltage((double)msg.batVoltage / 100.0);
        hardware_publisher->publish_temp((double)msg.boardTemp / 10.0);

        // // Convert RPM to RAD/S
        // hw_velocities_[left_wheel] = direction_correction * (abs(wheelL_speed) * 0.10472);
        // hw_velocities_[right_wheel] = direction_correction * (abs(wheelR_speed) * 0.10472);

        hw_velocities_[left_wheel] = direction_correction * (abs(msg.wheelL_cnt)/(double)TICKS_PER_LROTATION * 0.10472);
        hw_velocities_[right_wheel] = direction_correction * (abs(msg.wheelR_cnt)/(double)TICKS_PER_RROTATION * 0.10472);
        hardware_publisher->publish_vel(left_wheel, hw_velocities_[left_wheel]);
        hardware_publisher->publish_vel(right_wheel, hw_velocities_[right_wheel]);
        // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),"L : %04x, R: %04x", hw_velocities_[left_wheel],hw_velocities_[right_wheel]);
        // Process encoder values and update odometry    
        on_encoder_update(time, msg.wheelR_cnt, msg.wheelL_cnt);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("hoverboard_driver"), "%s Hoverboard checksum mismatch: %d vs %d",prefix.c_str(), msg.checksum, checksum);
      }
      msg_len = 0;
    }
    prev_byte = byte;
  }

  hardware_interface::return_type hoverboard_driver::hoverboard_driver::write(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    if (port_fd == -1)
    {
      RCLCPP_ERROR(rclcpp::get_logger("hoverboard_driver"), "Attempt to write on closed serial");
      return hardware_interface::return_type::ERROR;
    }
    // Inform interested parties about the commands we've got
    hardware_publisher->publish_cmd(left_wheel, hw_commands_[left_wheel]);
    hardware_publisher->publish_cmd(right_wheel, hw_commands_[right_wheel]);

    // Set PID Parameters
    pids[0].setParameters(hardware_publisher->pid_config.f, hardware_publisher->pid_config.p,
                          hardware_publisher->pid_config.i, hardware_publisher->pid_config.d,
                          hardware_publisher->pid_config.i_clamp_max, hardware_publisher->pid_config.i_clamp_min,
                          hardware_publisher->pid_config.antiwindup);
    pids[1].setParameters(hardware_publisher->pid_config.f, hardware_publisher->pid_config.p,
                          hardware_publisher->pid_config.i, hardware_publisher->pid_config.d,
                          hardware_publisher->pid_config.i_clamp_max, hardware_publisher->pid_config.i_clamp_min,
                          hardware_publisher->pid_config.antiwindup);

    // calculate PID values
    double pid_outputs[2];
    pid_outputs[0] = pids[0](hw_velocities_[left_wheel], hw_commands_[left_wheel], period);
    pid_outputs[1] = pids[1](hw_velocities_[left_wheel], hw_commands_[right_wheel], period);

    // // Convert PID outputs in RAD/S to RPM
    double set_speed[2] = {
       pid_outputs[0] / 0.10472,
       pid_outputs[1] / 0.10472};

    //  double set_speed[2] = {
    //        hw_commands_[left_wheel] / 0.10472,
    //        hw_commands_[right_wheel] / 0.10472
    //  };

    // Calculate steering from difference of left and right
    const double speed = (set_speed[0] + set_speed[1]) / 2.0;
    const double steer = (set_speed[0] - set_speed[1]) / wheel_base;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
    //             "%s , Start: 0x%X, Steer: %d, Speed: %d",prefix.c_str(), (int16_t)command.start, (int16_t)command.steer,command.speed);


    int rc = ::write(port_fd, (const void *)&command, sizeof(command));
    if (rc < 0)
    {
      RCLCPP_ERROR(rclcpp::get_logger("hoverboard_driver"), "%s Error writing to hoverboard serial port", prefix.c_str());
    }
    return hardware_interface::return_type::OK;
  }

  void hoverboard_driver::on_encoder_update(const rclcpp::Time &time, int16_t right, int16_t left)
  {
    // std::cout << right << left << "\n";
    double posL = 0.0, posR = 0.0;
    // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
    //           "%s right: %d, left: %d ",prefix.c_str(),right,left);

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
      multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
      multR--;
    posR = right + multR * (ENCODER_MAX - ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
      multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
      multL--;
    posL = left + multL * (ENCODER_MAX - ENCODER_MIN);
    last_wheelcountL = left;

    // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
    // This section accumulates ticks even if board shuts down and is restarted
    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;

    // IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
    //(the board seems to often report 1-3 ticks on startup instead of zero)
    // reset the last read ticks to the startup values
    if ((time - last_read).seconds() > 0.2 && abs(posL) < 5 && abs(posR) < 5)
    {
      lastPosL = posL;
      lastPosR = posR;
    }
    double posLDiff = 0;
    double posRDiff = 0;

    // if node is just starting keep odom at zeros
    if (nodeStartFlag)
    {
      nodeStartFlag = false;
    }
    else
    {
      posLDiff = posL - lastPosL;
      posRDiff = posR - lastPosR;
    }

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;

    countR -= lastPosR;
    countL += lastPosL;

    // RCLCPP_INFO(rclcpp::get_logger("hoverboard_driver"),
    //               " %s __ countR : %d, countL : %d",prefix.c_str(),countR, countL);

    // Convert position in accumulated ticks to position in radians
    // hw_positions_[left_wheel] = 2.0 * M_PI * lastPubPosL / (double)TICKS_PER_ROTATION;
    // hw_positions_[right_wheel] = 2.0 * M_PI * lastPubPosR / (double)TICKS_PER_ROTATION; 

    hw_positions_[left_wheel] =  2.0 * M_PI * countL / (double)TICKS_PER_LROTATION;
    hw_positions_[right_wheel] =  2.0 * M_PI * countR / (double)TICKS_PER_RROTATION;      

    // std::cout << count << std::endl;
    hardware_publisher->publish_pos(left_wheel, hw_positions_[left_wheel]);
    hardware_publisher->publish_pos(right_wheel, hw_positions_[right_wheel]);
  }

} // namespace hoverboard_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    hoverboard_driver::hoverboard_driver, hardware_interface::SystemInterface)