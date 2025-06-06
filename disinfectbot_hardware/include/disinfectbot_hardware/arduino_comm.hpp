#pragma once

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_cmd(const std::string &msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send + "\n");

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  void read_encoder_values(double &l_pos, double &r_pos, double &l_vel, double &r_vel)
  {
    std::string response = send_cmd("get encoder");
    size_t p1 = response.find(',');
    size_t p2 = response.find(',', p1 + 1);
    size_t p3 = response.find(',', p2 + 1);

    l_pos = std::atof(response.substr(0, p1).c_str());
    r_pos = std::atof(response.substr(p1 + 1, p2 - p1 - 1).c_str());
    l_vel = std::atof(response.substr(p2 + 1, p3 - p2 - 1).c_str());
    r_vel = std::atof(response.substr(p3 + 1).c_str());
  }

  void read_uv_state(bool &state, double &power)
  {
    std::string response = send_cmd("get uv_state");
    size_t comma_pos = response.find(',');

    int state_int = std::atoi(response.substr(0, comma_pos).c_str());
    state = (state_int != 0);  // convert to bool cleanly

    power = std::atof(response.substr(comma_pos + 1).c_str());
  }

  void set_motor_velocities(double vel_l, double vel_r)
  {
    send_cmd("set speed_rad_s " + std::to_string(vel_l) + " " + std::to_string(vel_r));
  }

  void set_uv_state(bool state)
  {
    std::string cmd = "set uv_state " + std::to_string(state ? 1 : 0);
    send_cmd(cmd);
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};
