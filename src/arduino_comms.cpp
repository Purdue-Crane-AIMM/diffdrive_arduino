#include "diffdrive_arduino/arduino_comms.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <experimental/filesystem>  // Use experimental filesystem for C++14

namespace fs = std::experimental::filesystem;

// Helper function to find the device port based on device_identifier
static std::string find_device_port(const std::string &device_identifier) {
  fs::path serialPath("/dev/serial/by-id");
  if (!fs::exists(serialPath)) {
    throw std::runtime_error("/dev/serial/by-id does not exist");
  }
  for (const auto &entry : fs::directory_iterator(serialPath)) {
    std::string filename = entry.path().filename().string();
    // Modified to search for idVendor 0x2341 in the filename
    if (filename.find(device_identifier) != std::string::npos) {
      return fs::canonical(entry.path()).string();
    }
  }
  throw std::runtime_error("Device with identifier " + device_identifier + " not found.");
}

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

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    timeout_ms_ = timeout_ms;
    std::string port;
    try {
      port = find_device_port("Arduino");
    } catch (const std::exception &e) {
      std::cerr << "Error: " << e.what() 
                << "\nFalling back to provided device string." << std::endl;
      port = serial_device;
    }
    serial_conn_.Open(port);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
}

void ArduinoComms::disconnect() {
  serial_conn_.Close();
}

void ArduinoComms::sendEmptyMsg() {
    std::string response = sendMsg("\r");
}

void ArduinoComms::setMotorValues(long pwm_left, long pwm_right) {
    std::stringstream ss;
    ss << "m " << pwm_left << " " << pwm_right << "\r";
    sendMsg(ss.str());
    std::cout << "m " << pwm_left << " " << pwm_right << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Sending motor command: %s", ss.str().c_str());
}

void ArduinoComms::setGripperState(int8_t position) {
    std::stringstream ss;
    if (position >= 1) {   // Close gripper
      ss << "f 1";
    } else {
      ss << "f 0";
    }
    // RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Sending gripper command: %s", ss.str().c_str());
    sendMsg(ss.str());
}

// void ArduinoComms::setActuatorState(double position) {
//     if (position > 0.1) {
//       std::stringstream ss;
//       ss << "g 1";
//       sendMsg(ss.str());
//     } else {
//       std::stringstream ss;
//       ss << "g 0";
//       sendMsg(ss.str());
//     }
// }

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output) {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    } catch (const LibSerial::ReadTimeout&) {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output) {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
}