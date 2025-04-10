#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
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

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
}

void ArduinoComms::disconnect() {
  serial_conn_.Close();
}

void ArduinoComms::sendEmptyMsg() {
    std::string response = sendMsg("\r");
}

void ArduinoComms::setMotorValues(int val_1, int val_2) {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss.str());
    std::cout << "m " << val_1 << " " << val_2 << std::endl;
}

void ArduinoComms::setGripperState(double position) {
    if (position > 0.1) {   // CLose gripper
      std::stringstream ss;
      ss << "f 1";
      sendMsg(ss.str());
    } else {
      std::stringstream ss;
      ss << "f 0";
      sendMsg(ss.str());
    }
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