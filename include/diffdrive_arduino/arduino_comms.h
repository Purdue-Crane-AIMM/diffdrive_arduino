#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <libserial/SerialPort.h>
#include <cstring>

class ArduinoComms
{


public:

  ArduinoComms()
  {  }

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    setup(serial_device, baud_rate, timeout_ms);
  }

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void setMotorValues(int val_1, int val_2);
  /*
  void setGripperState(double position);
  void setActuatorState(double position);
*/
  bool connected() const { return serial_conn_.IsOpen(); }

  void disconnect();

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H