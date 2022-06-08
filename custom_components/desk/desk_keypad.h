#include "esphome.h"
#include <bitset>

class DeskKeypad : public Component, public UARTDevice, public Sensor
{
private:
  static const uint8_t REQUEST_HEADER = 0xA5;
  enum Command
  {
    Status = 0x00,
    Up = 0x20,
    Down = 0x40,
    M = 0x01,
    M1 = 0x02,
    M2 = 0x04,
    M3 = 0x08,
    T = 0x10,
    Reset = 0x60 // just a sum of Up and Down
  };

  int messageLength = 0;
  uint8_t collectedBytes[4];
  Command mReturnCommand;
  Command lastPublished = Command::Status;
  bool validMessage = false;

public:
  DeskKeypad(UARTComponent *parent) : UARTDevice(parent) {}

  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  void setup() override
  {
    // nothing to do here
  }

  void loop() override
  {
    while (this->available())
    {
      byte incomingByte = this->read();

      if (incomingByte == REQUEST_HEADER)
      {
        messageLength = 0;
        validMessage = false;
      }

      if (messageLength < 4)
      {
        collectedBytes[messageLength] = incomingByte;
        messageLength++;
      }
      else
      {
        // Check if checksum is correct
        validMessage = (incomingByte == ((collectedBytes[1] + collectedBytes[2] + collectedBytes[3]) & 0xff));
      }

      if (validMessage)
      {
        // command is in the third received byte
        mReturnCommand = (Command)collectedBytes[2];

        if (mReturnCommand != lastPublished)
        {
          this->publish_state(mReturnCommand);
          lastPublished = mReturnCommand;
        }

        // Send something to the screen so it doesn't think we are dead
        this->write_array({0x5A, 0x00, 0x00, 0x00, 0x01, 0x01});
      }
    }
  }
};
