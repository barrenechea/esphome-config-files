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
  Command lastCommand;
  uint32_t lastUpdate = 0;
  bool validMessage = false;

  // a little bit of fun
  uint32_t lastAnimUpdate = 0;
  uint8_t animIndex = 0;
  uint8_t anim[6] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20};

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
      uint8_t incomingByte = this->read();

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
        Command returnCommand = (Command)collectedBytes[2];

        if (returnCommand != lastCommand)
        {
          this->publish_state(returnCommand);
          lastCommand = returnCommand;
          lastUpdate = millis();
        }

        // run ack against the screen
        if (millis() - lastUpdate > 5000)
        {
          // send screen to sleep
          this->write_array({0x5A, 0xFF, 0xFF, 0xFF, 0x00, 0xFD});
          continue;
        }

        // update the animation only if at least 66ms have passed since the last update
        if (millis() - lastAnimUpdate > 66)
        {
          animIndex = (animIndex + 1) % 6;
          lastAnimUpdate = millis();
        }

        uint8_t packet[6] = {0x5A, anim[animIndex], anim[animIndex], anim[animIndex], 0x00};

        // attach the checksum
        packet[5] = (packet[1] + packet[2] + packet[3] + packet[4]) & 0xff;

        this->write_array(packet, 6);
      }
    }
  }
};
