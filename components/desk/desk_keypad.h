#include "esphome.h"
#include <bitset>

class DeskKeypad : public Component, public UARTDevice, public Sensor
{
protected:
  static const byte REQUEST_HEADER = 0xA5;
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
  byte collectedBytes[4];
  byte lastCommand;
  uint32_t lastUpdate = 0;
  bool validMessage = false;

  // a little bit of fun
  uint32_t lastAnimUpdate = 0;
  byte animIndex = 0;
  byte anim[6] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20};

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

      if (incomingByte == this->REQUEST_HEADER)
      {
        this->messageLength = 0;
        this->validMessage = false;
      }

      if (this->messageLength < 4)
      {
        this->collectedBytes[this->messageLength] = incomingByte;
        this->messageLength++;
      }
      else
      {
        // Check if checksum is correct
        this->validMessage = (incomingByte == ((this->collectedBytes[1] + this->collectedBytes[2] + this->collectedBytes[3]) & 0xff));
      }

      if (this->validMessage)
      {
        // command is in the third received byte
        if (this->collectedBytes[2] != this->lastCommand)
        {
          this->publish_state(this->collectedBytes[2]);
          this->lastCommand = this->collectedBytes[2];
          this->lastUpdate = millis();
        }

        // --- run ack against the screen logic ---

        // "timeout": send screen into deep sleep mode after 5 seconds
        if (millis() - this->lastUpdate > 5000)
        {
          this->write_array({0x5A, 0xFF, 0xFF, 0xFF, 0x00, 0xFD});
          continue;
        }

        // update the animation only if at least 66ms have passed since the last update
        if (millis() - this->lastAnimUpdate > 66)
        {
          animIndex = (this->animIndex + 1) % 6;
          this->lastAnimUpdate = millis();
        }

        byte packet[6] = {0x5A, this->anim[this->animIndex], this->anim[this->animIndex], this->anim[this->animIndex], 0x00};

        // attach the checksum
        packet[5] = (packet[1] + packet[2] + packet[3] + packet[4]) & 0xff;

        this->write_array(packet, 6);
      }
    }
  }
};
