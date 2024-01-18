#include "esphome.h"
#include <bitset>

class DeskHeightSensor : public Component, public UARTDevice, public Sensor
{
protected:
  static const byte REQUEST_HEADER = 0x5A;

  std::string lastPublished;
  byte collectedBytes[5];

  int messageLength = 0;
  byte msg_type;
  bool validMessage = false;

  char bcdDigitToChar(byte bcd_code)
  {
    auto withoutTopMostBit = bcd_code & 0x7f; // we don't care about the top-most bit
    switch (withoutTopMostBit)
    {
    case 0x3F:
      return '0';
    case 0x06:
      return '1';
    case 0x5B:
      return '2';
    case 0x4F:
      return '3';
    case 0x66:
      return '4';
    case 0x6D:
      return '5';
    case 0x7D:
      return '6';
    case 0x07:
      return '7';
    case 0x7F:
      return '8';
    case 0x6F:
      return '9';
    case 0x40:
      return '-';
    case 0x79:
      return 'E';
    case 0x50:
      return 'r';
    case 0x31:
      return 'T';
    case 0x76:
      return 'H';
    case 0x5C:
      return 'o';
    case 0x78:
      return 't';
    case 0x00:
      return ' ';
    default:
      return '?';
    }
  }

public:
  DeskHeightSensor(UARTComponent *parent) : UARTDevice(parent) {}

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

      // First byte, start of a packet
      if (incomingByte == this->REQUEST_HEADER)
      {
        // Reset message length
        this->messageLength = 0;
        this->validMessage = false;
      }

      if (this->messageLength < 5)
      {
        this->collectedBytes[this->messageLength] = incomingByte;
        this->messageLength++;
      }
      else
      {
        // Check if checksum is correct
        this->validMessage = (incomingByte == ((this->collectedBytes[1] + this->collectedBytes[2] + this->collectedBytes[3] + this->collectedBytes[4]) & 0xff));
      }

      if (this->validMessage)
      {
        auto first_digit = this->bcdDigitToChar(this->collectedBytes[1]);
        auto second_digit = this->bcdDigitToChar(this->collectedBytes[2]);
        auto third_digit = this->bcdDigitToChar(this->collectedBytes[3]);

        auto text = std::string(1, first_digit) + std::string(1, second_digit) + std::string(1, third_digit);
        
        if (this->lastPublished != text)
        {
          this->publish_state(text);
          this->lastPublished = text;
        }
      }
    }
  }
};
