#pragma once
#include <Arduino.h>
#include <SPI.h>

struct receiver_output
{
    int16_t throttle, roll, pitch, yaw, camera;
    bool autopilot;
};
class nRF24L01
{
public:
    const byte PAYLOAD_LENGTH = 11;

private:
    byte array[11];
    const byte CSN = 10, CE = 9; // pins 9 through 13 are used for SPI communication with the radio receiver

private:
    enum Registers : byte
    {
        CONFIG = 0X00,
        EN_AA,
        EN_RXADDR,
        SETUP_AW,
        SETUP_RETR,
        RF_CH,
        RF_SETUP,
        STATUS,
        OBSERVE_TX,
        CD,
        RX_ADDR_P0,
        RX_ADDR_P1,
        RX_ADDR_P2,
        RX_ADDR_P3,
        RX_ADDR_P4,
        RX_ADDR_P5,
        TX_ADDR,
        RX_PW_P0,
        RX_PW_P1,
        RX_PW_P2,
        RX_PW_P3,
        RX_PW_P4,
        RX_PW_P5,
        FIFO_STATUS,
        DYNPD = 0X1C,
        FEATURE
    };
    enum SPI_Commands : byte
    {
        R_REGISTER = 0X00,
        W_REGISTER = 0X20,
        R_RX_PAYLOAD = 0B01100001,
        W_TX_PAYLOAD = 0B10100000,
        FLUSH_TX = 0B11100001,
        FLUSH_RX = 0B11100010,
        REUSE_TX_PL = 0B11100011,
        ACTIVATE = 0B01010000,
        R_RX_PL_WID = 0B01100000,
        W_ACK_PAYLOAD = 0B10101000,
        W_TX_PAYLOAD_NOACK = 0B10110000,
        NOP = 0XFF
    };

private:
    void write_to_register(const Registers _registerAddress, const byte _data)
    {
        digitalWrite(CSN, LOW);

        SPI.transfer(W_REGISTER bitor _registerAddress);
        SPI.transfer(_data);

        digitalWrite(CSN, HIGH);
    }

    void command_SPI(const SPI_Commands _command)
    {
        digitalWrite(CSN, LOW);
        SPI.transfer(_command);
        digitalWrite(CSN, HIGH);
    }
    // for debugging only.
    void read_from_register(const byte _registerAddress, const unsigned _numberOfBytes = 1)
    {
        byte value;
        digitalWrite(CSN, LOW);

        SPI.transfer(R_REGISTER bitor _registerAddress);
        for (unsigned i = 0; i < _numberOfBytes; ++i)
        {
            value = SPI.transfer(NOP);
            Serial.print(value, HEX);
            Serial.print(' ');
        }
        Serial.println();
        digitalWrite(CSN, HIGH);
    }

public:
    void setup()
    {
        // begin the spi object
        SPI.begin();

        // configure the CSN pin to start from high
        pinMode(CSN, OUTPUT);
        digitalWrite(CSN, HIGH);

        // init the CE PIN as low
        pinMode(CE, OUTPUT);
        digitalWrite(CE, LOW);

        // set the SPI settings
        SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); // max SPI speed 8Mbps

        // declare as receiver
        write_to_register(CONFIG, 0b00001111); // enable CRC, 2 byte CRC length, power up, set as receiver

        // disable enhanced shockburst
        write_to_register(EN_AA, 0x00);

        // only enable data pipe 0
        write_to_register(EN_RXADDR, 0x01);

        // frequency channel setup
        write_to_register(RF_CH, 0b01111100); // 2524 MHz channel

        // data rate
        write_to_register(RF_SETUP, 0x01); // setup lna gain as this is a receiver

        // payload length setup
        write_to_register(RX_PW_P0, PAYLOAD_LENGTH); // 11 byte payload length

        // give the device a unique address 0xD6D6D6D6D6 (same as Transmitter)
        digitalWrite(CSN, LOW);
        SPI.transfer(W_REGISTER bitor RX_ADDR_P0);
        for (unsigned i = 0; i < 5; ++i)
        {
            SPI.transfer(0xD6);
        }
        digitalWrite(CSN, HIGH);

        // flush the rx fifo once
        command_SPI(FLUSH_RX);

        // CE high
        digitalWrite(CE, HIGH);
    }

    inline void read_rx_payload(receiver_output &_receiverData)
    {
        PORTB and_eq 0b11111011;
        SPI.transfer(SPI_Commands::R_RX_PAYLOAD);
        for (unsigned i = 0; i < PAYLOAD_LENGTH; ++i)
        {
            array[i] = SPI.transfer(SPI_Commands::NOP);
        }
        PORTB or_eq 0b00000100;

        // decompose the array into 6 channels and save it
        // throttle, roll, pitch, yaw, camera, autopilot
        // array[0] = autopilot
        // array[1] = camera least significant byte
        // array[2] = camera most significant byte
        _receiverData.autopilot = array[0];
        // _receiverData.autopilot = _receiverData.autopilot ? true : false;

        _receiverData.camera = array[2];
        _receiverData.camera = array[1] bitor (_receiverData.camera << 8);

        _receiverData.yaw = array[4];
        _receiverData.yaw = array[3] bitor (_receiverData.yaw << 8);

        _receiverData.pitch = array[6];
        _receiverData.pitch = array[5] bitor (_receiverData.pitch << 8);

        _receiverData.roll = array[8];
        _receiverData.roll = array[7] bitor (_receiverData.roll << 8);

        _receiverData.throttle = array[10];
        _receiverData.throttle = array[9] bitor (_receiverData.throttle << 8);
    }
};
nRF24L01 receiver;