#pragma once
#include <Arduino.h>
#include <SPI.h>

class nRF24L01
{
public:
    const byte PAYLOAD_LENGTH = 12;

private:
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
    void read_from_register(const Registers _registerAddress)
    {
        byte value;
        digitalWrite(CSN, LOW);

        SPI.transfer(R_REGISTER bitor _registerAddress);
        value = SPI.transfer(NOP);

        digitalWrite(CSN, HIGH);

        Serial.println(value, BIN);
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

        // frequency channel setup
        write_to_register(RF_CH, 0b01111100); // 2524 MHz channel

        // data rate
        write_to_register(RF_SETUP, 0x01); // setup lna gain as this is a receiver

        // payload length setup
        write_to_register(RX_PW_P0, PAYLOAD_LENGTH); // 12 byte payload length

        // flush the rx fifo once
        command_SPI(FLUSH_RX);

        // ACTIVATE

        // CE high
        digitalWrite(CE, HIGH);
    }

    inline void read_rx_payload(byte *array)
    {
        PORTB and_eq 0b11111011;
        SPI.transfer(SPI_Commands::R_RX_PAYLOAD);
        for (unsigned i = 0; i < 12; ++i)
        {
            array[i] = SPI.transfer(SPI_Commands::NOP);
        }
        PORTB or_eq 0b00000100;
    }
};

nRF24L01 receiver;