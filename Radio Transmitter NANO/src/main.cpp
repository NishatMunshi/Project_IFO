#include <Arduino.h>
#include <SPI.h>

// global variables
constexpr uint8_t CSN = 10, CE = 9; // this way pins 9 through 13 are used for SPI communication with the radio transmitter
constexpr uint8_t PAYLOAD_LENGTH = 12;
uint16_t throttle, roll, pitch, yaw, camera, autopilot;
uint8_t array[PAYLOAD_LENGTH];

uint32_t timer_i, timer_f;
byte value;

enum Registers : uint8_t
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
enum SPI_Commands : uint8_t
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

void write_to_register(const uint8_t _registerAddress, const uint8_t _data)
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
void read_from_register(const byte _registerAddress)
{
	byte value;
	digitalWrite(CSN, LOW);

	SPI.transfer(R_REGISTER bitor _registerAddress);
	value = SPI.transfer(NOP);

	digitalWrite(CSN, HIGH);

	Serial.println(value, BIN);
}

void setup()
{
	// begin the objects
	Serial.begin(9600);
	SPI.begin();

	// configure the CSN pin to start from high
	pinMode(CSN, OUTPUT);
	digitalWrite(CSN, HIGH);

	// init the CE PIN as low
	pinMode(CE, OUTPUT);
	digitalWrite(CE, LOW);

	pinMode(8, INPUT);

	// set the SPI settings
	SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

	// declare as transmitter
	write_to_register(CONFIG, 0b00001110); // enable CRC, 2 byte CRC length, power up, set as transmitter

	// disable enhanced shockburst
	write_to_register(EN_AA, 0x00);

	// frequency channel setup
	write_to_register(RF_CH, 0b01111100); // 2524 MHz channel

	// data rate
	write_to_register(RF_SETUP, 0x00);

	// payload length setup
	write_to_register(RX_PW_P0, PAYLOAD_LENGTH); // 12 byte payload length

	// flush the tx fifo
	command_SPI(FLUSH_TX);

	// CE high
	digitalWrite(CE, HIGH);
}
void loop()
{
	// timer_i = micros();

	// resuse last transmitted data packet until I do the calculations 
	PORTB and_eq 0b11111011;
	SPI.transfer(SPI_Commands::REUSE_TX_PL);
	PORTB or_eq 0b00000100;
	// --------------------------------------- 4 microseconds

	// read analog values
	throttle = analogRead(A0);
	roll = analogRead(A1);
	pitch = analogRead(A2);
	yaw = analogRead(A3);
	camera = analogRead(A4);
	autopilot = (PINB bitand 0b00000001) ? 0x5555 : 0x0000; // digitalRead(8)
	// -------------------------------------- 564 microseconds

	// map the values into pulsewidth,
	throttle = throttle * 1.8142717497556207 + 544;
	roll = roll * 1.8142717497556207 + 544;
	pitch = pitch * 1.8142717497556207 + 544;
	yaw = yaw * 1.8142717497556207 + 544;
	camera = camera * 1.8142717497556207 + 544; // 1500 us pulse is the centre position of a servo.
	// -------------------------------------- 120 microseconds

	// process the data into 12 byte array
	// throttle, roll, pitch, yaw, camera, autopilot
	// array[0] = autopilot least significant byte
	// array[1] = autopilot most significant byte
	array[11] = throttle >> 8;
	array[10] = throttle bitand 0x00ff;
	array[9] = roll >> 8;
	array[8] = roll bitand 0x00ff;
	array[7] = pitch >> 8;
	array[6] = pitch bitand 0x00ff;
	array[5] = yaw >> 8;
	array[4] = yaw bitand 0x00ff;
	array[3] = camera >> 8;
	array[2] = camera bitand 0x00ff;
	array[1] = autopilot >> 8;
	array[0] = autopilot bitand 0x00ff;
	// -------------------------------- < 4 microseconds

	// PORTB and_eq 0b11111011;
	// SPI.transfer(SPI_Commands::R_REGISTER bitor FIFO_STATUS);
	// value = SPI.transfer(NOP);
	// PORTB or_eq 0b00000100;

	// w_tx_payload with the processed data
	PORTB and_eq 0b11111011;
	SPI.transfer(SPI_Commands::W_TX_PAYLOAD);
	for (unsigned i = 0; i < PAYLOAD_LENGTH; ++i)
	{
		SPI.transfer(array[i]);
	}
	// digitalWrite(CSN, HIGH);
	PORTB or_eq 0b00000100;
	// -------------------------------- 36 microseconds

	// timer_f = micros();

	// Serial.println(timer_f-timer_i);

	// PORTD or_eq ((value bitand 0x10) << 0); // checking if tx fifo is empty before filling it
}