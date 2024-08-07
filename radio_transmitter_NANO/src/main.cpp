#include <Arduino.h>
#include <SPI.h>

// global variables
constexpr uint8_t CSN = 10, CE = 9, AUTOPILOT_INPUT = 3, ON_INPUT = 2; // this way pins 9 through 13 are used for SPI communication with the radio transmitter
constexpr uint8_t PAYLOAD_LENGTH = 12;
int16_t throttle, roll, pitch, yaw, camera, autopilot = 0, ON;
uint8_t transmitArray[PAYLOAD_LENGTH];

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


// using time consuming function calls is okay in setup
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

	pinMode(AUTOPILOT_INPUT, INPUT);
	pinMode(ON_INPUT, INPUT);

	// set the SPI settings
	SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

	// declare as transmitter
	write_to_register(CONFIG, 0b00001110); // enable CRC, 2 byte CRC length, power up, set as transmitter

	// disable enhanced shockburst
	write_to_register(EN_AA, 0x00);

	// setup 5 byte address width
	write_to_register(SETUP_AW, 0x03);

	// frequency channel setup
	write_to_register(RF_CH, 0b01111100); // 2524 MHz channel

	// data rate
	write_to_register(RF_SETUP, 0x00);

	// payload length setup
	write_to_register(RX_PW_P0, PAYLOAD_LENGTH); // 12 byte payload length

	// give the device a unique address 0xD6D6D6D6D6
	digitalWrite(CSN, LOW);
	SPI.transfer(W_REGISTER bitor TX_ADDR);
	for (unsigned i = 0; i < 5; ++i)
	{
		SPI.transfer(0xD6);
	}
	digitalWrite(CSN, HIGH);

	// flush the tx fifo
	command_SPI(FLUSH_TX);

	// CE high
	digitalWrite(CE, HIGH);
}

// loop needs to be as fast as possible, so no unnecessary function calls
void loop()
{
	// timer_i = micros();

	// resuse last transmitted data packet until I do the calculations
	PORTB and_eq 0b11111011; //digitalWrite(CSN, LOW);
	SPI.transfer(SPI_Commands::REUSE_TX_PL);
	PORTB or_eq 0b00000100;//digitalWrite(CSN, HIGH);
	// --------------------------------------- 4 microseconds

	// read analog values
	throttle = analogRead(A0);
	roll = analogRead(A1);
	pitch = analogRead(A2);
	yaw = analogRead(A3);
	camera = analogRead(A4);
	autopilot = (PIND bitand 0b00001000); // digitalRead(3)
	ON = (PIND bitand 0b00000100);		  // digitalRead(2);
	// -------------------------------------- 564 microseconds

	// map the values into pulsewidth,
	throttle = throttle * 0.977517106549365 + 1000;
	roll = roll * 1.95503421309873 - 1000;
	pitch = pitch * 1.95503421309873 - 1000;
	yaw = yaw * 1.95503421309873 - 1000;
	camera = camera * 1.95503421309873 - 1000; // 1500 us pulse is the centre position of a servo.
	// -------------------------------------- 120 microseconds

	// process the data into 12 byte transmitArray
	// throttle, roll, pitch, yaw, camera, autopilot
	// transmitArray[0] = ON
	// transmitArray[1] = autopilot
	// transmitArray[2] = camera least significant byte
	// transmitArray[3] = camera most significant byte
	transmitArray[11] = throttle >> 8;
	transmitArray[10] = throttle bitand 0x00ff;
	transmitArray[9] = roll >> 8;
	transmitArray[8] = roll bitand 0x00ff;
	transmitArray[7] = pitch >> 8;
	transmitArray[6] = pitch bitand 0x00ff;
	transmitArray[5] = yaw >> 8;
	transmitArray[4] = yaw bitand 0x00ff;
	transmitArray[3] = camera >> 8;
	transmitArray[2] = camera bitand 0x00ff;
	transmitArray[1] = autopilot;
	transmitArray[0] = ON;
	// -------------------------------- < 4 microseconds

	// DEBUG CODE
	// PORTB and_eq 0b11111011;
	// SPI.transfer(SPI_Commands::R_REGISTER bitor FIFO_STATUS);
	// byte value = SPI.transfer(NOP);
	// Serial.println(value, HEX);
	// PORTB or_eq 0b00000100;

	// w_tx_payload with the processed data
	PORTB and_eq 0b11111011;  //digitalWrite(CSN, LOW);
	SPI.transfer(SPI_Commands::W_TX_PAYLOAD);
	for (unsigned i = 0; i < PAYLOAD_LENGTH; ++i)
	{
		SPI.transfer(transmitArray[i]);
		// Serial.print(transmitArray[i]);
		// Serial.print(' ');
	}
	PORTB or_eq 0b00000100;  //digitalWrite(CSN, HIGH);
	// Serial.println();
	// -------------------------------- 36 microseconds

	// timer_f = micros();

	// Serial.println(timer_f-timer_i);

	// PORTD or_eq ((value bitand 0x10) << 0); // checking if tx fifo is empty before filling it
}