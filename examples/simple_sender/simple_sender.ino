// RFM70 simple_sender
// by Heye Everts <heye.everts.1@gmail.com>
//
// Demonstrates use of the rfm70 library
// Send data and print received with serail
// 
// Refer to the "echo" example for use with this
//
// Created 2 January 2013
// Copyright 2013 Heye Everts

#include <rfm70.h>
#include <SPI.h>

// CS, CE, IRQ, CLKDIV
RFM70 rfm70(10, 8, 2, RFM77_DEFAULT_SPI_CLOCK_DIV);

void setup() {
	//Serial connection for debugging
	Serial.begin(19200);
	Serial.println("##reset##");

	rfm70.begin();
	rfm70.onReceive(receiveEvent);
}

byte array[32] = "1234567890123456789012345678901";

void loop() {
	//send some text with length, the max length is 32 bytes!
	rfm70.send((byte*) "hello!", 6);
	delay(100);

	//the tick function checks for any received data and calls the receiveEvent
	rfm70.tick();

	//another way to send something
	rfm70.send(array, 32);
	delay(100);

	//the tick function checks for any received data and calls the receiveEvent
	rfm70.tick();
}

void receiveEvent(void) {
	//print received data
	Serial.println((char*) rfm70.getRcvBuffer());
}

