/*
 * rfm69hcw.c
 * v0.1
 * Author: Flox
 * https://github.com/floxo/rfm69.git
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See LICENSE for more details.
 *
 */


#include <stdint.h>
#include <util/delay.h>
#include <stdlib.h>

#include "main.h"
//#include "mavlink/mylink/mavlink.h"
//extern mavlink_system_t mavlink_system;


#include "rfm69hcw_registers.h"
#include "rfm69hcw.h"


#define MODE_SLEEP	0x00
#define MODE_STDBY	0x04
#define MODE_FS		0x08
#define MODE_TX		0x0C
#define MODE_RX		0x10

extern char* outstr;


void rfm69hcw_init(rfm69hcw_t* this, void (*process_incoming_byte)(uint8_t incoming_byte, uint8_t recv_state), volatile uint8_t* spi_data, volatile uint8_t* spi_port, uint8_t spi_pin_ss, volatile uint8_t* spi_spif, uint8_t spi_spif_mask, uint8_t high_power) {
	this->spi_data = spi_data;
	this->spi_port = spi_port;
	this->spi_pin_ss = spi_pin_ss;
	this->spi_spif = spi_spif;
	this->spi_spif_mask = spi_spif_mask;
	this->message_received = 0;
	this->high_power = high_power;
	this->process_incoming_byte = process_incoming_byte;
}

void rfm69hcw_set_spi_data(rfm69hcw_t* this, volatile uint8_t* spi_data) {
	this->spi_data = spi_data;
}

void rfm69hcw_set_spi_port(rfm69hcw_t* this, volatile uint8_t* spi_port) {
	this->spi_port = spi_port;
}

void rfm69hcw_set_spi_pin_ss(rfm69hcw_t* this, uint8_t spi_pin_ss) {
	this->spi_pin_ss = spi_pin_ss;
}

void rfm69hcw_set_spi_spif(rfm69hcw_t* this, volatile uint8_t* spi_spif) {
	this->spi_spif = spi_spif;
}

void rfm69hcw_set_spi_spif_mask(rfm69hcw_t* this, uint8_t spi_spif_mask) {
	this->spi_spif_mask = spi_spif_mask;
}

uint8_t rfm69hcw_setreg(rfm69hcw_t* this, uint8_t address, uint8_t data) {

	SPSR &= ~_BV(SPIF);

	*(this->spi_port) &= ~_BV(this->spi_pin_ss);
	*(this->spi_data) = address | 0x80; // set write operation for rfm69hcw

	while(!(SPSR & _BV(SPIF)));
	SPSR &= ~_BV(SPIF);

	*(this->spi_data) = data;

	while(!(SPSR & _BV(SPIF)));
	SPSR &= ~_BV(SPIF);

	*(this->spi_port) |= _BV(this->spi_pin_ss);

	return *(this->spi_data);
}

uint8_t rfm69hcw_getreg(rfm69hcw_t* this, uint8_t address) {

	SPSR &= ~_BV(SPIF);

	*(this->spi_port) &= ~_BV(this->spi_pin_ss);
	*(this->spi_data) = address & ~0x80; // set read operation for rfm69hcw

	while(!(SPSR & (1<<SPIF)));
	SPSR &= ~_BV(SPIF);

	*(this->spi_data) = 0x00;

	while(!(SPSR & (1<<SPIF)));
	SPSR &= ~_BV(SPIF);

	*(this->spi_port) |= _BV(this->spi_pin_ss);

	return (*(this->spi_data));
}

void rfm69hcw_write_fifo(rfm69hcw_t* this, uint8_t* data, uint8_t length) {

	SPSR &= ~_BV(SPIF);

	*(this->spi_port) &= ~_BV(this->spi_pin_ss);
	*(this->spi_data) = REG_FIFO | 0x80; // write to fifo address

	// write out data bytes
	for(int i=0; i<length; i++) {
		while(!(SPSR & _BV(SPIF)));
		SPSR &= ~_BV(SPIF);
		*(this->spi_data) = data[i];
	}

	while(!(SPSR & _BV(SPIF)));
	SPSR &= ~_BV(SPIF);

	*(this->spi_port) |= _BV(this->spi_pin_ss);
}

void rfm69hcw_set_high_power(rfm69hcw_t* this, uint8_t high_power) {
    if(high_power) this->high_power=1;
    else this->high_power=0;
}

uint8_t rfm69hcw_send(rfm69hcw_t* this, uint8_t* data, uint8_t length) {

	rfm69hcw_setmode(this, MODE_RX);

	if(!rfm69hcw_sendok(this)) {
        return 0;
	}
	else {

        if(this->high_power == 1) {
            rfm69hcw_setreg(this, REG_TESTPA1, 0x5D);
            rfm69hcw_setreg(this, REG_TESTPA2, 0x7C);
            rfm69hcw_setreg(this, REG_OCP, 0x0F);
            rfm69hcw_setreg(this, REG_PALEVEL, 0x7F);  // maximum power, PA1 and PA2 active, PA0 off
        }

        rfm69hcw_setmode(this, MODE_TX);

        rfm69hcw_write_fifo(this, data, length);

        while( !(rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x08 ) );


        if(this->high_power == 1) {	// disable high power mode
            rfm69hcw_setreg(this, REG_OCP, 0x16);
            rfm69hcw_setreg(this, REG_TESTPA1, 0x5d);
            rfm69hcw_setreg(this, REG_TESTPA2, 0x70);
            rfm69hcw_setreg(this, REG_PALEVEL, 0x9F);  // maximum power, PA0 on, PA1 and PA2 off
        }

        rfm69hcw_setmode(this, MODE_STDBY);

        return 1;
	}
}


uint8_t rfm69hcw_setmode(rfm69hcw_t* this, uint8_t mode) {
	rfm69hcw_setreg(this, REG_OPMODE, (rfm69hcw_getreg(this, REG_OPMODE) & 0xE3) | mode);
	while(!(rfm69hcw_getreg(this, REG_IRQFLAGS1) & 0x80));

	if(mode == MODE_RX) {
		//rfm69hcw_setreg(this, REG_PALEVEL, rfm69hcw_getreg(this, REG_PALEVEL) & 0x1F ); // Turn off TX amps
		rfm69hcw_setreg(this, REG_PACKETCONFIG2, rfm69hcw_getreg(this, REG_PACKETCONFIG2) | 0x04 ); // force RX WAIT
	}

	this->mode=mode;

	return 1;	// TODO - timeout and return status
}


uint8_t rfm69hcw_sendok(rfm69hcw_t* this) {
	if(this->mode == MODE_RX && rfm69hcw_readrssi(this) < CSMA_LIMIT ) {
		rfm69hcw_setmode(this, MODE_STDBY);
		return 1;
	}
	else return 0;
}

int16_t rfm69hcw_readrssi(rfm69hcw_t* this) {

	rfm69hcw_setreg(this, REG_RSSICONFIG, 0x01); // start rssi measurement
	_delay_ms(100);
	//while(!(rfm69hcw_getreg(this, REG_RSSICONFIG) & 0x02 )); // wait for finish
	return -1*rfm69hcw_getreg(this, REG_RSSIVALUE)/2;
}

/*
uint8_t rfm69hcw_mavlink_packet_data(rfm69hcw_t* this, uint8_t channel, mavlink_message_t* msg, mavlink_status_t* status) {
	uint8_t mav_msg_received;
	mav_msg_received=0;
	switch( rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x06 ) {
		case 0x06:
		case 0x02:
			// CRC ok
			while( rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x40 ) { // empty fifo
				mav_msg_received = mavlink_parse_char(channel, rfm69hcw_getreg(this, REG_FIFO), msg, status);

				if (mav_msg_received) {
					this->message_received=1;
					while( rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x40 ) rfm69hcw_getreg(this, REG_FIFO); // make sure fifo is empty
					break;
				}
			}
			break;
		default:
			while( rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x40 ) rfm69hcw_getreg(this, REG_FIFO); // CRC failed. Make sure fifo is empty
			mavlink_reset_channel_status(channel); // Reset mavlink	channel status to idle
			break;
	}

	return (mav_msg_received);
}
*/

uint8_t rfm69hcw_process_packet_data(rfm69hcw_t* this, uint8_t recv_complete ) {

	if(recv_complete) {
        if(!rfm69hcw_crc_ok(this)) {
            (this->process_incoming_byte)(0, 0);
        }
        else {
            while( PIN_PAYLOAD_READY ) { // empty fifo
                (this->process_incoming_byte)(rfm69hcw_getreg(this, REG_FIFO), 1);
            }
            (this->process_incoming_byte)(0, 2);
            this->message_received=1;
        }
	}
	else {
        while( rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x40 ) { // empty fifo   TODO: replace getreg with DIO line
            (this->process_incoming_byte)(rfm69hcw_getreg(this, REG_FIFO), 1);
        }
	}

	return (this->message_received);
}

uint8_t rfm69hcw_crc_ok(rfm69hcw_t* this) {

    return (rfm69hcw_getreg(this, REG_IRQFLAGS2) & 0x02);

}
