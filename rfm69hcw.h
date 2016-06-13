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

#ifndef RFM69HCW_H_
#define RFM69HCW_H_

// environment
#define SPIDATA         SPDR

#define MODE_TIMEOUT    100
#define CSMA_LIMIT      -70


#define MODE_SLEEP      0x00
#define MODE_STDBY      0x04
#define MODE_FS         0x08
#define MODE_TX         0x0C
#define MODE_RX         0x10




typedef struct rfm69hcw_t {
    volatile uint8_t* spi_port;
    uint8_t spi_pin_ss;
    volatile uint8_t* spi_data;
    volatile uint8_t* spi_spif;
    uint8_t spi_spif_mask;
    uint8_t mode;
    uint8_t message_received;
    uint8_t high_power;
    void (*process_incoming_byte)(uint8_t incoming_byte, uint8_t recv_state);
} rfm69hcw_t;



void     rfm69hcw_init(rfm69hcw_t* this, void (*process_incoming_byte)(uint8_t incoming_byte, uint8_t recv_state), volatile uint8_t* spi_data, volatile uint8_t* spi_port, uint8_t spi_pin_ss, volatile uint8_t* spi_spif, uint8_t spi_spif_mask, uint8_t high_power);
void     rfm69hcw_set_spi_data(rfm69hcw_t* this, volatile uint8_t* spi_data);
void     rfm69hcw_set_spi_port(rfm69hcw_t* this, volatile uint8_t* spi_port);
void     rfm69hcw_set_spi_pin_ss(rfm69hcw_t* this, uint8_t spi_pin_ss);
void     rfm69hcw_set_spi_spif(rfm69hcw_t* this, volatile uint8_t* spi_spif);
void     rfm69hcw_set_spi_spif_mask(rfm69hcw_t* this, uint8_t spi_spif_mask);
uint8_t  rfm69hcw_setreg(rfm69hcw_t* this, uint8_t address, uint8_t data);
uint8_t  rfm69hcw_getreg(rfm69hcw_t* this, uint8_t address);
void     rfm69hcw_write_fifo(rfm69hcw_t* this, uint8_t* data, uint8_t length);
uint8_t  rfm69hcw_setmode(rfm69hcw_t* this, uint8_t mode);
uint8_t  rfm69hcw_sendok(rfm69hcw_t* this);
int16_t  rfm69hcw_readrssi(rfm69hcw_t* this);
uint8_t  rfm69hcw_send(rfm69hcw_t* this, uint8_t* data, uint8_t length);
uint8_t  rfm69hcw_crc_ok(rfm69hcw_t* this);
uint8_t  rfm69hcw_mavlink_packet_data(rfm69hcw_t* this, uint8_t channel, mavlink_message_t* msg, mavlink_status_t* status);  // for IRQ handlers. read packet data. return 1 if payload ready and crc ok, 0 else
uint8_t  rfm69hcw_process_packet_data(rfm69hcw_t* this, uint8_t recv_state );
void     rfm69hcw_set_high_power(rfm69hcw_t* this, uint8_t high_power);





#endif /* RFM69HCW_H_ */
