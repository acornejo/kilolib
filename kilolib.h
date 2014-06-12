/****************************************************************************
 * Copyright (C) 2014 by Alex Cornejo
 *
 * This file is part of the Kilobot Library.
 *                                                                          
 *   Kilobot Library is free software: you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation, either
 *   version 3 of the License, or (at your option) any later version.    
 *                                                                        
 *   The Kilobot Library is distributed in the hope that it will be
 *   useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 *   of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *   Lesser General Public License for more details.                    
 *                                                                          
 *   You should have received a copy of the GNU Lesser General Public       
 *   License along with Box.  If not, see <http://www.gnu.org/licenses/>.
 ****************************************************************************/

/**
 * @file kilolib.h
 * @author Alex Cornejo
 * @date Jun 12, 2014
 * @brief Definitions of the Kilobot Library API.
 *
 * At its core this library provides a function to initialize the
 * hardware of the kilobots, and another function to setup a basic event
 * loop programming paradigm (similar to that used in the arduino
 * software).
 *
 * Functions are also provided to read the various sensors (battery
 * voltage, ambient light and temperature), and to control the
 * individual pager motors and the RGB led available at each kilobot.
 *
 * The user can register callbacks to interact with the messaging
 * subsystem. There are callbacks for the events of message reception,
 * message transmission, and notification of successful transmission. By
 * default every kilobot attempts to send message twice per second.
 * Advanced users can modify this through the kilo_tx_period variable
 * (not recommended unless you know what you are doing).
 *
 * To prevent collisions the kilobot library uses a basic exponential
 * backoff strategy with carrier sensing. There are no acknowledgement
 * packets, and as such a message is considered to be successfuly
 * transmitted when a kilobot is able transmit a message without
 * detecting any contention in the channel.
 */


#ifndef __KILOLIB_H__
#define __KILOLIB_H__

#include <stdint.h>
#include "message.h"
#include "message_crc.h"

#define RGB(r,g,b) (r&3)|(((g&3)<<2))|((b&3)<<4)
#define TICKS_PER_SEC 31

typedef struct {
    int16_t low_gain;
    int16_t high_gain;
} distance_measurement_t;

typedef void (*message_rx_t)(message_t *, distance_measurement_t *d);
typedef message_t *(*message_tx_t)(void);
typedef void (*message_tx_success_t)(void);

extern volatile uint32_t kilo_ticks;
extern volatile uint16_t kilo_tx_period;
extern uint16_t kilo_uid;
extern uint8_t kilo_turn_left;
extern uint8_t kilo_turn_right;
extern uint8_t kilo_straight_left;
extern uint8_t kilo_straight_right;
extern message_rx_t kilo_message_rx;
extern message_tx_t kilo_message_tx;
extern message_tx_success_t kilo_message_tx_success;

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

int16_t get_voltage();
int16_t get_temperature();
int16_t get_ambientlight();
void set_motors(uint8_t, uint8_t);
void set_color(uint8_t);
void delay(uint16_t);
void kilo_init();
void kilo_start(void (*setup)(void), void (*program)(void));

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#endif//__KILOLIB_H__
