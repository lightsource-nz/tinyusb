/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Alex Fulton (lightsource.nz)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _TUSB_MIDI_HOST_H_
#define _TUSB_MIDI_HOST_H_

#include "class/audio/audio.h"
#include "midi.h"

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Class Driver Configuration
//--------------------------------------------------------------------+

// by default, support both existing versions of the USB MIDI device class spec.
// disabling unused spec versions saves RAM
#ifndef CFG_TUH_MIDI_SPEC_1_0
#define CFG_TUH_MIDI_SPEC_1_0 1
#endif
#ifndef CFG_TUH_MIDI_SPEC_2_0
#define CFG_TUH_MIDI_SPEC_2_0 1
#endif
#if(CFG_TUH_MIDI_SPEC_1_0)
#define MIDI_SPEC_1_0
#endif
#if(CFG_TUH_MIDI_SPEC_2_0)
#define MIDI_SPEC_2_0
#endif
#if(!(CFG_TUH_MIDI_SPEC_1_0 || CFG_TUH_MIDI_SPEC_2_0))
#error "at least one version of the USB MIDI spec must be enabled"
#endif

enum {
  USB_MIDI_SPEC_VERSION_1 = 1,
  USB_MIDI_SPEC_VERSION_2 = 2
};

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Check if device supports MidiStreaming interface.
// This function true after tuh_midi_mounted_cb() and false after tuh_midi_unmounted_cb()
bool tuh_midi_mounted(uint8_t dev_addr, uint8_t itf_num);

// Check which version of the USB MIDI spec this interface supports
uint8_t tuh_midi_spec_version(uint8_t dev_addr, uint8_t itf_num);

bool tuh_midi_spec_version_supported(uint8_t spec);

#ifdef MIDI_SPEC_1_0
// Get number of virtual midi cables
uint8_t tuh_midi_get_in_cables(uint8_t dev_addr, uint8_t itf_num);
uint8_t tuh_midi_get_out_cables(uint8_t dev_addr, uint8_t itf_num);
// select a cable by zero-based array index
bool tuh_midi_in_ready(uint8_t dev_addr, uint8_t itf_num, uint8_t cable_id);
bool tuh_midi_out_ready(uint8_t dev_addr, uint8_t itf_num, uint8_t cable_id);
uint8_t tuh_midi_get_in_cable_id(uint8_t dev_addr, uint8_t itf_num, uint8_t index);
uint8_t tuh_midi_get_out_cable_id(uint8_t dev_addr, uint8_t itf_num, uint8_t index);
uint8_t tuh_midi_get_in_cable_endpoint(uint8_t dev_addr, uint8_t itf_num, uint8_t index);
uint8_t tuh_midi_get_out_cable_endpoint(uint8_t dev_addr, uint8_t itf_num, uint8_t index);
#endif

#ifdef MIDI_SPEC_2_0

typedef struct
{
  unsigned endpoint_id:8;
  unsigned block_id:8;
  unsigned block_type:2;
} m20_term_block_t;

#endif

// Send a 32-bit MIDI event packet to specified output jack and interface
// return true if success, false if there is already pending operation.
bool tuh_midi_packet_write(uint8_t dev_addr, uint8_t itf, uint8_t ep, uint8_t *event);

// Read a 32-bit MIDI event packet into the supplied buffer
bool tuh_midi_packet_read(uint8_t dev_addr, uint8_t itf, uint8_t ep, uint8_t *buffer);

//------------- Application Callback -------------//

// Invoked when a device with MidiStreaming interface is mounted
TU_ATTR_WEAK void tuh_midi_mount_cb(uint8_t idx);

// Invoked when a device with MidiStreaming interface is unmounted
TU_ATTR_WEAK void tuh_midi_umount_cb(uint8_t idx);

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+

void midih_init       (void);
bool midih_open       (uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len);
bool midih_set_config (uint8_t dev_addr, uint8_t itf_num);
void midih_close      (uint8_t dev_addr);
bool midih_xfer_cb    (uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_MSC_HOST_H_ */
