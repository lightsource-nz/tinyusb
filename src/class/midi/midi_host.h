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
bool tuh_midi_mounted(uint8_t index);

uint8_t tuh_midi_get_in_endpoints(uint8_t index);
uint8_t tuh_midi_get_out_endpoints(uint8_t index);
bool tuh_midi_in_endpoint_ready(uint8_t index, uint8_t ep_idx);
bool tuh_midi_out_endpoint_ready(uint8_t index, uint8_t ep_idx);

// Check which version of the USB MIDI spec this interface supports
uint8_t tuh_midi_spec_version(uint8_t index);
bool tuh_midi_spec_version_supported(uint8_t spec);

#ifdef MIDI_SPEC_1_0
uint8_t tuh_midi_v1_get_in_endpoint_jacks(uint8_t index, uint8_t ep_idx);
uint8_t tuh_midi_v1_get_out_endpoint_jacks(uint8_t index, uint8_t ep_idx);
uint8_t tuh_midi_v1_get_in_endpoint_jack_id(uint8_t index, uint8_t ep_idx, uint8_t cable_num);
uint8_t tuh_midi_v1_get_out_endpoint_jack_id(uint8_t index, uint8_t ep_idx, uint8_t cable_num);
// write multiple 32-bit MIDI event packets to the stream buffer of the given endpoint.
// returns number of packets written to the buffer.
uint16_t tuh_midi_v1_stream_n_write(uint8_t index, uint8_t ep_index, void const *data, uint16_t n);
uint16_t tuh_midi_v1_stream_write(uint8_t index, void const *data, uint16_t n);

uint16_t tuh_midi_v1_stream_n_out_available(uint8_t index, uint8_t ep_index);
uint16_t tuh_midi_v1_stream_out_available(uint8_t index);
void tuh_midi_v1_stream_out_n_flush(uint8_t index, uint8_t ep_index);
void tuh_midi_v1_stream_out_n_flush_sync(uint8_t index, uint8_t ep_index);
void tuh_midi_v1_stream_out_flush(uint8_t index);
void tuh_midi_v1_stream_out_flush_sync(uint8_t index);

// read 32-bit MIDI event packets out of the endpoint's stream buffer.
// returns number of packets read out.   
uint16_t tuh_midi_v1_stream_n_read(uint8_t index, uint8_t ep_index, void *data, uint16_t n);
uint16_t tuh_midi_v1_stream_read(uint8_t index, void *data, uint16_t n);

uint16_t tuh_midi_v1_stream_n_in_available(uint8_t index, uint8_t ep_index);
uint16_t tuh_midi_v1_stream_in_available(uint8_t index);
#endif 

#ifdef MIDI_SPEC_2_0
// spec-v2 API functions
#endif

//------------- Application Callback -------------//

// Invoked when a device with MidiStreaming interface is mounted
TU_ATTR_WEAK void tuh_midi_mount_cb(uint8_t index);

// Invoked when a device with MidiStreaming interface is unmounted
TU_ATTR_WEAK void tuh_midi_umount_cb(uint8_t index);

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
