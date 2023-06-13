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

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUH_MIDI

#include "host/usbh.h"
#include "host/usbh_classdriver.h"

#include "midi_host.h"

// Debug level, TUSB_CFG_DEBUG must be at least this level for debug message
#define MIDIH_DEBUG   2
#define TU_LOG_MIDIH(...)   TU_LOG(MIDIH_DEBUG, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

#ifdef MIDI_SPEC_1_0
//    VCable is a simplified representation of the underlying USB-MIDI
// function descriptors. VCable objects have a 1:1 relationship to
// "embedded MIDI Jacks" in the MIDI Streaming Class 1.0 descriptors,
// but their nominal direction (in or out) is considered the same as
// the direction of the endpoint they are attached to.
//    VCables are stored as an 8-bit jack ID which corresponds to the
// cable number (CN) of messages for this cable in the 32-bit USB MIDI
// event packet, and the 8-bit endpoint address of the MIDI streaming
// endpoint which they are bound to.
typedef struct
{
  uint8_t jack_id;
  uint8_t endpoint_id;
} midi_vcable_t;

typedef struct
{
  uint8_t cable_in_count;
  uint8_t cable_out_count;
  midi_vcable_t cable_in[CFG_TUH_MIDI_CABLES];
  midi_vcable_t cable_out[CFG_TUH_MIDI_CABLES];
} midih_io_m10_t;
/*
static inline uint8_t v1_cable_id(midi_vcable_t *cable)
{
  return cable->jack_id;
}
static inline uint8_t v1_cable_endpoint(midi_vcable_t *cable)
{
  return cable->endpoint_id;
}
static inline uint8_t v1_cable_dir(midi_vcable_t *cable)
{
  return tu_edpt_dir(cable->endpoint_id);
}
*/
#endif

#ifdef MIDI_SPEC_2_0
// represents the virtual I/O constructs of MIDI Streaming Class v2.0
typedef struct
{
  m20_term_block_t term_block[CFG_TUH_MIDI_V2_TERMINAL_BLOCKS];
} midih_io_m20_t;

#endif

typedef union {
#ifdef MIDI_SPEC_1_0
  midih_io_m10_t v1;
#endif
#ifdef MIDI_SPEC_2_0
  midih_io_m20_t v2;
#endif
} midih_io_t;

typedef struct
{
  uint8_t dev_num;
  uint8_t itf_num;
  uint8_t midi_spec;

  uint8_t ep_in[CFG_TUH_ENDPOINT_MAX];
  uint8_t ep_in_c;
  uint8_t ep_out[CFG_TUH_ENDPOINT_MAX];
  uint8_t ep_out_c;

  volatile bool configured;
  volatile bool mounted;

  // contains version-specific I/O fields
  midih_io_t io;
} midih_interface_t;

bool tuh_midi_spec_version_supported(uint8_t spec)
{
  switch (spec)
  {
#ifdef MIDI_SPEC_1_0
  case USB_MIDI_SPEC_VERSION_1:
    return true;
#endif

#ifdef MIDI_SPEC_2_0
  case USB_MIDI_SPEC_VERSION_2:
    return true;
#endif

  default:
    return false;
  }
}

CFG_TUH_MEM_SECTION static midih_interface_t _midih_itf[CFG_TUH_MIDI];

// buffer used to read midi information when mounted
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN
static uint8_t _midih_buffer[64];

// FIXME potential nul reference
TU_ATTR_ALWAYS_INLINE
static inline midih_interface_t* get_itf(uint8_t index)
{
  return &_midih_itf[index - 1];
}

static midih_interface_t* make_new_itf(uint8_t dev_addr, uint8_t itf_num, uint8_t spec_version)
{
  TU_VERIFY(tuh_midi_spec_version_supported(spec_version));

  midih_interface_t *itf;
  for(uint8_t i = 0; i < CFG_TUH_MIDI; i++) {
    itf = &_midih_itf[i];
    if(itf->dev_num == 0) {
      itf->dev_num = dev_addr;
      itf->itf_num = itf_num;
      return itf;
    }
  }
  return NULL;
}

static uint8_t get_index(uint8_t dev_addr, uint8_t itf_num)
{
  for(uint8_t i = 0; i < CFG_TUH_MIDI; i++) {
    if(_midih_itf[i].dev_num == dev_addr && _midih_itf[i].itf_num == itf_num) {
      return i + 1;
    }
  }
  return 0;
}
static uint8_t get_index_by_endpoint(uint8_t dev_addr, uint8_t endpoint_addr)
{
  for(uint8_t i = 0; i < CFG_TUH_MIDI; i++) {
    midih_interface_t *itf = &_midih_itf[i];
    if(itf->dev_num == dev_addr) {
      uint8_t (*eps)[CFG_TUH_ENDPOINT_MAX];
      if(tu_edpt_dir(endpoint_addr) == TUSB_DIR_IN)
        eps = &itf->ep_in;
      else
        eps = &itf->ep_out;
      for(uint8_t j = 0; j < CFG_TUH_ENDPOINT_MAX; j++) {
        if((*eps)[j] == endpoint_addr)
          return i + 1;
      }
    }
  }
  return 0;
}

//--------------------------------------------------------------------+
// PUBLIC API
//--------------------------------------------------------------------+

bool tuh_midi_packet_write(uint8_t dev_addr, uint8_t itf, uint8_t ep, uint8_t *event)
{
  usbh_edpt_claim(dev_addr, ep);
  return usbh_edpt_xfer_with_callback(dev_addr, ep, event, 4, midih_xfer_cb, 0);
}

bool tuh_midi_packet_read(uint8_t dev_addr, uint8_t itf, uint8_t ep, uint8_t *buffer)
{
  usbh_edpt_claim(dev_addr, ep);
  return usbh_edpt_xfer_with_callback(dev_addr, ep, buffer, 4, midih_xfer_cb, 0);
}

bool tuh_midi_mounted(uint8_t dev_addr, uint8_t itf_num)
{
  midih_interface_t* p_midi = get_itf(get_index(dev_addr, itf_num));
  return p_midi->mounted;
}

uint8_t tuh_midi_spec_version(uint8_t dev_addr, uint8_t itf_num)
{
  return get_itf(get_index(dev_addr, itf_num))->midi_spec;
}

#ifdef MIDI_SPEC_1_0
static midi_vcable_t *get_in_cable(uint8_t dev_addr, uint8_t itf_num, uint8_t index)
{
  midih_interface_t *itf = get_itf(get_index(dev_addr, itf_num));
  TU_VERIFY(index < itf->io.v1.cable_in_count);
  return &itf->io.v1.cable_in[index];
}
static midi_vcable_t *get_out_cable(uint8_t dev_addr, uint8_t itf_num, uint8_t index)
{
  midih_interface_t *itf = get_itf(get_index(dev_addr, itf_num));
  TU_VERIFY(index < itf->io.v1.cable_out_count);
  return &itf->io.v1.cable_out[index];
}
uint8_t tuh_midi_get_in_cables(uint8_t dev_addr, uint8_t itf_num)
{
  TU_VERIFY(tuh_midi_spec_version(dev_addr, itf_num) == USB_MIDI_SPEC_VERSION_1);

  return get_itf(get_index(dev_addr, itf_num))->io.v1.cable_in_count;
}
uint8_t tuh_midi_get_out_cables(uint8_t dev_addr, uint8_t itf_num)
{
  TU_VERIFY(tuh_midi_spec_version(dev_addr, itf_num) == USB_MIDI_SPEC_VERSION_1);

  return get_itf(get_index(dev_addr, itf_num))->io.v1.cable_out_count;
}
bool tuh_midi_in_ready(uint8_t dev_addr, uint8_t itf_num, uint8_t cable_idx)
{
  midih_interface_t* p_midi = get_itf(get_index(dev_addr, itf_num));
  return p_midi->mounted && !usbh_edpt_busy(dev_addr, p_midi->io.v1.cable_in[cable_idx].endpoint_id);
}
bool tuh_midi_out_ready(uint8_t dev_addr, uint8_t itf_num, uint8_t cable_idx)
{
  midih_interface_t* p_midi = get_itf(get_index(dev_addr, itf_num));
  return p_midi->mounted && !usbh_edpt_busy(dev_addr, p_midi->io.v1.cable_out[cable_idx].endpoint_id);
}
uint8_t tuh_midi_get_in_cable_id(uint8_t dev_addr, uint8_t itf_num, uint8_t index)
{
  return get_in_cable(dev_addr, itf_num, index)->jack_id;
}
uint8_t tuh_midi_get_out_cable_id(uint8_t dev_addr, uint8_t itf_num, uint8_t index)
{
  return get_out_cable(dev_addr, itf_num, index)->jack_id;
}
uint8_t tuh_midi_get_in_cable_endpoint(uint8_t dev_addr, uint8_t itf_num, uint8_t index)
{
  return get_in_cable(dev_addr, itf_num, index)->endpoint_id;
}
uint8_t tuh_midi_get_out_cable_endpoint(uint8_t dev_addr, uint8_t itf_num, uint8_t index)
{
  return get_out_cable(dev_addr, itf_num, index)->endpoint_id;
}
#endif

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+
void midih_init(void)
{
  tu_memclr(_midih_itf, sizeof(_midih_itf));
}

void midih_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );

  midih_interface_t* p_midi;
  for(uint8_t i = 0; i < CFG_TUH_DEVICE_ITF_MAX; i++) {
    if(p_midi = get_itf(get_index(dev_addr, i))) {
      TU_LOG_MIDIH("  MIDIh close addr = %d\r\n", dev_addr);

      if (p_midi->mounted) {
        if(tuh_midi_umount_cb) tuh_midi_umount_cb(dev_addr);
      }
      tu_memclr(p_midi, sizeof(midih_interface_t));
    }
  }
}

bool midih_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  midih_interface_t* p_midi = get_itf(get_index_by_endpoint(dev_addr, ep_addr));

  if(event == XFER_RESULT_SUCCESS) {
    if(tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
      
    }
  }

  return true;
}

//--------------------------------------------------------------------+
// MIDI Enumeration
//--------------------------------------------------------------------+

#ifdef MIDI_SPEC_1_0
static bool config_process_spec_v1_interface(midih_interface_t *itf, tusb_desc_interface_t const *desc_itf, uint16_t max_len);
#endif
#ifdef MIDI_SPEC_2_0
static bool config_process_spec_v2_interface(midih_interface_t *itf, tusb_desc_interface_t const *desc_itf, uint16_t max_len);
#endif

bool midih_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  (void) rhport;
  // first interface is either audio-control v1
  // OR midi-streaming v2
  TU_VERIFY (TUSB_CLASS_AUDIO == desc_itf->bInterfaceClass);
  if(AUDIO_SUBCLASS_CONTROL == desc_itf->bInterfaceSubClass) {
    desc_itf =(tusb_desc_interface_t const *) tu_desc_find2(desc_itf, (const uint8_t *)(desc_itf + max_len),
                              TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_MIDI_STREAMING);
  }

  TU_VERIFY (TUSB_CLASS_AUDIO == desc_itf->bInterfaceClass);
  TU_VERIFY (AUDIO_SUBCLASS_MIDI_STREAMING == desc_itf->bInterfaceSubClass);

  midi_desc_header_t *header = (midi_desc_header_t *) tu_desc_next(desc_itf);
  uint8_t midi_spec;
  if(header->bcdMSC == 0x0100) midi_spec = USB_MIDI_SPEC_VERSION_1;
  else if(header->bcdMSC == 0x0200) midi_spec = USB_MIDI_SPEC_VERSION_2;
  else return false; // unknown spec version

  if(!tuh_midi_spec_version_supported(midi_spec)) return false; // unsupported spec version

  midih_interface_t* p_midi = make_new_itf(dev_addr, desc_itf->bInterfaceNumber, midi_spec);

#ifdef MIDI_SPEC_1_0
  if(midi_spec == USB_MIDI_SPEC_VERSION_1)
    return config_process_spec_v1_interface(p_midi, desc_itf, max_len);
#endif
#ifdef MIDI_SPEC_2_0
  if(midi_spec == USB_MIDI_SPEC_VERSION_2)
    return config_process_spec_v2_interface(p_midi, desc_itf, max_len);
#endif

  return false;
}

#ifdef MIDI_SPEC_1_0
static bool config_process_spec_v1_interface(midih_interface_t *itf, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  uint8_t const *desc = tu_desc_next(desc_itf); // desc = CS interface: header
  midi_desc_header_t *header = (midi_desc_header_t *) desc;
  uint8_t const *end = desc + header->wTotalLength;
  desc = tu_desc_next(desc);
  
  /* skip element and jack descriptors for now
  while(desc < end) {
    TU_VERIFY(tu_desc_type(desc) == TUSB_DESC_CS_INTERFACE);
    switch(desc[2]) {
    case MIDI_CS_INTERFACE_ELEMENT:
      break;
    case MIDI_CS_INTERFACE_IN_JACK:
      midi_desc_in_jack_t *jack_in_desc = (midi_desc_in_jack_t *)desc;
      if(iob->jack_count < CFG_TUH_MIDI_JACKS) {
        midih_v10_jack_t *jack = &iob->jack[iob->jack_count++];
        jack->jack_id = jack_in_desc->bJackID;
        jack->direction = TUSB_DIR_IN;
        jack->embedded = (jack_in_desc->bJackType == MIDI_JACK_EMBEDDED);
      }
      break;
    case MIDI_CS_INTERFACE_OUT_JACK:
      midi_desc_out_jack_t *jack_out_desc = (midi_desc_out_jack_t *)desc;
      if(iob->jack_count < CFG_TUH_MIDI_JACKS) {
        midih_v10_jack_t *jack = &iob->jack[iob->jack_count++];
        jack->jack_id = jack_out_desc->bJackID;
        jack->direction = TUSB_DIR_OUT;
        jack->embedded = (jack_out_desc->bJackType == MIDI_JACK_EMBEDDED);
      }
      break;
    }
    desc = tu_desc_next(desc);
  }
  */
  desc = tu_desc_find(desc, (void *) desc_itf + max_len, TUSB_DESC_ENDPOINT);
  for(uint8_t i = 0; i < desc_itf->bNumEndpoints; i++) {
    TU_VERIFY(tu_desc_type(desc) == TUSB_DESC_ENDPOINT);
    const tusb_desc_endpoint_t *ep = (tusb_desc_endpoint_t *)desc;
    TU_ASSERT(tuh_edpt_open(itf->dev_num, ep));
    desc = tu_desc_next(desc);
    midi_desc_endpoint_t *ep_cs = (midi_desc_endpoint_t *)desc;
    if(tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
      if(itf->ep_in_c < CFG_TUH_ENDPOINT_MAX) {
        itf->ep_in[itf->ep_in_c++] = ep->bEndpointAddress;
        for(uint8_t j = 0; j < ep_cs->bNumEmbMIDIJack; j++) {
          if(itf->io.v1.cable_in_count < CFG_TUH_MIDI_CABLES) {
            itf->io.v1.cable_in[itf->io.v1.cable_in_count++] = (midi_vcable_t) { 
              .jack_id = ep_cs->baAssocJackID[j],
              .endpoint_id = ep->bEndpointAddress
            };
          }
        }
      }
    } else {
      if(itf->ep_out_c < CFG_TUH_ENDPOINT_MAX) {
        itf->ep_out[itf->ep_out_c++] = ep->bEndpointAddress;
        for(uint8_t j = 0; j < ep_cs->bNumEmbMIDIJack; j++) {
          if(itf->io.v1.cable_out_count < CFG_TUH_MIDI_CABLES) {
            itf->io.v1.cable_out[itf->io.v1.cable_out_count++] = (midi_vcable_t) { 
              .jack_id = ep_cs->baAssocJackID[j],
              .endpoint_id = ep->bEndpointAddress
            };
          }
        }
      }
    }
      
  }

  return true;
}
#endif

#ifdef MIDI_SPEC_2_0
static bool config_process_spec_v2_interface(midih_interface_t *itf, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  return true;
}
#endif

bool midih_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  uint8_t index = get_index(dev_addr, itf_num);
  midih_interface_t* p_midi = get_itf(index);

  tuh_midi_mount_cb(index);

  // we return itf_num + 1 because midi driver binds 2 interfaces
  usbh_driver_set_config_complete(dev_addr, itf_num + 1);

  return true;
}

#endif
