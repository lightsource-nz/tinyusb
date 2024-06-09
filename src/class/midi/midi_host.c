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
#include "tusb_private.h"

#include "midi_host.h"

// Debug level, TUSB_CFG_DEBUG must be at least this level for debug message
#define MIDIH_DEBUG   2
#define TU_LOG_MIDIH(...)   TU_LOG(MIDIH_DEBUG, __VA_ARGS__)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
#define MIDIH_EP_JACK_MAX         16
#define MIDIH_STREAM_BUFFER_SZ    64
#define MIDIH_STREAM_FIFO_SZ      128

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

typedef struct {
  uint8_t jack_count;
  uint8_t jack_id[MIDIH_EP_JACK_MAX];
  tu_edpt_stream_t *stream;
} midi_v1_ep_t;

typedef struct
{
#if (CFG_TUH_MIDI_EP_MAX > 1)
  midi_v1_ep_t ep_in[CFG_TUH_MIDI_EP_MAX];
  midi_v1_ep_t ep_out[CFG_TUH_MIDI_EP_MAX];
#else
  midi_v1_ep_t ep_in;
  midi_v1_ep_t ep_out;
#endif
} midih_io_v1_t;
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
// TODO fill out spec-v2 structures and routines
typedef struct
{

} v2_term_block_t;
typedef struct
{
  uint8_t block_count;
  v2_term_block_t block[CFG_TUH_MIDI_V2_TERMINAL_BLOCKS];
} midi_v2_ep_t;
typedef struct
{
  midi_v2_ep_t endpoint[CFG_TUH_MIDI_EP_MAX];
} midih_io_v2_t;

#endif

typedef union {
#ifdef MIDI_SPEC_1_0
  midih_io_v1_t v1;
#endif
#ifdef MIDI_SPEC_2_0
  midih_io_v2_t v2;
#endif
} midih_io_t;

typedef struct
{
  uint8_t dev_num;
  uint8_t itf_num;
  uint8_t midi_spec;

#if (CFG_TUH_MIDI_EP_MAX > 1)
  uint8_t ep_in[CFG_TUH_MIDI_EP_MAX];
  uint8_t ep_in_c;
  uint8_t ep_out[CFG_TUH_MIDI_EP_MAX];
  uint8_t ep_out_c;
#else
  uint8_t ep_in;
  uint8_t ep_out;
#endif

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

static tu_edpt_stream_t _stream_blk[CFG_TUH_ENDPOINT_MAX];
CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN
static uint8_t _stream_buffer[CFG_TUH_ENDPOINT_MAX][MIDIH_STREAM_BUFFER_SZ];
static uint8_t _stream_fifo[CFG_TUH_ENDPOINT_MAX][MIDIH_STREAM_FIFO_SZ];
static uint8_t _buffer_owner[CFG_TUH_ENDPOINT_MAX];

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
#if (CFG_TUH_MIDI_EP_MAX > 1)
      uint8_t (*eps)[CFG_TUH_MIDI_EP_MAX];
      if(tu_edpt_dir(endpoint_addr) == TUSB_DIR_IN)
        eps = &itf->ep_in;
      else
        eps = &itf->ep_out;
      for(uint8_t j = 0; j < CFG_TUH_MIDI_EP_MAX; j++) {
        if((*eps)[j] == endpoint_addr)
          return i + 1;
      }
#else
      if(itf->ep_in == endpoint_addr || itf->ep_out == endpoint_addr)
        return i + 1;
#endif
    }
  }
  return 0;
}

#ifdef MIDI_SPEC_1_0
static uint8_t alloc_stream_blk(uint8_t itf_index)
{
  // TODO this is the wrong constant to use for max stream block limit
  for(uint8_t i = 0; i < CFG_TUH_ENDPOINT_MAX; i++) {
    if(_buffer_owner[i] == 0) {
      _buffer_owner[i] = itf_index;
      return i + 1;
    }
  }
  return 0;
}
static tu_edpt_stream_t *get_stream_blk(uint8_t stream_id)
{
  return &_stream_blk[stream_id - 1];
}
static uint8_t *get_stream_buffer(uint8_t stream_id)
{
  return _stream_buffer[stream_id - 1];
}
static uint8_t *get_stream_fifo(uint8_t stream_id)
{
  return _stream_fifo[stream_id - 1];
}
static midi_v1_ep_t *get_in_ep_n_v1(uint8_t index, uint8_t ep_index)
{
  midih_interface_t *itf = get_itf(index);
#if(CFG_MIDIH_EP_MAX > 1)
  TU_VERIFY(ep_index < itf->io.v1.ep_in_count);
  return &itf->io.v1.ep_in[ep_index];
#else
  return &itf->io.v1.ep_in;
#endif
}
static inline midi_v1_ep_t *get_in_ep_v1(uint8_t index)
{
  return get_in_ep_n_v1(index, 0);
}
static midi_v1_ep_t *get_out_ep_n_v1(uint8_t index, uint8_t ep_index)
{
  midih_interface_t *itf = get_itf(index);
#if(CFG_MIDIH_EP_MAX > 1)
  TU_VERIFY(ep_index < itf->io.v1.ep_out_count);
  return &itf->io.v1.ep_out[ep_index];
#else
  return &itf->io.v1.ep_out;
#endif
}
static inline midi_v1_ep_t *get_out_ep_v1(uint8_t index)
{
  return get_out_ep_n_v1(index, 0);
}
static midi_v1_ep_t *get_ep_v1_by_addr(uint8_t index, uint8_t ep_addr)
{
  midih_interface_t *itf = get_itf(index);
#if (CFG_TUH_MIDI_EP_MAX > 1)
  for(uint8_t i = 0; i < CFG_TUH_MIDI_EP_MAX; i++) {
    if(itf->ep_in[i] == ep_addr) {
      return &itf->io.v1.ep_in[i];
    }
    if(itf->ep_out[i] == ep_addr) {
      return &itf->io.v1.ep_out[i];
    }
  }
#else
  if(itf->ep_in == ep_addr) {
    return &itf->io.v1.ep_in;
  }
  if(itf->ep_out == ep_addr) {
    return &itf->io.v1.ep_out;
  }
#endif
  return NULL;
}
#endif
//--------------------------------------------------------------------+
// PUBLIC API
//--------------------------------------------------------------------+
#ifdef MIDI_SPEC_1_0
uint16_t tuh_midi_v1_stream_n_write(uint8_t index, uint8_t ep_index, void const *data, uint16_t n)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  midih_interface_t *itf = get_itf(index);
  midi_v1_ep_t *ep = get_out_ep_n_v1(index, ep_index);
  TU_ASSERT(ep);
  return tu_edpt_stream_write(ep->stream, data, n);
}
uint16_t tuh_midi_v1_stream_write(uint8_t index, void const *data, uint16_t n)
{
  return tuh_midi_v1_stream_n_write(index, 0, data, n);
}

uint16_t tuh_midi_v1_stream_n_out_available(uint8_t index, uint8_t ep_index)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  midih_interface_t *itf = get_itf(index);
  midi_v1_ep_t *ep = get_out_ep_n_v1(index, ep_index);
  return tu_edpt_stream_write_available(ep->stream);
}
uint16_t tuh_midi_v1_stream_out_available(uint8_t index)
{
  return tuh_midi_v1_stream_n_out_available(index, 0);
}

void tuh_midi_v1_stream_out_n_flush(uint8_t index, uint8_t ep_index)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1,);
  midi_v1_ep_t *ep = get_out_ep_n_v1(index, ep_index);
  tu_edpt_stream_write_xfer(ep->stream);
}
void tuh_midi_v1_stream_out_n_flush_sync(uint8_t index, uint8_t ep_index)
{
  tuh_midi_v1_stream_out_n_flush(index, ep_index);
  while(!tuh_midi_out_endpoint_ready(index, ep_index));
}
void tuh_midi_v1_stream_out_flush(uint8_t index)
{
  tuh_midi_v1_stream_out_n_flush(index, 0);
}
void tuh_midi_v1_stream_out_flush_sync(uint8_t index)
{
  tuh_midi_v1_stream_out_n_flush_sync(index, 0);
}

uint16_t tuh_midi_v1_stream_n_read(uint8_t index, uint8_t ep_index, void *data, uint16_t n)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  midih_interface_t *itf = get_itf(index);
  midi_v1_ep_t *ep = get_in_ep_n_v1(index, ep_index);
  TU_ASSERT(ep);
  return tu_edpt_stream_read(ep->stream, data, n);
}
uint16_t tuh_midi_v1_stream_read(uint8_t index, void *data, uint16_t n)
{
  return tuh_midi_v1_stream_n_read(index, 0, data, n);
}

uint16_t tuh_midi_v1_stream_n_in_available(uint8_t index, uint8_t ep_index)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  midi_v1_ep_t *ep = get_in_ep_n_v1(index, ep_index);
  return tu_edpt_stream_write_available(ep->stream);
}
uint16_t tuh_midi_v1_stream_in_available(uint8_t index)
{
  return tuh_midi_v1_stream_n_in_available(index, 0);
}
#endif

bool tuh_midi_mounted(uint8_t index)
{
  midih_interface_t* p_midi = get_itf(index);
  return p_midi->mounted;
}

uint8_t tuh_midi_spec_version(uint8_t index)
{
  return get_itf(index)->midi_spec;
}

uint8_t tuh_midi_get_in_endpoints(uint8_t index)
{
#if (CFG_TUH_MIDI_EP_MAX > 1)
  return get_itf(index)->ep_in_count;
#else
  return get_itf(index)->ep_in? 1 : 0;
#endif
}
uint8_t tuh_midi_get_out_endpoints(uint8_t index)
{
#if(CFG_TUH_MIDI_EP_MAX > 1)
  return get_itf(index)->ep_out_count;
#else
  return get_itf(index)->ep_out? 1 : 0;
#endif
}
bool tuh_midi_in_endpoint_ready(uint8_t index, uint8_t ep_index)
{
  midih_interface_t* p_midi = get_itf(index);
#if (CFG_TUH_MIDI_EP_MAX > 1)
  return p_midi->mounted && !usbh_edpt_busy(p_midi->dev_num, p_midi->ep_in[ep_index]);
#else
  return p_midi->mounted && !usbh_edpt_busy(p_midi->dev_num, p_midi->ep_in);
#endif
}
bool tuh_midi_out_endpoint_ready(uint8_t index, uint8_t ep_index)
{
  midih_interface_t* p_midi = get_itf(index);
#if (CFG_TUH_MIDI_EP_MAX > 1)
  return p_midi->mounted && !usbh_edpt_busy(p_midi->dev_num, p_midi->ep_out[ep_index]);
#else
  return p_midi->mounted && !usbh_edpt_busy(p_midi->dev_num, p_midi->ep_out);
#endif
}
#ifdef MIDI_SPEC_1_0
uint8_t tuh_midi_v1_get_in_endpoint_jacks(uint8_t index, uint8_t ep_idx)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  return get_in_ep_n_v1(index, ep_idx)->jack_count;
}
uint8_t tuh_midi_v1_get_out_endpoint_jacks(uint8_t index, uint8_t ep_idx)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  return get_out_ep_n_v1(index, ep_idx)->jack_count;
}
uint8_t tuh_midi_v1_get_in_endpoint_jack_id(uint8_t index, uint8_t ep_idx, uint8_t cable_num)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  return get_in_ep_n_v1(index, ep_idx)->jack_id[cable_num];
}
uint8_t tuh_midi_v1_get_out_endpoint_jack_id(uint8_t index, uint8_t ep_idx, uint8_t cable_num)
{
  TU_ASSERT(tuh_midi_spec_version(index) == USB_MIDI_SPEC_VERSION_1);
  return get_out_ep_n_v1(index, ep_idx)->jack_id[cable_num];
}
#endif

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+
void midih_init(void)
{
  tu_memclr(_midih_itf, sizeof(_midih_itf));
  tu_memclr(_stream_blk, sizeof(_stream_blk));
  tu_memclr(_stream_buffer, sizeof(_stream_buffer));
  tu_memclr(_stream_fifo, sizeof(_stream_fifo));
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
  uint8_t index = get_index_by_endpoint(dev_addr, ep_addr);
  midih_interface_t* p_midi = get_itf(index);
  midi_v1_ep_t *endpoint = get_ep_v1_by_addr(index, ep_addr);

  if(event == XFER_RESULT_SUCCESS) {
    if(tu_edpt_dir(ep_addr) == TUSB_DIR_IN) {
      tu_edpt_stream_read_xfer_complete(endpoint->stream, xferred_bytes);
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
    desc_itf =(tusb_desc_interface_t const *) tu_desc_find2((const uint8_t *)desc_itf, (const uint8_t *)(desc_itf + max_len),
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
#if (CFG_TUH_MIDI_EP_MAX > 1)
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
  #else
  desc = tu_desc_find(desc, (void *) desc_itf + max_len, TUSB_DESC_ENDPOINT);
  for(uint8_t i = 0; i < desc_itf->bNumEndpoints; i++) {
    if(itf->ep_in && itf->ep_out)
      break;
    TU_VERIFY(tu_desc_type(desc) == TUSB_DESC_ENDPOINT);
    const tusb_desc_endpoint_t *ep = (tusb_desc_endpoint_t *)desc;
    uint8_t direction = tu_edpt_dir(ep->bEndpointAddress);
    uint8_t ep_index;
    tu_edpt_stream_t *stream;
    switch(direction) {
    case TUSB_DIR_IN:
      if(itf->ep_in) continue;
      itf->ep_in = ep->bEndpointAddress;
      ep_index = alloc_stream_blk(get_index(itf->dev_num, itf->itf_num));
      stream = get_stream_blk(ep_index);
      itf->io.v1.ep_in.stream = stream;
      break;
    case TUSB_DIR_OUT:
      if(itf->ep_out) continue;
      itf->ep_out = ep->bEndpointAddress;
      ep_index = alloc_stream_blk(get_index(itf->dev_num, itf->itf_num));
      stream = get_stream_blk(ep_index);
      itf->io.v1.ep_in.stream = stream;
      break;
    }
    void *ep_buffer = (void *) get_stream_buffer(ep_index);
    void *fifo_buffer = (void *) get_stream_fifo(ep_index);
    TU_LOG_MIDIH("MIDIh open stream: device/interface/endpoint = %d/%d/%d\r\n",
                  itf->dev_num, itf->itf_num, ep->bEndpointAddress);
    tu_edpt_stream_init(stream, true, (direction == TUSB_DIR_OUT), true,
                        fifo_buffer, MIDIH_STREAM_FIFO_SZ,
                        ep_buffer, MIDIH_STREAM_BUFFER_SZ);
  }
  #endif

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
