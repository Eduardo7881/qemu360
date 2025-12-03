#include "qemu/osdep.h"
#include "hw/xbox360/xbox360_xma.h"
#include "hw/xbox360/xbox360.h"
#include "hw/audio/soundhw.h"
#include "audio/audio.h"
#include "qemu/timer.h"
#include "qemu/thread.h"
#include "migration/vmstate.h"

/* ==================== XMA CONFIGURATION ==================== */
#define XMA_BASE_ADDRESS          0x80005000
#define XMA_REGISTER_SIZE         0x1000

#define XMA_MAX_CHANNELS          256
#define XMA_MAX_BUFFERS           32
#define XMA_BUFFER_SIZE           0x8000      /* 32KB per buffer */
#define XMA_SAMPLE_RATE           48000
#define XMA_BITS_PER_SAMPLE       16
#define XMA_CHANNELS              2           /* Stereo */

/* XMA Registers */
#define XMA_REG_CONTROL           0x0000
#define XMA_REG_STATUS            0x0004
#define XMA_REG_INTERRUPT         0x0008
#define XMA_REG_SAMPLE_RATE       0x000C
#define XMA_REG_VOLUME            0x0010
#define XMA_REG_PLAYBACK_POS      0x0014
#define XMA_REG_BUFFER_BASE       0x0020
#define XMA_REG_BUFFER_SIZE       0x0024
#define XMA_REG_BUFFER_WRITE      0x0028
#define XMA_REG_BUFFER_READ       0x002C
#define XMA_REG_DECODE_STATE      0x0030
#define XMA_REG_DECODE_POS        0x0034

/* XMA Control Bits */
#define XMA_CTRL_ENABLE           (1 << 0)
#define XMA_CTRL_RESET            (1 << 1)
#define XMA_CTRL_PLAY             (1 << 2)
#define XMA_CTRL_PAUSE            (1 << 3)
#define XMA_CTRL_LOOP             (1 << 4)
#define XMA_CTRL_INTERRUPT_EN     (1 << 5)
#define XMA_CTRL_DMA_EN           (1 << 6)

/* XMA Status Bits */
#define XMA_STATUS_READY          (1 << 0)
#define XMA_STATUS_PLAYING        (1 << 1)
#define XMA_STATUS_BUFFER_EMPTY   (1 << 2)
#define XMA_STATUS_BUFFER_FULL    (1 << 3)
#define XMA_STATUS_DECODE_ERROR   (1 << 4)
#define XMA_STATUS_DMA_ACTIVE     (1 << 5)

/* XMA Interrupt Bits */
#define XMA_INTR_BUFFER_EMPTY     (1 << 0)
#define XMA_INTR_BUFFER_HALF      (1 << 1)
#define XMA_INTR_DECODE_COMPLETE  (1 << 2)
#define XMA_INTR_DECODE_ERROR     (1 << 3)
#define XMA_INTR_DMA_COMPLETE     (1 << 4)

/* XMA Decoder States */
#define XMA_STATE_IDLE            0
#define XMA_STATE_DECODING        1
#define XMA_STATE_WAITING         2
#define XMA_STATE_ERROR           3

/* ==================== XMA BUFFER STRUCTURES ==================== */

typedef struct XMABuffer {
    uint32_t physical_addr;
    uint32_t size;
    uint32_t write_pos;
    uint32_t read_pos;
    uint8_t *data;
    bool valid;
    bool locked;
} XMABuffer;

typedef struct XMAChannel {
    uint32_t control;
    uint32_t status;
    uint32_t sample_rate;
    uint32_t volume;
    uint32_t playback_pos;
    uint32_t buffer_base;
    uint32_t buffer_size;
    uint32_t decode_state;
    uint32_t decode_pos;
    
    XMABuffer *buffers[XMA_MAX_BUFFERS];
    int current_buffer;
    int buffer_count;
    
    bool playing;
    bool loop;
    uint64_t samples_decoded;
    uint64_t total_samples;
    
    QEMUTimer *timer;
    SWVoiceOut *voice;
} XMAChannel;

/* ==================== XMA DECODER STATE ==================== */

typedef struct XMAFrame {
    uint8_t header[4];
    uint32_t frame_size;
    uint32_t sample_count;
    uint8_t *compressed_data;
    int16_t *pcm_data;
} XMAFrame;

typedef struct XMADecoder {
    /* XMA stream info */
    uint32_t sample_rate;
    uint32_t channels;
    uint32_t bits_per_sample;
    uint32_t block_size;
    
    /* Decoder state */
    int16_t prev_sample[2];
    int16_t step_index[2];
    uint32_t partial_frame;
    uint32_t bit_buffer;
    uint32_t bit_count;
    
    /* Current frame */
    XMAFrame current_frame;
    uint32_t frame_pos;
} XMADecoder;

/* ==================== XMA CONTROLLER STATE ==================== */

typedef struct XenonXMAState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    
    /* Registers */
    uint32_t control;
    uint32_t status;
    uint32_t interrupt;
    uint32_t sample_rate;
    uint32_t volume;
    uint32_t playback_pos;
    uint32_t buffer_base;
    uint32_t buffer_size;
    uint32_t buffer_write;
    uint32_t buffer_read;
    uint32_t decode_state;
    uint32_t decode_pos;
    
    /* Channels */
    XMAChannel channels[XMA_MAX_CHANNELS];
    int active_channels;
    
    /* Buffers */
    XMABuffer buffers[XMA_MAX_BUFFERS];
    int buffer_count;
    
    /* Decoder */
    XMADecoder decoder;
    
    /* DMA */
    struct {
        uint32_t control;
        uint32_t src_addr;
        uint32_t dst_addr;
        uint32_t transfer_size;
        bool active;
        QEMUTimer *timer;
    } dma;
    
    /* Audio */
    QEMUSoundCard card;
    struct audsettings as;
    AudioState *audio_state;
    
    /* Interrupts */
    qemu_irq irq;
    
    /* Statistics */
    uint64_t total_frames_decoded;
    uint64_t total_samples;
    uint64_t underruns;
    
    /* Xenon-specific */
    uint32_t xenon_features;
    uint32_t debug;
} XenonXMAState;

/* ==================== REGISTER ACCESS ==================== */

static uint64_t xma_read(void *opaque, hwaddr offset, unsigned size) {
    XenonXMAState *s = opaque;
    uint32_t value = 0;
    int ch;
    
    switch (offset) {
        case XMA_REG_CONTROL:
            value = s->control;
            break;
        case XMA_REG_STATUS:
            value = s->status;
            for (int i = 0; i < XMA_MAX_CHANNELS; i++) {
                if (s->channels[i].playing) {
                    value |= XMA_STATUS_PLAYING;
                    break;
                }
            }
            break;
        case XMA_REG_INTERRUPT:
            value = s->interrupt;
            break;
        case XMA_REG_SAMPLE_RATE:
            value = s->sample_rate;
            break;
        case XMA_REG_VOLUME:
            value = s->volume;
            break;
        case XMA_REG_PLAYBACK_POS:
            value = s->playback_pos;
            break;
        case XMA_REG_BUFFER_BASE:
            value = s->buffer_base;
            break;
        case XMA_REG_BUFFER_SIZE:
            value = s->buffer_size;
            break;
        case XMA_REG_BUFFER_WRITE:
            value = s->buffer_write;
            break;
        case XMA_REG_BUFFER_READ:
            value = s->buffer_read;
            break;
        case XMA_REG_DECODE_STATE:
            value = s->decode_state;
            break;
        case XMA_REG_DECODE_POS:
            value = s->decode_pos;
            break;
        default:
            /* Channel-specific registers */
            if (offset >= 0x0100 && offset < 0x0100 + XMA_MAX_CHANNELS * 0x40) {
                ch = (offset - 0x0100) / 0x40;
                offset = (offset - 0x0100) % 0x40;
                
                if (ch < XMA_MAX_CHANNELS) {
                    XMAChannel *c = &s->channels[ch];
                    switch (offset) {
                        case 0x00: value = c->control; break;
                        case 0x04: value = c->status; break;
                        case 0x08: value = c->sample_rate; break;
                        case 0x0C: value = c->volume; break;
                        case 0x10: value = c->playback_pos; break;
                        case 0x14: value = c->buffer_base; break;
                        case 0x18: value = c->buffer_size; break;
                        case 0x20: value = c->decode_state; break;
                        case 0x24: value = c->decode_pos; break;
                    }
                }
            }
            break;
    }
    
    return value;
}

static void xma_write(void *opaque, hwaddr offset, 
                     uint64_t value, unsigned size) {
    XenonXMAState *s = opaque;
    int ch;
    
    switch (offset) {
        case XMA_REG_CONTROL:
            s->control = value;
            if (value & XMA_CTRL_RESET) {
                xma_reset(s);
            }
            if (value & XMA_CTRL_ENABLE) {
                s->status |= XMA_STATUS_READY;
            } else {
                s->status &= ~XMA_STATUS_READY;
            }
            break;
        case XMA_REG_INTERRUPT:
            s->interrupt = value;
            break;
        case XMA_REG_SAMPLE_RATE:
            s->sample_rate = value & 0xFFFF;
            break;
        case XMA_REG_VOLUME:
            s->volume = value & 0xFFFF;
            audio_set_volume(s->audio_state, value & 0xFFFF);
            break;
        case XMA_REG_BUFFER_BASE:
            s->buffer_base = value;
            break;
        case XMA_REG_BUFFER_SIZE:
            s->buffer_size = value & 0xFFFF;
            break;
        case XMA_REG_BUFFER_WRITE:
            s->buffer_write = value;
            xma_buffer_write(s, value);
            break;
        case XMA_REG_BUFFER_READ:
            s->buffer_read = value;
            break;
        default:
            /* Channel-specific registers */
            if (offset >= 0x0100 && offset < 0x0100 + XMA_MAX_CHANNELS * 0x40) {
                ch = (offset - 0x0100) / 0x40;
                offset = (offset - 0x0100) % 0x40;
                
                if (ch < XMA_MAX_CHANNELS) {
                    XMAChannel *c = &s->channels[ch];
                    switch (offset) {
                        case 0x00:  /* Control */
                            c->control = value;
                            if (value & XMA_CTRL_PLAY) {
                                xma_channel_start(s, ch);
                            }
                            if (value & XMA_CTRL_PAUSE) {
                                xma_channel_pause(s, ch);
                            }
                            if (value & XMA_CTRL_RESET) {
                                xma_channel_reset(s, ch);
                            }
                            c->loop = (value & XMA_CTRL_LOOP) != 0;
                            break;
                        case 0x08:  /* Sample Rate */
                            c->sample_rate = value & 0xFFFF;
                            break;
                        case 0x0C:  /* Volume */
                            c->volume = value & 0xFFFF;
                            break;
                        case 0x14:  /* Buffer Base */
                            c->buffer_base = value;
                            break;
                        case 0x18:  /* Buffer Size */
                            c->buffer_size = value & 0xFFFF;
                            break;
                    }
                }
            }
            break;
    }
}

static const MemoryRegionOps xma_ops = {
    .read = xma_read,
    .write = xma_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
    .impl = { .min_access_size = 4, .max_access_size = 4 },
};

/* ==================== XMA DECODER FUNCTIONS ==================== */

/* XMA ADPCM tables */
static const int16_t xma_step_size_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
    253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
    3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487,
    12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

static const int8_t xma_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static int16_t xma_decode_sample(int16_t prev_sample, int16_t *step_index,
                                 uint8_t nibble, int channel) {
    int step = xma_step_size_table[*step_index];
    int diff = step >> 3;
    
    if (nibble & 1) diff += step >> 2;
    if (nibble & 2) diff += step >> 1;
    if (nibble & 4) diff += step;
    if (nibble & 8) diff = -diff;
    
    int16_t sample = prev_sample + diff;
    
    /* Clamp to 16-bit range */
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    
    /* Update step index */
    *step_index += xma_index_table[nibble & 0x7];
    if (*step_index < 0) *step_index = 0;
    if (*step_index > 88) *step_index = 88;
    
    return sample;
}

static int xma_decode_frame(XMADecoder *decoder, uint8_t *input,
                           int16_t *output, uint32_t input_size) {
    if (input_size < 4) {
        return 0;
    }
    
    /* Parse XMA frame header */
    uint32_t header = (input[0] << 24) | (input[1] << 16) | 
                     (input[2] << 8) | input[3];
    
    int frame_size = (header >> 16) & 0x1FFF;
    int sample_count = (header >> 8) & 0xFF;
    int channels = (header >> 1) & 0x3;
    int bits_per_sample = (header >> 4) & 0x3;
    
    if (frame_size > input_size) {
        return 0;
    }
    
    /* Initialize decoder state if needed */
    if (decoder->partial_frame == 0) {
        decoder->prev_sample[0] = 0;
        decoder->prev_sample[1] = 0;
        decoder->step_index[0] = 0;
        decoder->step_index[1] = 0;
        decoder->bit_buffer = 0;
        decoder->bit_count = 0;
    }
    
    /* Decode samples */
    int output_pos = 0;
    uint8_t *data = input + 4;
    int data_size = frame_size - 4;
    
    for (int i = 0; i < data_size * 2; i++) {
        if (decoder->bit_count == 0) {
            if (data_size == 0) break;
            decoder->bit_buffer = *data++;
            data_size--;
            decoder->bit_count = 8;
        }
        
        uint8_t nibble = (decoder->bit_buffer >> 4) & 0xF;
        decoder->bit_buffer <<= 4;
        decoder->bit_count -= 4;
        
        /* Decode for each channel */
        for (int ch = 0; ch < channels; ch++) {
            if (ch == 0) {
                int16_t sample = xma_decode_sample(
                    decoder->prev_sample[ch],
                    &decoder->step_index[ch],
                    nibble, ch);
                decoder->prev_sample[ch] = sample;
                output[output_pos++] = sample;
            } else if (ch == 1 && channels > 1) {
                /* Second channel uses same nibble? */
                /* Simplified: just copy first channel */
                output[output_pos++] = decoder->prev_sample[0];
            }
        }
    }
    
    decoder->partial_frame++;
    return output_pos / channels;  /* Return number of samples per channel */
}

/* ==================== BUFFER MANAGEMENT ==================== */

static void xma_buffer_write(XenonXMAState *s, uint32_t value) {
    uint32_t addr = s->buffer_base + s->buffer_write;
    uint32_t size = s->buffer_size;
    
    /* Find free buffer */
    XMABuffer *buf = NULL;
    for (int i = 0; i < XMA_MAX_BUFFERS; i++) {
        if (!s->buffers[i].valid) {
            buf = &s->buffers[i];
            break;
        }
    }
    
    if (!buf) {
        /* No free buffers */
        s->status |= XMA_STATUS_BUFFER_FULL;
        return;
    }
    
    buf->physical_addr = addr;
    buf->size = size;
    buf->write_pos = 0;
    buf->read_pos = 0;
    buf->valid = true;
    buf->locked = false;
    
    if (!buf->data) {
        buf->data = g_malloc(XMA_BUFFER_SIZE);
    }
    
    /* Read data from memory into buffer */
    cpu_physical_memory_read(addr, buf->data, size);
    buf->write_pos = size;
    
    s->buffer_write = (s->buffer_write + size) & 0xFFFF;
    
    /* Check if buffers need to be processed */
    for (int ch = 0; ch < XMA_MAX_CHANNELS; ch++) {
        if (s->channels[ch].playing) {
            xma_process_buffers(s, ch);
        }
    }
}

static void xma_process_buffers(XenonXMAState *s, int channel) {
    XMAChannel *c = &s->channels[channel];
    
    if (c->buffer_count >= XMA_MAX_BUFFERS) {
        return;
    }
    
    /* Find buffers for this channel */
    for (int i = 0; i < XMA_MAX_BUFFERS; i++) {
        if (s->buffers[i].valid && !s->buffers[i].locked) {
            if (s->buffers[i].physical_addr >= c->buffer_base &&
                s->buffers[i].physical_addr < c->buffer_base + c->buffer_size) {
                
                s->buffers[i].locked = true;
                c->buffers[c->buffer_count++] = &s->buffers[i];
                
                if (c->buffer_count == 1) {
                    /* Start decoding first buffer */
                    xma_start_decode(s, channel);
                }
            }
        }
    }
}

/* ==================== CHANNEL MANAGEMENT ==================== */

static void xma_channel_start(XenonXMAState *s, int channel) {
    XMAChannel *c = &s->channels[channel];
    
    if (c->playing || c->buffer_count == 0) {
        return;
    }
    
    c->playing = true;
    c->status |= XMA_STATUS_PLAYING;
    s->status |= XMA_STATUS_PLAYING;
    
    /* Create audio voice if needed */
    if (!c->voice) {
        c->voice = AUD_open_out(&s->card, c->voice, "xma",
                                s, xma_audio_callback, &s->as);
    }
    
    /* Start decode timer */
    int64_t period_ns = NANOSECONDS_PER_SECOND / c->sample_rate;
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
    
    printf("[XMA] Channel %d started: %d Hz, %d buffers\n",
           channel, c->sample_rate, c->buffer_count);
}

static void xma_channel_pause(XenonXMAState *s, int channel) {
    XMAChannel *c = &s->channels[channel];
    
    if (!c->playing) {
        return;
    }
    
    c->playing = false;
    c->status &= ~XMA_STATUS_PLAYING;
    
    timer_del(c->timer);
    
    /* Check if any channels are still playing */
    s->status &= ~XMA_STATUS_PLAYING;
    for (int i = 0; i < XMA_MAX_CHANNELS; i++) {
        if (s->channels[i].playing) {
            s->status |= XMA_STATUS_PLAYING;
            break;
        }
    }
}

static void xma_channel_reset(XenonXMAState *s, int channel) {
    XMAChannel *c = &s->channels[channel];
    
    timer_del(c->timer);
    
    c->control = 0;
    c->status = 0;
    c->sample_rate = XMA_SAMPLE_RATE;
    c->volume = 0xFFFF;
    c->playback_pos = 0;
    c->buffer_base = 0;
    c->buffer_size = 0;
    c->decode_state = XMA_STATE_IDLE;
    c->decode_pos = 0;
    
    c->playing = false;
    c->loop = false;
    c->samples_decoded = 0;
    c->total_samples = 0;
    
    /* Release buffers */
    for (int i = 0; i < c->buffer_count; i++) {
        if (c->buffers[i]) {
            c->buffers[i]->locked = false;
        }
    }
    c->buffer_count = 0;
    c->current_buffer = 0;
}

static void xma_start_decode(XenonXMAState *s, int channel) {
    XMAChannel *c = &s->channels[channel];
    
    if (c->buffer_count == 0 || c->current_buffer >= c->buffer_count) {
        return;
    }
    
    XMABuffer *buf = c->buffers[c->current_buffer];
    if (!buf || buf->read_pos >= buf->write_pos) {
        /* Move to next buffer */
        c->current_buffer++;
        if (c->current_buffer >= c->buffer_count) {
            if (c->loop) {
                c->current_buffer = 0;
            } else {
                c->playing = false;
                c->status &= ~XMA_STATUS_PLAYING;
                return;
            }
        }
        buf = c->buffers[c->current_buffer];
    }
    
    c->decode_state = XMA_STATE_DECODING;
    c->decode_pos = buf->read_pos;
    
    /* Start decode timer */
    int64_t decode_time_ns = (buf->write_pos - buf->read_pos) * 100; /* Simplified */
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + decode_time_ns);
}

/* ==================== AUDIO CALLBACK ==================== */

static void xma_audio_callback(void *opaque, int free) {
    XenonXMAState *s = opaque;
    int16_t buffer[1024 * 2];  /* Stereo */
    int samples = free / (XMA_BITS_PER_SAMPLE / 8 * XMA_CHANNELS);
    
    if (samples > 1024) {
        samples = 1024;
    }
    
    /* Mix all active channels */
    memset(buffer, 0, samples * XMA_CHANNELS * sizeof(int16_t));
    
    for (int ch = 0; ch < XMA_MAX_CHANNELS; ch++) {
        if (s->channels[ch].playing) {
            /* Generate samples for this channel */
            int16_t channel_buffer[1024];
            int channel_samples = xma_generate_samples(s, ch, channel_buffer, samples);
            
            /* Mix into output buffer */
            for (int i = 0; i < channel_samples * XMA_CHANNELS; i++) {
                int32_t mixed = buffer[i] + channel_buffer[i];
                if (mixed > 32767) mixed = 32767;
                if (mixed < -32768) mixed = -32768;
                buffer[i] = mixed;
            }
        }
    }
    
    /* Send to audio backend */
    if (s->audio_state) {
        AUD_write(s->audio_state, buffer, samples * XMA_CHANNELS * sizeof(int16_t));
    }
}

static int xma_generate_samples(XenonXMAState *s, int channel,
                               int16_t *buffer, int samples) {
    XMAChannel *c = &s->channels[channel];
    
    if (c->current_buffer >= c->buffer_count) {
        return 0;
    }
    
    XMABuffer *buf = c->buffers[c->current_buffer];
    if (!buf) {
        return 0;
    }
    
    int samples_generated = 0;
    int max_samples = MIN(samples, 1024);
    
    while (samples_generated < max_samples) {
        if (buf->read_pos >= buf->write_pos) {
            /* Buffer exhausted, move to next */
            c->current_buffer++;
            if (c->current_buffer >= c->buffer_count) {
                if (c->loop) {
                    c->current_buffer = 0;
                } else {
                    /* End of stream */
                    c->playing = false;
                    c->status &= ~XMA_STATUS_PLAYING;
                    break;
                }
            }
            buf = c->buffers[c->current_buffer];
            if (!buf) break;
        }
        
        /* Decode a frame */
        int frame_samples = xma_decode_frame(&s->decoder,
                                            buf->data + buf->read_pos,
                                            buffer + samples_generated * XMA_CHANNELS,
                                            buf->write_pos - buf->read_pos);
        
        if (frame_samples > 0) {
            buf->read_pos += s->decoder.current_frame.frame_size;
            samples_generated += frame_samples;
            c->samples_decoded += frame_samples;
            s->total_samples += frame_samples;
            s->total_frames_decoded++;
        } else {
            /* Decode error */
            c->decode_state = XMA_STATE_ERROR;
            c->status |= XMA_STATUS_DECODE_ERROR;
            s->interrupt |= XMA_INTR_DECODE_ERROR;
            break;
        }
    }
    
    return samples_generated;
}

/* ==================== DEVICE INITIALIZATION ==================== */

static void xenon_xma_realize(DeviceState *dev, Error **errp) {
    XenonXMAState *s = XENON_XMA(dev);
    
    /* Initialize registers */
    s->control = XMA_CTRL_ENABLE;
    s->status = XMA_STATUS_READY;
    s->interrupt = 0;
    s->sample_rate = XMA_SAMPLE_RATE;
    s->volume = 0xFFFF;
    
    /* Initialize channels */
    for (int i = 0; i < XMA_MAX_CHANNELS; i++) {
        XMAChannel *c = &s->channels[i];
        
        c->control = 0;
        c->status = 0;
        c->sample_rate = XMA_SAMPLE_RATE;
        c->volume = 0xFFFF;
        c->playback_pos = 0;
        c->buffer_base = 0;
        c->buffer_size = 0;
        c->decode_state = XMA_STATE_IDLE;
        c->decode_pos = 0;
        
        c->playing = false;
        c->loop = false;
        c->samples_decoded = 0;
        c->total_samples = 0;
        c->buffer_count = 0;
        c->current_buffer = 0;
        
        c->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xma_channel_timer_cb, s);
        timer_set_cb(c->timer, xma_channel_timer_cb, INT2VOIDPTR(i));
    }
    
    /* Initialize buffers */
    for (int i = 0; i < XMA_MAX_BUFFERS; i++) {
        s->buffers[i].physical_addr = 0;
        s->buffers[i].size = 0;
        s->buffers[i].write_pos = 0;
        s->buffers[i].read_pos = 0;
        s->buffers[i].data = NULL;
        s->buffers[i].valid = false;
        s->buffers[i].locked = false;
    }
    
    /* Initialize decoder */
    memset(&s->decoder, 0, sizeof(XMADecoder));
    s->decoder.sample_rate = XMA_SAMPLE_RATE;
    s->decoder.channels = XMA_CHANNELS;
    s->decoder.bits_per_sample = XMA_BITS_PER_SAMPLE;
    s->decoder.block_size = 2048;
    
    /* Initialize DMA */
    s->dma.control = 0;
    s->dma.active = false;
    s->dma.timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, xma_dma_complete, s);
    
    /* Initialize audio */
    s->as.freq = XMA_SAMPLE_RATE;
    s->as.nchannels = XMA_CHANNELS;
    s->as.fmt = AUDIO_FORMAT_S16;
    s->as.endianness = AUDIO_HOST_ENDIANNESS;
    
    s->audio_state = AUD_init(&s->card, s->audio_state);
    if (!s->audio_state) {
        error_setg(errp, "Failed to initialize audio");
        return;
    }
    
    /* Initialize statistics */
    s->total_frames_decoded = 0;
    s->total_samples = 0;
    s->underruns = 0;
    
    /* Initialize memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &xma_ops, s,
                         "xenon.xma", XMA_REGISTER_SIZE);
    
    printf("[XMA] Xenon XMA Audio Processor initialized\n");
    printf("[XMA] %d channels, %d buffers, %d Hz\n",
           XMA_MAX_CHANNELS, XMA_MAX_BUFFERS, XMA_SAMPLE_RATE);
}

static void xenon_xma_reset(DeviceState *dev) {
    XenonXMAState *s = XENON_XMA(dev);
    
    s->control = XMA_CTRL_ENABLE;
    s->status = XMA_STATUS_READY;
    s->interrupt = 0;
    
    /* Reset all channels */
    for (int i = 0; i < XMA_MAX_CHANNELS; i++) {
        xma_channel_reset(s, i);
    }
    
    /* Reset buffers */
    for (int i = 0; i < XMA_MAX_BUFFERS; i++) {
        if (s->buffers[i].data) {
            g_free(s->buffers[i].data);
            s->buffers[i].data = NULL;
        }
        s->buffers[i].valid = false;
        s->buffers[i].locked = false;
    }
    
    /* Reset DMA */
    s->dma.control = 0;
    s->dma.active = false;
    timer_del(s->dma.timer);
    
    qemu_set_irq(s->irq, 0);
}

static void xma_channel_timer_cb(void *opaque) {
    uintptr_t channel = (uintptr_t)opaque;
    XenonXMAState *s = container_of(opaque, XenonXMAState, channels[channel]);
    
    XMAChannel *c = &s->channels[channel];
    
    if (!c->playing) {
        return;
    }
    
    /* Process next chunk of audio */
    xma_process_buffers(s, channel);
    
    /* Schedule next timer tick */
    int64_t period_ns = NANOSECONDS_PER_SECOND / c->sample_rate;
    timer_mod(c->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + period_ns);
}

/* ==================== QEMU DEVICE ==================== */

static void xenon_xma_class_init(ObjectClass *klass, void *data) {
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    dc->realize = xenon_xma_realize;
    dc->reset = xenon_xma_reset;
    dc->desc = "Xenon XMA Audio Processor";
}

static const TypeInfo xenon_xma_type_info = {
    .name = TYPE_XENON_XMA,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(XenonXMAState),
    .class_init = xenon_xma_class_init,
};

static void xenon_xma_register_types(void) {
    type_register_static(&xenon_xma_type_info);
}

type_init(xenon_xma_register_types);

/* ==================== PUBLIC FUNCTIONS ==================== */

XenonXMAState *xenon_xma_create(MemoryRegion *parent, hwaddr base) {
    DeviceState *dev;
    XenonXMAState *s;
    
    dev = qdev_new(TYPE_XENON_XMA);
    s = XENON_XMA(dev);
    
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    
    /* Connect interrupt */
    s->irq = qemu_allocate_irq(xenon_xma_irq_handler, s, 0);
    
    return s;
}

void xenon_xma_play_buffer(XenonXMAState *s, int channel,
                          uint32_t buffer_addr, uint32_t buffer_size) {
    if (channel < 0 || channel >= XMA_MAX_CHANNELS) {
        return;
    }
    
    XMAChannel *c = &s->channels[channel];
    
    /* Set up buffer */
    c->buffer_base = buffer_addr;
    c->buffer_size = buffer_size;
    
    /* Start playback */
    c->control |= XMA_CTRL_PLAY;
    xma_channel_start(s, channel);
}

void xenon_xma_stop_channel(XenonXMAState *s, int channel) {
    if (channel < 0 || channel >= XMA_MAX_CHANNELS) {
        return;
    }
    
    xma_channel_pause(s, channel);
}

void xenon_xma_dump_state(XenonXMAState *s) {
    printf("Xenon XMA Audio Processor State:\n");
    printf("  Control: 0x%08X, Status: 0x%08X, Interrupt: 0x%08X\n",
           s->control, s->status, s->interrupt);
    printf("  Sample Rate: %d Hz, Volume: 0x%04X\n",
           s->sample_rate, s->volume);
    printf("  Buffers: %d valid, %d locked\n",
           s->buffer_count, xma_count_locked_buffers(s));
    printf("  Statistics: %" PRIu64 " frames, %" PRIu64 " samples\n",
           s->total_frames_decoded, s->total_samples);
    
    printf("  Active channels:\n");
    for (int i = 0; i < XMA_MAX_CHANNELS; i++) {
        if (s->channels[i].playing) {
            XMAChannel *c = &s->channels[i];
            printf("    Channel %d: %d Hz, %d buffers, %" PRIu64 " samples\n",
                   i, c->sample_rate, c->buffer_count, c->samples_decoded);
        }
    }
}
