/* ESP-NOW audio sink (I2S master + PLC)
 *
 * Notes:
 * - Keep RX callback tiny (copy only, no decode)
 * - Let blocking I2S writes be the real timing source
 * - If a packet is late/missing, replay last frame (PLC)
 * - Decode in playback task, not in the callback
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"

static const char *TAG = "ADPCM_SINK";

// ---- Config ----
#define SAMPLE_RATE       48000
#define CHANNELS          2
#define MAX_PAYLOAD       96       // 96 stereo sample-pairs (~2.0ms @ 48k)
#define HEADER_SIZE       19       // magic + seq + payload_len + L/R state + src timestamp
#define SAMPLES_PER_FRAME MAX_PAYLOAD    // 1 ADPCM byte = 1 stereo sample-pair
#define BYTES_PER_FRAME   (SAMPLES_PER_FRAME * CHANNELS * sizeof(int32_t))
#define I2S_OUT_BITS      I2S_DATA_BIT_WIDTH_24BIT
#define I2S_BYTES_PER_SAMPLE sizeof(int32_t)
#define I2S_BYTES_PER_FRAME (SAMPLES_PER_FRAME * CHANNELS * I2S_BYTES_PER_SAMPLE)
#define PCM24_MAX         8388607
#define PCM24_MIN        -8388608

// Timing helpers
#define FRAME_TIME_US     ((SAMPLES_PER_FRAME * 1000000) / SAMPLE_RATE)  // ~5333us per frame
#define DMA_FRAMES        32       // ~0.67ms per DMA buffer
#define DMA_BUFFERS       2        // DMA buffer count
#define DMA_LATENCY_US    ((DMA_FRAMES * DMA_BUFFERS * 1000000) / SAMPLE_RATE)  // ~5333us

// I2S pins
#define PIN_BCLK          GPIO_NUM_27
#define PIN_WS            GPIO_NUM_25
#define PIN_DOUT          GPIO_NUM_26

// ESP-NOW
#define WIFI_CHANNEL      11
#define AUDIO_MAGIC       0xAD

// Packet queue (single producer callback, single consumer playback task)
#define PACKET_RING_SIZE  12
#define PREBUFFER_FRAMES  1        // unmute after 1 frame queued
#define TARGET_BUF_FILL   1        // normal queue target
#define TARGET_BUF_FILL_MAX 2      // temporary target under jitter
#define CROSSFADE_SAMPLES 24       // short blend after PLC recovery
#define PACKET_WAIT_MS    1        // normal packet wait before PLC
#define PACKET_WAIT_MS_MAX 2       // temporary wait under jitter
#define OUTPUT_GAIN_NUM   4        // gain numerator (4/2 = 2.0x)
#define OUTPUT_GAIN_DEN   2
#define TARGET_PLAY_LAG_US 12000   // target source->play lag
#define MAX_PLAY_LAG_US   17000    // hard lag ceiling
#define MIN_PLAY_LAG_US    9000    // low lag boundary
#define LIMITER_THRESHOLD 6000000  // soft limiter knee
#define LIMITER_RATIO_NUM 1        // limiter ratio above knee: 1/4
#define LIMITER_RATIO_DEN 4
#define SOURCE_TIMEOUT_US 120000   // if source is silent too long, reset stream

// ==================== IMA ADPCM TABLES ====================
static const int16_t ima_index_table[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

static const int16_t ima_step_table[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

// ---- Packet format ----
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint16_t seq;
    uint16_t payload_len;    // valid bytes in payload[]
    int32_t  pred_L;
    int8_t   index_L;
    int32_t  pred_R;
    int8_t   index_R;
    uint32_t src_t_us;       // source timestamp (lower 32-bit usec)
    uint8_t  payload[MAX_PAYLOAD];
} audio_packet_t;

// ---- State ----
static i2s_chan_handle_t i2s_tx = NULL;
static TaskHandle_t playback_task_handle = NULL;

typedef struct {
    audio_packet_t pkt;
} packet_slot_t;

static packet_slot_t packet_ring[PACKET_RING_SIZE];
static volatile uint8_t ring_head = 0;
static volatile uint8_t ring_tail = 0;
static volatile uint8_t ring_count = 0;
static portMUX_TYPE ring_lock = portMUX_INITIALIZER_UNLOCKED;

// Runtime counters
static volatile uint32_t rx_count = 0, rx_dropped = 0, underruns = 0, played = 0;
static volatile int32_t clock_offset_us = 0;
static volatile int32_t clock_jitter_us = 0;
static volatile int32_t last_play_lag_us = 0;
static volatile uint32_t clock_sync_updates = 0;
static volatile int32_t clock_offset_ref_us = 0;
static volatile int32_t clock_drift_us = 0;
static volatile uint32_t sync_drops = 0;
static volatile uint32_t sync_holds = 0;
static volatile uint32_t last_rx_us = 0;
static volatile uint8_t adaptive_target_fill = TARGET_BUF_FILL;
static volatile uint8_t adaptive_packet_wait_ms = PACKET_WAIT_MS;

static inline int32_t abs_i32(int32_t value) {
    return (value < 0) ? -value : value;
}

static inline int32_t sat24_from_i32(int32_t value) {
    if (value > PCM24_MAX) return PCM24_MAX;
    if (value < PCM24_MIN) return PCM24_MIN;
    return value;
}

static inline int32_t soft_limit_sample(int32_t value) {
    int32_t sign = (value < 0) ? -1 : 1;
    int32_t abs_value = (value < 0) ? -value : value;

    if (abs_value <= LIMITER_THRESHOLD) {
        return sat24_from_i32(value);
    }

    int32_t excess = abs_value - LIMITER_THRESHOLD;
    int32_t compressed = LIMITER_THRESHOLD + (excess * LIMITER_RATIO_NUM) / LIMITER_RATIO_DEN;
    if (compressed > PCM24_MAX) compressed = PCM24_MAX;
    return sign * compressed;
}

static inline void apply_output_gain(int32_t *pcm, int samples_per_channel) {
    const int total_samples = samples_per_channel * CHANNELS;
    for (int i = 0; i < total_samples; i++) {
        int32_t scaled = (pcm[i] * OUTPUT_GAIN_NUM) / OUTPUT_GAIN_DEN;
        pcm[i] = soft_limit_sample(scaled);
    }
}

static inline void pcm24_to_i2s24(const int32_t *pcm24, int32_t *i2s_out, int samples_per_channel) {
    const int total_samples = samples_per_channel * CHANNELS;
    for (int i = 0; i < total_samples; i++) {
        // Put signed 24-bit PCM into 32-bit I2S slot (MSB aligned).
        i2s_out[i] = sat24_from_i32(pcm24[i]) << 8;
    }
}

static inline uint8_t ring_count_get(void) {
    uint8_t count;
    portENTER_CRITICAL(&ring_lock);
    count = ring_count;
    portEXIT_CRITICAL(&ring_lock);
    return count;
}

static inline void ring_clear(void) {
    portENTER_CRITICAL(&ring_lock);
    ring_head = 0;
    ring_tail = 0;
    ring_count = 0;
    portEXIT_CRITICAL(&ring_lock);
}

static bool ring_pop(audio_packet_t *out) {
    bool got = false;
    portENTER_CRITICAL(&ring_lock);
    if (ring_count > 0) {
        *out = packet_ring[ring_tail].pkt;
        ring_tail = (ring_tail + 1) % PACKET_RING_SIZE;
        ring_count--;
        got = true;
    }
    portEXIT_CRITICAL(&ring_lock);
    return got;
}

static inline void ring_push(const audio_packet_t *pkt) {
    portENTER_CRITICAL(&ring_lock);

    // Queue full -> drop oldest, keep newest.
    if (ring_count == PACKET_RING_SIZE) {
        ring_tail = (ring_tail + 1) % PACKET_RING_SIZE;
        ring_count--;
        rx_dropped++;
    }

    packet_ring[ring_head].pkt = *pkt;
    ring_head = (ring_head + 1) % PACKET_RING_SIZE;
    ring_count++;

    portEXIT_CRITICAL(&ring_lock);
}

// ---- ESP-NOW RX callback ----
// Keep this callback tiny: copy + queue only.
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    // quick size/magic checks
    if (len < HEADER_SIZE) return;
    if (data[0] != AUDIO_MAGIC) return;
    
    // protect against oversized packet
    if (len > (int)sizeof(audio_packet_t)) return;
    
    rx_count++;
    
    // copy packet into local slot
    audio_packet_t pkt = {0};
    memcpy(&pkt, data, len);
    
    // ignore bad payload_len
    if (pkt.payload_len > MAX_PAYLOAD) return;

    // track source->sink clock offset and jitter
    uint32_t now_us = (uint32_t)esp_timer_get_time();
    last_rx_us = now_us;
    int32_t sample_offset = (int32_t)(now_us - pkt.src_t_us);
    if (clock_sync_updates == 0) {
        clock_offset_us = sample_offset;
        clock_offset_ref_us = sample_offset;
        clock_drift_us = 0;
        clock_jitter_us = 0;
    } else {
        int32_t offset_error = sample_offset - clock_offset_us;

        // source reboot/clock jump -> rebase now
        if (abs_i32(offset_error) > 200000) {
            clock_offset_us = sample_offset;
            clock_offset_ref_us = sample_offset;
            clock_drift_us = 0;
            clock_jitter_us = 0;
        } else {
            clock_offset_us += offset_error / 16;
            clock_drift_us = clock_offset_us - clock_offset_ref_us;
            int32_t inst_jitter = sample_offset - clock_offset_us;
            clock_jitter_us += (inst_jitter - clock_jitter_us) / 16;
        }
    }
    clock_sync_updates++;
    
    // push to queue
    ring_push(&pkt);

    if (playback_task_handle != NULL) {
        xTaskNotifyGive(playback_task_handle);
    }
}

// ---- ADPCM decode ----
typedef struct {
    int32_t predicted;
    int8_t index;
} adpcm_state_t;

static inline int32_t adpcm_decode_sample(uint8_t nibble, adpcm_state_t *state) {
    int32_t step = ((int32_t)ima_step_table[state->index]) << 8;
    int32_t delta = step >> 3;
    
    if (nibble & 4) delta += step;
    if (nibble & 2) delta += step >> 1;
    if (nibble & 1) delta += step >> 2;
    
    int32_t predicted = state->predicted;
    if (nibble & 8)
        predicted -= delta;
    else
        predicted += delta;

    if (predicted > PCM24_MAX) predicted = PCM24_MAX;
    else if (predicted < PCM24_MIN) predicted = PCM24_MIN;
    state->predicted = predicted;
    
    state->index += ima_index_table[nibble];
    if (state->index < 0) state->index = 0;
    else if (state->index > 88) state->index = 88;
    
    return state->predicted;
}

static void adpcm_decode_frame(const audio_packet_t *pkt, int32_t *pcm, int *out_samples) {
    adpcm_state_t state_L = { .predicted = pkt->pred_L, .index = pkt->index_L };
    adpcm_state_t state_R = { .predicted = pkt->pred_R, .index = pkt->index_R };
    
    // Each byte = 1 L sample + 1 R sample (2 nibbles)
    int num_samples = pkt->payload_len;
    *out_samples = num_samples;
    
    for (int i = 0; i < num_samples; i++) {
        uint8_t byte = pkt->payload[i];
        pcm[i * 2] = adpcm_decode_sample(byte >> 4, &state_L);
        pcm[i * 2 + 1] = adpcm_decode_sample(byte & 0x0F, &state_R);
    }
}

// ---- I2S setup ----
static void init_i2s(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_BUFFERS;   // 2 buffers for low latency
    chan_cfg.dma_frame_num = DMA_FRAMES;   // 128 frames = ~2.67ms per buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_APLL,
            .mclk_multiple = I2S_MCLK_MULTIPLE_384,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_OUT_BITS, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_BCLK,
            .ws = PIN_WS,
            .dout = PIN_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = { false, false, false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx));
    ESP_LOGI(TAG, "I2S TX up at %d Hz", SAMPLE_RATE);
}

// ---- Wi-Fi + ESP-NOW setup ----
static void init_wifi_espnow(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    ESP_LOGI(TAG, "ESP-NOW ready on channel %d", WIFI_CHANNEL);
}

// ---- Playback task ----
static int32_t last_pcm[MAX_PAYLOAD * CHANNELS];  // frame used for PLC replay
static int last_pcm_bytes = 0;                    // bytes in last_pcm

static void playback_task(void *arg) {
    playback_task_handle = xTaskGetCurrentTaskHandle();
    init_i2s();
    ESP_ERROR_CHECK(i2s_channel_disable(i2s_tx));
    
    audio_packet_t pkt;
    int32_t pcm[MAX_PAYLOAD * CHANNELS];  // Current decoded frame
    int32_t out_pcm[MAX_PAYLOAD * CHANNELS];
    int32_t pending_pcm[MAX_PAYLOAD * CHANNELS];
    int32_t i2s_pcm[MAX_PAYLOAD * CHANNELS];
    size_t written;
    uint32_t consecutive_plc = 0;
    bool has_pending = false;
    bool stream_active = false;
    bool i2s_enabled = false;
    
    ESP_LOGI(TAG, "Playback ready, waiting for packets...");
    ESP_LOGI(TAG, "Timing: frame~%luus (%d samples), dma~%luus, prebuf=%d",
             (unsigned long)FRAME_TIME_US, SAMPLES_PER_FRAME, (unsigned long)DMA_LATENCY_US, PREBUFFER_FRAMES);
    
    while (1) {
        uint32_t now_us = (uint32_t)esp_timer_get_time();

        // Source timed out: stop output and reset pipeline right away.
        if (stream_active && (uint32_t)(now_us - last_rx_us) > SOURCE_TIMEOUT_US) {
            ESP_LOGW(TAG, "Source timeout, stopping I2S and clearing buffers");
            if (i2s_enabled) {
                i2s_channel_disable(i2s_tx);
                i2s_enabled = false;
            }
            ring_clear();
            memset(last_pcm, 0, sizeof(last_pcm));
            memset(pending_pcm, 0, sizeof(pending_pcm));
            last_pcm_bytes = BYTES_PER_FRAME;
            has_pending = false;
            consecutive_plc = 0;
            stream_active = false;
            clock_sync_updates = 0;
            last_play_lag_us = 0;
            adaptive_target_fill = TARGET_BUF_FILL;
            adaptive_packet_wait_ms = PACKET_WAIT_MS;
            continue;
        }

        if (!stream_active) {
            if (ring_count_get() < PREBUFFER_FRAMES) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
                continue;
            }

            // Startup trim: keep only the newest packets before unmuting.
            while (ring_count_get() > adaptive_target_fill) {
                audio_packet_t drop;
                if (!ring_pop(&drop)) {
                    break;
                }
                rx_dropped++;
                sync_drops++;
            }

            if (!i2s_enabled) {
                i2s_channel_enable(i2s_tx);
                i2s_enabled = true;
            }

            ESP_LOGI(TAG, "Playback started with %d frames buffered", ring_count_get());

            if (ring_pop(&pkt)) {
                int num_samples = 0;
                adpcm_decode_frame(&pkt, last_pcm, &num_samples);

                if (num_samples < SAMPLES_PER_FRAME) {
                    int32_t fill_l = 0;
                    int32_t fill_r = 0;
                    if (num_samples > 0) {
                        fill_l = last_pcm[(num_samples - 1) * 2];
                        fill_r = last_pcm[(num_samples - 1) * 2 + 1];
                    }
                    for (int i = num_samples; i < SAMPLES_PER_FRAME; i++) {
                        last_pcm[i * 2] = fill_l;
                        last_pcm[i * 2 + 1] = fill_r;
                    }
                }

                last_pcm_bytes = BYTES_PER_FRAME;
                memcpy(out_pcm, last_pcm, last_pcm_bytes);
                apply_output_gain(out_pcm, SAMPLES_PER_FRAME);
                pcm24_to_i2s24(out_pcm, i2s_pcm, SAMPLES_PER_FRAME);
                i2s_channel_write(i2s_tx, i2s_pcm, I2S_BYTES_PER_FRAME, &written, pdMS_TO_TICKS(20));
                played++;
                stream_active = true;
                consecutive_plc = 0;
            }
            continue;
        }

        bool rendered_packet = false;
        bool got_packet = false;

        // If sync logic held a packet, play it first.
        if (has_pending) {
            memcpy(pcm, pending_pcm, BYTES_PER_FRAME);
            has_pending = false;
            got_packet = true;
            rendered_packet = true;
        } else {
            // Try now, then wait a bit before falling back to PLC.
            got_packet = ring_pop(&pkt);
            if (!got_packet) {
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(adaptive_packet_wait_ms));
                got_packet = ring_pop(&pkt);
            }
        }
        
        int pcm_bytes;
        
        if (got_packet && !rendered_packet) {
            // Decode packet.
            int num_samples = 0;
            adpcm_decode_frame(&pkt, pcm, &num_samples);

            if (num_samples < SAMPLES_PER_FRAME) {
                int32_t fill_l = 0;
                int32_t fill_r = 0;
                if (num_samples > 0) {
                    fill_l = pcm[(num_samples - 1) * 2];
                    fill_r = pcm[(num_samples - 1) * 2 + 1];
                }
                for (int i = num_samples; i < SAMPLES_PER_FRAME; i++) {
                    pcm[i * 2] = fill_l;
                    pcm[i * 2 + 1] = fill_r;
                }
            }

            pcm_bytes = BYTES_PER_FRAME;

            // Blend back after PLC burst to avoid clicks.
            if (consecutive_plc > 0) {
                int fade_samples = (CROSSFADE_SAMPLES < SAMPLES_PER_FRAME) ? CROSSFADE_SAMPLES : SAMPLES_PER_FRAME;
                for (int i = 0; i < fade_samples; i++) {
                    int32_t t = (i + 1);
                    int32_t n = fade_samples;
                    int32_t l_from = last_pcm[i * 2];
                    int32_t r_from = last_pcm[i * 2 + 1];
                    int32_t l_to = pcm[i * 2];
                    int32_t r_to = pcm[i * 2 + 1];
                    pcm[i * 2] = (int32_t)(((n - t) * l_from + t * l_to) / n);
                    pcm[i * 2 + 1] = (int32_t)(((n - t) * r_from + t * r_to) / n);
                }
            }
            
            rendered_packet = true;
            consecutive_plc = 0;

            // Clock-sync catch-up: if lag is too high, drop one queued frame.
            if (clock_sync_updates > 10) {
                uint32_t now_us = (uint32_t)esp_timer_get_time();
                int32_t play_lag_us = (int32_t)(now_us - (pkt.src_t_us + (uint32_t)clock_offset_us));
                last_play_lag_us = play_lag_us;

                // Too late, drop one queued frame.
                if (play_lag_us > MAX_PLAY_LAG_US && ring_count_get() > adaptive_target_fill) {
                    audio_packet_t drop;
                    if (ring_pop(&drop)) {
                        rx_dropped++;
                        sync_drops++;
                    }
                }

                // Keep steady-state latency near target.
                if (play_lag_us > TARGET_PLAY_LAG_US && ring_count_get() > (adaptive_target_fill + 1)) {
                    audio_packet_t drop;
                    if (ring_pop(&drop)) {
                        rx_dropped++;
                        sync_drops++;
                    }
                }

                // No early-hold in ultra-low-latency mode.
            }
        } else if (got_packet && rendered_packet) {
            pcm_bytes = BYTES_PER_FRAME;
            consecutive_plc = 0;
        } else {
            // PLC path: replay last frame.
            memcpy(pcm, last_pcm, last_pcm_bytes);
            pcm_bytes = last_pcm_bytes;
            underruns++;
            consecutive_plc++;
        }

        // Blocking I2S write is the timing master.
        memcpy(out_pcm, pcm, pcm_bytes);
        apply_output_gain(out_pcm, SAMPLES_PER_FRAME);
        pcm24_to_i2s24(out_pcm, i2s_pcm, SAMPLES_PER_FRAME);
        esp_err_t err = i2s_channel_write(i2s_tx, i2s_pcm, I2S_BYTES_PER_FRAME, &written, pdMS_TO_TICKS(20));
        if (err != ESP_OK || written != I2S_BYTES_PER_FRAME) {
            // Not expected with this timeout.
        }

        // Save last rendered frame for PLC/hold continuity.
        memcpy(last_pcm, pcm, pcm_bytes);
        last_pcm_bytes = pcm_bytes;
        if (rendered_packet) {
            played++;
        }

    }
}

// ---- Runtime stats ----
static void status_task(void *arg) {
    uint32_t last_rx = 0, last_played = 0, last_underruns = 0;
    uint8_t stable_windows = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t rx_rate = (rx_count - last_rx) / 5;
        uint32_t play_rate = (played - last_played) / 5;
        uint32_t underrun_delta = underruns - last_underruns;
        last_rx = rx_count;
        last_played = played;
        last_underruns = underruns;
        uint8_t qfill = ring_count_get();

        uint8_t new_target = adaptive_target_fill;
        uint8_t new_wait = adaptive_packet_wait_ms;
        if (underrun_delta >= 2) {
            new_target = TARGET_BUF_FILL_MAX;
            new_wait = PACKET_WAIT_MS_MAX;
            stable_windows = 0;
        } else if (underrun_delta == 0) {
            if (stable_windows < 3) {
                stable_windows++;
            }
            if (stable_windows >= 3) {
                new_target = TARGET_BUF_FILL;
                new_wait = PACKET_WAIT_MS;
            }
        } else {
            stable_windows = 0;
        }
        if (new_target != adaptive_target_fill || new_wait != adaptive_packet_wait_ms) {
            adaptive_target_fill = new_target;
            adaptive_packet_wait_ms = new_wait;
            ESP_LOGI(TAG, "Adaptive: tf=%u wait=%ums (und+%lu/5s)",
                     adaptive_target_fill, adaptive_packet_wait_ms, underrun_delta);
        }
        
        // Sink latency = queue + DMA buffering.
        uint32_t queue_latency_us = qfill * FRAME_TIME_US;
        uint32_t total_sink_latency_ms = (queue_latency_us + DMA_LATENCY_US) / 1000;
        
        ESP_LOGI(TAG, "rx=%lu(%lu/s) play=%lu(%lu/s) q=%u drop=%lu und=%lu lat=%lums lag=%ldus off=%ldus jit=%ldus sdrop=%lu shold=%lu tf=%u wait=%ums",
                 rx_count, rx_rate, played, play_rate, qfill, rx_dropped, underruns,
                 total_sink_latency_ms, last_play_lag_us, clock_drift_us, clock_jitter_us,
                 sync_drops, sync_holds, adaptive_target_fill, adaptive_packet_wait_ms);
    }
}

// ---- App entry ----
void app_main(void) {
    ESP_LOGI(TAG, "ESP-NOW ADPCM sink online");
    ESP_LOGI(TAG, "Max payload: %d, Ring: %d, Prebuffer: %d, Target fill: %d",
             MAX_PAYLOAD, PACKET_RING_SIZE, PREBUFFER_FRAMES, TARGET_BUF_FILL);
    
    // Start with silent PLC buffer.
    memset(last_pcm, 0, sizeof(last_pcm));
    last_pcm_bytes = BYTES_PER_FRAME;
    memset(packet_ring, 0, sizeof(packet_ring));
    ring_head = 0;
    ring_tail = 0;
    ring_count = 0;
    
    init_wifi_espnow();
    
    // Keep playback off Wi-Fi core to reduce underruns.
    xTaskCreatePinnedToCore(playback_task, "playback", 8192, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 1, NULL, 0);
}
