#include <string.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "rmt_uart.h"

static const char *TAG = "RMT";

typedef struct {
    rmt_tx_channel_config_t channel_config;
    rmt_channel_handle_t channel;
    rmt_symbol_word_t *symbols;
    int item_index;
} rmt_uart_context_tx_t;

typedef struct {
    rmt_rx_channel_config_t channel_config;
    rmt_channel_handle_t channel;
    rmt_receive_config_t receive_config;
    rmt_symbol_word_t *symbols;
    uint8_t* bytes;
    int byte_num;
    int bit_num;
    uint16_t raw_data;
    QueueHandle_t queue;
} rmt_uart_context_rx_t;

typedef struct {
    rmt_uart_config_t uart_config;
    rmt_uart_context_tx_t uart_context_tx;
    rmt_uart_context_rx_t uart_context_rx;
    uint16_t bit_ticks;
    bool configured;
} rmt_uart_context_t;

static rmt_uart_context_t rmt_uart_contexts[RMT_UART_NUM_MAX] = {0};

static int convert_byte_to_symbols(rmt_uart_context_t* ctx, uint16_t byte)
{
    rmt_uart_context_tx_t* rtc = &ctx->uart_context_tx;
    size_t total_bits = ctx->uart_config.data_bits == RMT_UART_DATA_8_BITS ? 9 : 11;
    uint16_t stop_bit = ctx->uart_config.data_bits == RMT_UART_DATA_8_BITS ? 0x200 : 0xC00;
    uint16_t data = (byte << 1) | stop_bit;
    ESP_LOGD("rmt convert","total_bits:%d, stop_bit:%d, data: %.4X", total_bits, stop_bit, data);

    for (int i = 0; i < total_bits; i += 2) {
        rmt_symbol_word_t* symbol = &rtc->symbols[rtc->item_index];
        symbol->duration0 = ctx->bit_ticks;
        symbol->duration1 = ctx->bit_ticks;
        symbol->level0 = (data >> i) ^ 1;
        symbol->level1 = (data >> (i + 1)) ^ 1;
        rtc->item_index++;
        if (rtc->item_index >= ctx->uart_config.buffer_size / sizeof(rmt_symbol_word_t)) {
            ESP_LOGE(TAG, "DATA TOO LONG - increase tx_items_buffer_size");
            return -1;
        }
        ESP_LOGD(TAG, "\trmt tx item %02d: dur0: %d lvl0: %d  dur1: %d lvl1: %d",
            rtc->item_index, symbol->duration0, symbol->level0, symbol->duration1, symbol->level1);
    }
    return 0;
}

static int convert_data_to_symbols(rmt_uart_context_t* ctx, const uint8_t* data, uint16_t len)
{
    rmt_uart_context_tx_t* rtc = &ctx->uart_context_tx;
    rtc->item_index = 0;
    
    for (int i = 0; i < len; ++i) {
        if (ctx->uart_config.data_bits == RMT_UART_DATA_9_BITS) {
            if (convert_byte_to_symbols(ctx, (data[i] << 8)|data[i + 1])) return -1;
            i++;
        } else {
            if (convert_byte_to_symbols(ctx, (uint16_t)data[i])) return -1;
        }
    }
    return rtc->item_index;
}

static bool IRAM_ATTR rmt_rx_done_cb(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t higher_prio_woken = pdFALSE;
    rmt_uart_context_rx_t* uart_context_rx = (rmt_uart_context_rx_t *)user_data;
    xQueueSendFromISR(uart_context_rx->queue, &edata->num_symbols, &higher_prio_woken);
    return higher_prio_woken == pdTRUE;
}

esp_err_t rmt_uart_init(uint8_t uart_num, const rmt_uart_config_t* uart_config)
{
    ESP_RETURN_ON_FALSE((uart_num < RMT_UART_NUM_MAX), ESP_FAIL, TAG, "uart_num error");
    ESP_RETURN_ON_FALSE((uart_config), ESP_FAIL, TAG, "uart_config error");
    ESP_RETURN_ON_FALSE((uart_config->baud_rate >= 1200), ESP_FAIL, TAG, "baud_rate too low");
    ESP_RETURN_ON_FALSE((uart_config->baud_rate <= 460800), ESP_FAIL, TAG, "baud_rate too high");

    int baud_rate = uart_config->baud_rate;
    int bit_ticks = 21;
    if (baud_rate < 2400) bit_ticks = 250;
    else if (baud_rate < 4800) bit_ticks = 125;
    else if (baud_rate < 9600) bit_ticks = 62;
    else if (baud_rate < 57600) bit_ticks = 52;
    else if (baud_rate < 230400) bit_ticks = 23;
    else if (baud_rate < 460800) bit_ticks = 19;
    const int RMT_RES_HZ = uart_config->baud_rate * bit_ticks;
    // ns per tick
    const int RMT_NS_PER_TICK = (int)(1000000000UL / (uint64_t)RMT_RES_HZ);

    rmt_uart_context_t* ctx = &rmt_uart_contexts[uart_num];

    ctx->bit_ticks = bit_ticks;
    memcpy(&ctx->uart_config, uart_config, sizeof(rmt_uart_config_t));

    if (uart_config->mode == RMT_UART_MODE_RX_ONLY || uart_config->mode == RMT_UART_MODE_TX_RX) {
        rmt_channel_handle_t rx_channel = NULL;
        rmt_rx_channel_config_t rx_chan_cfg = {
            .gpio_num = uart_config->rx_io_num,
            .clk_src = RMT_CLK_SRC_XTAL,  // 40 MHz
            .resolution_hz = RMT_RES_HZ,
            .mem_block_symbols = (uart_config->mode == RMT_UART_MODE_RX_ONLY) ? SOC_RMT_MEM_WORDS_PER_CHANNEL : (SOC_RMT_MEM_WORDS_PER_CHANNEL << 1),
            .intr_priority = 0,
        };

        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_cfg, &rx_channel));

        // Store for later use (replace your rmt_uart_contex[uart_num].rmt_config_rx)
        rmt_uart_context_rx_t* uart_context_rx = &ctx->uart_context_rx;
        uart_context_rx->channel = rx_channel;

        // 2. Enable the channel
        ESP_ERROR_CHECK(rmt_enable(rx_channel));

        // 5. Start continuous reception with idle timeout and filter
        // This replaces rx_config.idle_threshold and filter settings
        uart_context_rx->receive_config.signal_range_min_ns = RMT_NS_PER_TICK / 20;  // ticks → ns (resolution_hz gives ns per tick)
        uart_context_rx->receive_config.signal_range_max_ns = 10 * RMT_NS_PER_TICK * bit_ticks;   // 10 bits idle → frame end
        uart_context_rx->symbols = calloc(uart_config->buffer_size / sizeof(rmt_symbol_word_t) * 2, sizeof(rmt_symbol_word_t));
        uart_context_rx->queue = xQueueCreate(32, sizeof(size_t));

        rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = rmt_rx_done_cb,
        };
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_channel, &cbs, (void *)uart_context_rx));
        ESP_ERROR_CHECK(rmt_receive(rx_channel, uart_context_rx->symbols, uart_config->buffer_size * 2, &uart_context_rx->receive_config));
    }

    if (uart_config->mode == RMT_UART_MODE_TX_ONLY || uart_config->mode == RMT_UART_MODE_TX_RX) {
        rmt_channel_handle_t tx_channel = NULL;
        rmt_tx_channel_config_t tx_chan_cfg = {
            .gpio_num = uart_config->tx_io_num,
            .clk_src = RMT_CLK_SRC_XTAL,  // 40 MHz
            .resolution_hz = RMT_RES_HZ,
            .mem_block_symbols = (uart_config->mode == RMT_UART_MODE_RX_ONLY) ? SOC_RMT_MEM_WORDS_PER_CHANNEL : (SOC_RMT_MEM_WORDS_PER_CHANNEL << 1),
            .intr_priority = 0,
        };

        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &tx_channel));

        // Store for later use (replace your rmt_uart_contex[uart_num].rmt_config_rx)
        ctx->uart_context_tx.channel = tx_channel;

        // 2. Enable the channel
        ESP_ERROR_CHECK(rmt_enable(tx_channel));

        // 4. Allocate TX items buffer if needed (unchanged — this is not RMT-related)
        #if CONFIG_SPIRAM_USE_MALLOC
        ctx->uart_contex_tx.symbols = heap_caps_calloc(uart_config->buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT, sizeof(rmt_symbol_word_t));
        #else
        ctx->uart_context_tx.symbols = calloc(uart_config->buffer_size, sizeof(rmt_symbol_word_t));
        #endif
    }
    ctx->configured = true;
    return ESP_OK;
}

esp_err_t rmt_uart_write(uint8_t uart_num, const uint8_t* data, size_t size)
{
    rmt_uart_context_t* ctx = &rmt_uart_contexts[uart_num];
    ESP_RETURN_ON_FALSE((ctx->configured), ESP_FAIL, TAG, "uart not configured");
    ESP_RETURN_ON_FALSE((ctx->uart_config.mode != RMT_UART_MODE_RX_ONLY), ESP_FAIL, TAG, "uart RX only");
    rmt_uart_context_tx_t* rtc = &ctx->uart_context_tx;
    if (convert_data_to_symbols(ctx, data, size) < 0) return ESP_FAIL;

    // Create copy encoder (copies raw symbols directly)
    rmt_copy_encoder_config_t copy_encoder_cfg = {};
    rmt_encoder_handle_t copy_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_cfg, &copy_encoder));

    // Transmit (blocking)
    rmt_channel_handle_t tx_channel = ctx->uart_context_tx.channel;
    rmt_symbol_word_t *symbols = rtc->symbols;
    ESP_ERROR_CHECK(rmt_transmit(tx_channel, copy_encoder, symbols, sizeof(rmt_symbol_word_t), NULL));

    // Optional: Wait for completion
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channel, portMAX_DELAY));
    return 0;
}

int rmt_uart_read(uint8_t uart_num, uint8_t* buf, size_t buf_size, TickType_t timeout)
{
    rmt_uart_context_t* ctx = &rmt_uart_contexts[uart_num];
    ESP_RETURN_ON_FALSE((ctx->configured), ESP_FAIL, TAG, "uart not configured");
    ESP_RETURN_ON_FALSE((ctx->uart_config.mode != RMT_UART_MODE_TX_ONLY), ESP_FAIL, TAG, "uart TX only");
    rmt_uart_context_rx_t* uart_context_rx = &ctx->uart_context_rx;
    size_t num_symbols = 0;
    if (xQueueReceive(uart_context_rx->queue, &num_symbols, timeout) != pdTRUE) {
        return 0;  // Timeout
    }

    // Simple 8N1 decoder (LSB first, idle high)
    uint8_t current_byte = 0;
    int bit_count = 0;
    bool in_frame = false;
    int16_t bit_ticks = (int16_t)ctx->bit_ticks;
    int16_t bit_ticks_tol = bit_ticks >> 2;
    int16_t bit_ticks_min = bit_ticks - bit_ticks_tol;
    uint8_t byte_cnt = 0;
    //ESP_LOGI(TAG, "num_symbols=%d bit_ticks=%d tolerance_ticks=%d", num_symbols, bit_ticks, bit_ticks_tol);

    for (size_t i = 0; i < num_symbols && buf_size > 0; ++i) {
        rmt_symbol_word_t s = uart_context_rx->symbols[i];
        uint16_t duration = s.duration0;
        // Process low then high parts
        //ESP_LOGI(TAG, "i=%d lvl0=%d dur0=%d lvl1=%d dur1=%d in_frame=%d bit_count=%d", i, s.level0, s.duration0, s.level1, s.duration1, in_frame, bit_count);
        if (!in_frame && s.level0 == 0 && duration > bit_ticks_min) {  // Start bit?
            in_frame = true;
            bit_count = 0;
            current_byte = 0;
            if (duration >= bit_ticks) duration -= bit_ticks;
            else duration = 0;
        }
        if (in_frame) {
            if (s.level0 != 0) {
                // Invalid symbol received
                in_frame = false;
                ESP_LOGE(TAG, "INVALID SYMBOL");
                continue;
            }
            while (duration > bit_ticks_min) {
                bit_count++;
                if (duration >= bit_ticks) duration -= bit_ticks;
                else duration = 0;
                //ESP_LOGI(TAG, "0 bit_cnt=%d dur=%d", bit_count, duration);
            }
            if (bit_count > 8) {
                // Invalid symbol received (STOP bit is missing)
                in_frame = false;
                ESP_LOGE(TAG, "NO STOP bit_count=%d duration=%d RX=%02x", bit_count, duration, current_byte);
                continue;
            }
            duration = s.duration1;
            while (duration > bit_ticks_min) {
                if (bit_count == 8) {
                    *buf++ = current_byte;
                    buf_size--;
                    byte_cnt++;
                    in_frame = false;
                    break;
                }
                current_byte |= (1 << bit_count);
                bit_count++;
                if (duration >= bit_ticks) duration -= bit_ticks;
                else duration = 0;
                //ESP_LOGI(TAG, "1 bit_cnt=%d dur=%d", bit_count, duration);
            }
        }
    }
    if (in_frame) {
        while (bit_count < 8) {
            current_byte |= (1 << bit_count);
            bit_count++;
        }
        *buf++ = current_byte;
        byte_cnt++;
    }

    ESP_ERROR_CHECK(rmt_receive(uart_context_rx->channel, uart_context_rx->symbols, ctx->uart_config.buffer_size * 2, &uart_context_rx->receive_config));
    return byte_cnt;
}

esp_err_t rmt_uart_deinit(uint8_t uart_num)
{
    rmt_uart_context_t* ctx = &rmt_uart_contexts[uart_num];
    ESP_RETURN_ON_FALSE((ctx->configured), ESP_FAIL, TAG, "uart not configured");
    esp_err_t ret = ESP_OK;

    if (ctx->uart_context_rx.channel != NULL) {
        ESP_ERROR_CHECK(rmt_disable(ctx->uart_context_rx.channel));
        // Delete the channel - this frees all resources
        ESP_ERROR_CHECK(rmt_del_channel(ctx->uart_context_rx.channel));
        ctx->uart_context_rx.channel = NULL;
    }
    if (ctx->uart_config.mode != RMT_UART_MODE_RX_ONLY) {
        rmt_uart_context_tx_t* rtc = &ctx->uart_context_tx;
#if CONFIG_SPIRAM_USE_MALLOC
        heap_caps_free(rtc->symbols);
#else
        free(rtc->symbols);
#endif
        // For a TX channel (similar)
        if (ctx->uart_context_tx.channel != NULL) {
            ESP_ERROR_CHECK(rmt_disable(ctx->uart_context_tx.channel));
            ESP_ERROR_CHECK(rmt_del_channel(ctx->uart_context_tx.channel));
            ctx->uart_context_tx.channel = NULL;
        }
    }
    ctx->configured = false;
    return ret;
}
