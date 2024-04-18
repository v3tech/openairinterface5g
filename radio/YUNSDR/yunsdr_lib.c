/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/** yunsdr_lib.c
 *
 * Author: eric
 * base on bladerf_lib.c
 */

#pragma GCC optimize(3, "Ofast", "inline", "fast-math", "unroll-loops")
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "system.h"
#include <math.h>
#include "yunsdr_lib.h"
#include "common/utils/LOG/log.h"

/** @addtogroup _YUNSDR_PHY_RF_INTERFACE_
 * @{
 */
#include "openair1/PHY/sse_intrin.h"
#if defined (__AVX512BW__)
#include <simde/x86/avx512.h>
#endif

extern int usrp_tx_thread;

//! Number of YUNSDR devices
int num_devices = 0;
static bool running = false;

#define BUFFER_SIZE    (122880 * 100 * sizeof(int))
#define NCHAN_PER_DEV  4
static void *cache_buf[NCHAN_PER_DEV];
static void *iq_buf[NCHAN_PER_DEV];
static uint32_t remain = 0;
static uint32_t RX_MTU = 30720;
static uint8_t shift = 2;

#define REMOVE_SUBSTRING_WITHCOMAS(S, TOREMOVE)                                                                        \
  remove_substring(S, TOREMOVE ",");                                                                                   \
  remove_substring(S, TOREMOVE ", ");                                                                                  \
  remove_substring(S, "," TOREMOVE);                                                                                   \
  remove_substring(S, ", " TOREMOVE);                                                                                  \
  remove_substring(S, TOREMOVE)

static inline void remove_substring(char* s, const char* toremove)
{
  while ((s = strstr(s, toremove))) {
    memmove(s, s + strlen(toremove), 1 + strlen(s + strlen(toremove)));
  }
}

static inline void copy_subdev_string(char* dst, char* src)
{
  int n   = 0;
  int len = (int)strlen(src);
  /* Copy until end of string or comma */
  while (n < len && src[n] != '\0' && src[n] != ',') {
    dst[n] = src[n];
    n++;
  }
  dst[n] = '\0';
}

static inline int channel_to_mask(int nchan)
{
    assert(nchan > 0 && nchan < 8);

    return (1 << nchan) - 1;
}

static yunsdr_cfg_result_t yunsdr_get_config(YUNSDRBoardType model, uint64_t sample_rate)
{
    yunsdr_cfg_result_t result = {0, -1, false};

    if (model == UNKNOWN || sample_rate == 0) {
        return result;
    }

    const yunsdr_cfg_table_t *cfg = NULL;
    for (size_t i = 0; g_board_tables[i].model != UNKNOWN; i++) {
        if (g_board_tables[i].model == model) {
            cfg = &g_board_tables[i];
            break;
        }
    }
    if (cfg == NULL || cfg->items == NULL) return result;

    for (size_t i = 0;; i++) {
        const yunsdr_cfg_item_t *item = &cfg->items[i];
        if (item->sample_rate == 0 && item->bandwidth == 0 && item->tx_adv == -1) break;
        if (item->sample_rate == sample_rate) {
            result.bandwidth = item->bandwidth;
            result.tx_adv = item->tx_adv;
            result.success = true;
            LOG_I(HW, "[yunsdr] Device: %s, TX advance: %d\n", cfg->name, item->tx_adv);
            return result;
        }
    }

    return result;
}

static inline void rx_iq_shift(int16_t *dst, int16_t *src, ssize_t len, uint8_t shift)
{
    int16_t *iq       = src;
    int16_t *iq_scale = dst;
#if defined(__x86_64__) || defined(__i386__)
#if defined(__AVX512BW__)
    simde__m512i a, *b;

    while (len >= 32) {
        if ((((uintptr_t)iq) & 0x3F) == 0) {
            a  = *(simde__m512i *)&iq[0];
            b  = (simde__m512i *)&iq_scale[0];
            *b = simde_mm512_srai_epi16(a, shift);
        } else {
            simde__m512i tmp = simde_mm512_loadu_si512((simde__m512i *)&iq[0]);
            tmp = simde_mm512_srai_epi16(tmp, shift);
            simde_mm512_storeu_si512((simde__m512i *)&iq_scale[0], tmp);
        }
        iq += 32;
        iq_scale += 32;
        len -= 32;
    }
#else
    simde__m256i a, *b;

    while (len >= 16) {
        if (((((uintptr_t)iq) & 0x1F) == 0)
        && ((((uintptr_t)iq_scale) & 0x1F) == 0)) {
            a  = *(simde__m256i *)&iq[0];
            b  = (simde__m256i *)&iq_scale[0];
            *b = simde_mm256_srai_epi16(a, shift);
        } else {
            simde__m256i tmp = simde_mm256_loadu_si256((simde__m256i *)&iq[0]);
            tmp = simde_mm256_srai_epi16(tmp, shift);
            simde_mm256_storeu_si256((simde__m256i *)&iq_scale[0], tmp);
        }
        iq += 16;
        iq_scale += 16;
        len -= 16;
    }
#endif
#elif defined(__arm__) || defined(__aarch64__)
    int16x8_t a, *b;

    while (len >= 8) {
        a = *(int16x8_t *)&iq[0];
        b = (int16x8_t *)&iq_scale[0];
        *b = vshrq_n_s16(a, shift);
        iq += 8;
        iq_scale += 8;
        len -= 8;
    }
#endif
    /* remaining data */
    while (len != 0) {
        iq_scale[0] = iq[0] >> shift;
        iq++;
        iq_scale++;
        len--;
    }
}

/*! \brief Start yunsdr
 * \param device the hardware to use
 * \returns 0 on success
 */
int trx_yunsdr_start(openair0_device_t *device)
{
    yunsdr_state_t *yunsdr = (yunsdr_state_t *)device->priv;

    yunsdr->rx_current_ts = 0;
    LOG_I(HW, "[yunsdr] Start yunsdr ...\n");
    running = true;

    return 0;
}

/*! \brief Called to send samples to the yunsdr RF target
  \param device pointer to the device structure specific to the RF hardware target
  \param timestamp The timestamp at whicch the first sample MUST be sent
  \param buff Buffer which holds the samples
  \param nsamps number of samples to be sent
  \param cc index of the component carrier
  \param flags Ignored for the moment
  \returns 0 on success
  */
static int trx_yunsdr_write(openair0_device_t *device,openair0_timestamp_t timestamp, void **buff, int nsamps, int cc, int flags)
{
    int status;
    yunsdr_state_t *yunsdr = (yunsdr_state_t*)device->priv;
    timestamp -= device->openair0_cfg->command_line_sample_advance + device->openair0_cfg->tx_sample_advance;

    uint32_t trx_flags = 0;
    radio_tx_burst_flag_t flags_burst = (radio_tx_burst_flag_t) (flags & 0xf);

    int end;
    openair0_thread_t *write_thread = &device->write_thread;
    openair0_write_package_t *write_package = write_thread->write_package;
    bool first_packet_state = false, last_packet_state = false;

    if (flags_burst == TX_BURST_START) {
        trx_flags = 0;
        first_packet_state = true;
        last_packet_state  = false;
    } else if (flags_burst == TX_BURST_END) {
        trx_flags = 1;
        first_packet_state = false;
        last_packet_state  = true;
    } else if (flags_burst == TX_BURST_START_AND_END) {
        trx_flags = 1;
        first_packet_state = true;
        last_packet_state  = true;
    } else if (flags_burst == TX_BURST_MIDDLE) {
        trx_flags = 0;
        first_packet_state = false;
        last_packet_state  = false;
    } else if (flags_burst == 10) { // fail safe mode
        first_packet_state = false;
        last_packet_state  = true;
    }

    if (usrp_tx_thread == 0) {
        for(int i = 0; i < cc; i++) {
            int len = nsamps * 2;
            int16_t *iq = buff[i];
#if defined(__x86_64) || defined(__i386__)
#if defined(__AVX512BW__)
            simde__m512i a, *b;

            while (len >= 32) {
                if ((((uintptr_t)iq) & 0x3F) == 0) {
                    a = *(simde__m512i *)&iq[0];
                    b = (simde__m512i *)&iq[0];
                    *b = simde_mm512_slli_epi16(a, 4);
                } else {
                    simde__m512i tmp = simde_mm512_loadu_si512((simde__m512i *)&iq[0]);
                    tmp = simde_mm512_slli_epi16(tmp, 4);
                    simde_mm512_storeu_si512((simde__m512i *)&iq[0], tmp);
                }
                iq += 32;
                len -= 32;
            }
#else
            simde__m256i a, *b;

            while (len >= 16) {
                if ((((uintptr_t)iq) & 0x1F) == 0) {
                    a = *(simde__m256i *)&iq[0];
                    b = (simde__m256i *)&iq[0];
                    *b = simde_mm256_slli_epi16(a, 4);
                } else {
                    simde__m256i tmp = simde_mm256_loadu_si256((simde__m256i *)&iq[0]);
                    tmp = simde_mm256_slli_epi16(tmp, 4);
                    simde_mm256_storeu_si256((simde__m256i *)&iq[0], tmp);
                }
                iq += 16;
                len -= 16;
            }
#endif
#elif defined(__arm__) || defined(__aarch64__)
            int16x8_t a, *b;

            while (len >= 8) {
                a = *(int16x8_t *)&iq[0];
                b = (int16x8_t *)&iq[0];
                *b = vshlq_n_s16(a, 4);
                iq += 8;
                len -= 8;
            }
#endif
            /* remaining data */
            while (len != 0) {
                iq[0] <<= 4;
                iq++;
                len--;
            }
        }

        status = yunsdr_write_samples_multiport(yunsdr->dev, (const void **)buff, nsamps, channel_to_mask(cc), timestamp, trx_flags);
        if (status < 0) {
            yunsdr->num_tx_errors++;
            LOG_E(HW, "[yunsdr] Failed to TX samples\n");
            exit(-1);
        }

        //LOG_D(HW, "Provided TX timestamp: %u, nsamps: %u\n", timestamp, nsamps);

        yunsdr->tx_current_ts = timestamp;
        yunsdr->tx_nsamps += nsamps;
        yunsdr->tx_count++;

        return nsamps;
    } else {
        pthread_mutex_lock(&write_thread->mutex_write);

        if (write_thread->count_write >= MAX_WRITE_THREAD_PACKAGE) {
            LOG_W(HW,
                    "Buffer overflow, count_write = %d, start = %d end = %d, resetting write package\n",
                    write_thread->count_write,
                    write_thread->start,
                    write_thread->end);
            write_thread->end = write_thread->start;
            write_thread->count_write = 0;
        }

        end = write_thread->end;
        write_package[end].timestamp = timestamp;
        write_package[end].nsamps = nsamps;
        write_package[end].cc = cc;
        write_package[end].first_packet = first_packet_state;
        write_package[end].last_packet = last_packet_state;
        for (int i = 0; i < cc; i++)
            write_package[end].buff[i] = buff[i];
        write_thread->count_write++;
        write_thread->end = (write_thread->end + 1) % MAX_WRITE_THREAD_PACKAGE;
        LOG_D(HW, "Signaling TX TS %llu\n", (unsigned long long)timestamp);
        pthread_cond_signal(&write_thread->cond_write);
        pthread_mutex_unlock(&write_thread->mutex_write);

        return nsamps;
    }
}

void *trx_yunsdr_write_thread(void *arg)
{
    int ret = 0;
    openair0_device_t *device = (openair0_device_t *)arg;
    openair0_thread_t *write_thread = &device->write_thread;
    openair0_write_package_t *write_package = write_thread->write_package;

    yunsdr_state_t *s;
    int start;
    openair0_timestamp_t timestamp;
    void               **buff;
    int                nsamps;
    int                cc;
    unsigned int       trx_flags;

    while(1) {
        pthread_mutex_lock(&write_thread->mutex_write);
        while (write_thread->count_write == 0) {
            pthread_cond_wait(&write_thread->cond_write,&write_thread->mutex_write); // this unlocks mutex_rxtx while waiting and then locks it again
        }
        if (write_thread->write_thread_exit)
            break;
        s = (yunsdr_state_t *)device->priv;
        start = write_thread->start;
        timestamp    = write_package[start].timestamp;
        buff         = write_package[start].buff;
        nsamps       = write_package[start].nsamps;
        cc           = write_package[start].cc;
        write_thread->start = (write_thread->start + 1)% MAX_WRITE_THREAD_PACKAGE;
        write_thread->count_write--;
        pthread_mutex_unlock(&write_thread->mutex_write);

        for(int i = 0; i < cc; i++) {
            int len = nsamps * 2;
            int16_t *iq = buff[i];
#if defined(__x86_64) || defined(__i386__)
            simde__m256i a, *b;

            while (len >= 16) {
                if ((((uintptr_t)iq) & 0x1F) == 0) {
                    a = *(simde__m256i *)&iq[0];
                    b = (simde__m256i *)&iq[0];
                    *b = simde_mm256_slli_epi16(a, 4);
                } else {
                    simde__m256i tmp = simde_mm256_loadu_si256((simde__m256i *)&iq[0]);
                    tmp = simde_mm256_slli_epi16(tmp, 4);
                    simde_mm256_storeu_si256((simde__m256i *)&iq[0], tmp);
                }
                iq += 16;
                len -= 16;
            }
#elif defined(__arm__) || defined(__aarch64__)
            int16x8_t a, *b;

            while (len >= 8) {
                a = *(int16x8_t *)&iq[0];
                b = (int16x8_t *)&iq[0];
                *b = vshlq_n_s16(a, 4);
                iq += 8;
                len -= 8;
            }
#endif
            /* remaining data */
            while (len != 0) {
                iq[0] <<= 4;
                iq++;
                len--;
            }
        }

        if ((write_package[start].first_packet == false &&
                    write_package[start].last_packet == true) ||
                (write_package[start].first_packet == true &&
                 write_package[start].last_packet == true)) {
            trx_flags = 1;
        } else {
            trx_flags = 0;
        }

        ret = yunsdr_write_samples_multiport(s->dev, (const void **)buff, nsamps, channel_to_mask(cc), timestamp, trx_flags);
        if (ret < 0) {
            s->num_tx_errors++;
            LOG_E(HW, "[yunsdr] Failed to TX samples\n");
            exit(-1);
        }

        //LOG_D(HW, "Provided TX timestamp: %u, nsamps: %u\n", timestamp, nsamps);

        s->tx_current_ts = timestamp;
        s->tx_nsamps += nsamps;
        s->tx_count++;

    }

    return NULL;
}

int trx_yunsdr_write_init(openair0_device_t *device)
{
    openair0_thread_t *write_thread = &device->write_thread;
    printf("initializing tx write thread\n");

    write_thread->start              = 0;
    write_thread->end                = 0;
    write_thread->count_write        = 0;
    write_thread->write_thread_exit  = false;
    printf("end of tx write thread\n");
    pthread_mutex_init(&write_thread->mutex_write, NULL);
    pthread_cond_init(&write_thread->cond_write, NULL);
    threadCreate(&write_thread->pthread_write,
            trx_yunsdr_write_thread,
            (void *)device,
            (char*)"trx_yunsdr_write_thread",
            -1,
            OAI_PRIORITY_RT_MAX);
    return(0);
}

static void trx_yunsdr_write_reset(openair0_thread_t *wt)
{
    pthread_mutex_lock(&wt->mutex_write);
    wt->count_write = 1;
    wt->write_thread_exit = true;
    pthread_cond_signal(&wt->cond_write);
    pthread_mutex_unlock(&wt->mutex_write);
    void *retval = NULL;
    pthread_join(wt->pthread_write, &retval);
    LOG_I(HW, "stopped YUNSDR write thread\n");
}


/*! \brief Receive samples from hardware.
 * Read \ref nsamps samples from each channel to buffers. buff[0] is the array for
 * the first channel. *ptimestamp is the time at which the first sample
 * was received.
 * \param device the hardware to use
 * \param[out] ptimestamp the time at which the first sample was received.
 * \param[out] buff An array of pointers to buffers for received samples. The buffers must be large enough to hold the number of samples \ref nsamps.
 * \param nsamps Number of samples. One sample is 2 byte I + 2 byte Q => 4 byte.
 * \param cc  Index of component carrier
 * \returns number of samples read
 */
static int trx_yunsdr_read(openair0_device_t *device, openair0_timestamp_t *ptimestamp, void **buff, int nsamps, int cc)
{
    int status;
    yunsdr_state_t *yunsdr = (yunsdr_state_t *)device->priv;
    uint64_t timestamp = 0UL;

retry:
    if(remain == 0) {
        int recv = 0;
        timestamp = 0UL;
        if(nsamps % RX_MTU) {
            recv = (nsamps / RX_MTU + 1) * RX_MTU;
            status = yunsdr_read_samples_multiport(yunsdr->dev, iq_buf, recv, channel_to_mask(cc), &timestamp);
            if (status < 0) {
                LOG_E(HW, "[yunsdr] Failed to read samples %d\n", nsamps);
                yunsdr->num_rx_errors++;
                exit(-1);
            }
            for(int i = 0; i < cc; i++)
                rx_iq_shift((int16_t *)buff[i], (int16_t *)iq_buf[i], nsamps * 2, shift);
            if(recv > nsamps) {
                for(int i = 0; i < cc; i++)
                    memcpy(cache_buf[i], iq_buf[i] + nsamps * 4, (recv - nsamps) * 4);
                remain = recv - nsamps;
            }
        } else {
            recv = nsamps;
            status = yunsdr_read_samples_multiport(yunsdr->dev, buff, recv, channel_to_mask(cc), &timestamp);
            if (status < 0) {
                LOG_E(HW, "[yunsdr] Failed to read samples %d\n", nsamps);
                yunsdr->num_rx_errors++;
                exit(-1);
            }
            for (int i = 0; i < cc; i++)
                rx_iq_shift((int16_t *)buff[i], (int16_t *)buff[i], nsamps * 2, shift);
        }

        *(uint64_t *)ptimestamp = timestamp;
        yunsdr->rx_current_ts = timestamp + nsamps;
        //LOG_D(HW, "case 0: Current RX timestamp  %"PRIu64", hw ts %"PRIu64", nsamps %u, remain %u, recv: %u\n",  *ptimestamp, timestamp, nsamps, remain, recv);
    } else if(remain >= nsamps) {
        for(int i = 0; i < cc; i++)
            rx_iq_shift((int16_t *)buff[i], (int16_t *)cache_buf[i], nsamps * 2, shift);
        remain -= nsamps;
        if(remain > 0) {
            for(int i = 0; i < cc; i++)
                memmove(cache_buf[i], cache_buf[i] + nsamps * 4, remain * 4);
        }
        *(uint64_t *)ptimestamp = yunsdr->rx_current_ts;
        yunsdr->rx_current_ts += nsamps;
        //LOG_D(HW, "case 1: Current RX timestamp  %"PRIu64", nsamps %u, remain %u\n",  *ptimestamp, nsamps, remain);
    } else {
        int recv;
        if(remain + RX_MTU >= nsamps)
            recv = RX_MTU;
        else
            recv = (nsamps / RX_MTU + 1) * RX_MTU;
        timestamp = 0UL;
        void *tmp_ptr[4];
        for(int i = 0; i < cc; i++) {
            tmp_ptr[i] = cache_buf[i] + remain * 4;
        }
        status = yunsdr_read_samples_multiport(yunsdr->dev, tmp_ptr, recv, channel_to_mask(cc), &timestamp);
        if (status < 0) {
            LOG_E(HW, "[yunsdr] Failed to read samples %d\n", nsamps);
            yunsdr->num_rx_errors++;
            exit(-1);
        }
        if(timestamp != (yunsdr->rx_current_ts + remain)) {
            int overflow = timestamp - (yunsdr->rx_current_ts + remain);
            LOG_W(HW, "Rx overflow %u samples\n", overflow);
            //remain += overflow;
            remain = 0;
            goto retry;
        }
        remain += recv;
        for(int i = 0; i < cc; i++) {
            rx_iq_shift((int16_t *)buff[i], (int16_t *)cache_buf[i], nsamps * 2, shift);
            memmove(cache_buf[i], cache_buf[i] + nsamps * 4, (remain - nsamps) * 4);
        }
        remain -= nsamps;

        *(uint64_t *)ptimestamp = yunsdr->rx_current_ts;
        yunsdr->rx_current_ts += nsamps;
        //LOG_D(HW, "case 2: Current RX timestamp  %"PRIu64", hw ts %"PRIu64", nsamps %u, remain %u, recv: %u\n",  *ptimestamp, timestamp, nsamps, remain, recv);
    }

    //LOG_D(HW, "Current RX timestamp  %"PRIu64", nsamps %u\n",  *ptimestamp, nsamps);
    yunsdr->rx_nsamps += nsamps;
    yunsdr->rx_count++;

    //fixme:
    //if(device->openair0_cfg->nr_band >= 257)
      //  *(uint64_t *)ptimestamp -= device->openair0_cfg->tx_sample_advance;

    return nsamps;
}

/*! \brief Terminate operation of the yunsdr transceiver -- free all associated resources
 * \param device the hardware to use
 */
void trx_yunsdr_end(openair0_device_t *device)
{
    yunsdr_state_t *yunsdr = (yunsdr_state_t*)device->priv;

    if(!running)
        return;
    running = false;

    LOG_I(HW, ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
    for(int i = 0; i < yunsdr->tx_num_channels; i++) {
        uint32_t count = 0;
        yunsdr_get_channel_event(yunsdr->dev, TX_CHANNEL_TIMEOUT, i+1, &count);
        LOG_I(HW, "[yunsdr] TX%d Channel timeout: %u\n", i+1, count);
    }
    for(int i = 0; i < yunsdr->rx_num_channels; i++) {
        uint32_t count = 0;
        yunsdr_get_channel_event(yunsdr->dev, RX_CHANNEL_OVERFLOW, i+1, &count);
        LOG_I(HW, "[yunsdr] RX%d Channel overflow: %u\n", i+1, count);
    }
    LOG_I(HW, "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");

    if (usrp_tx_thread != 0)
        trx_yunsdr_write_reset(&device->write_thread);

    uint64_t timestamp = 0xFFAABBC0;
    int status = yunsdr_read_samples_multiport(yunsdr->dev, NULL, 0, 0x1, &timestamp);
    if (status < 0) {
        LOG_E(HW, "[yunsdr] Failed to reset rx stream.\n");
        exit(-1);
    }

    yunsdr_close_device(yunsdr->dev);

    return;
}

/*! \brief print the yunsdr statistics
 * \param device the hardware to use
 * \returns  0 on success
 */
int trx_yunsdr_get_stats(openair0_device_t* device)
{
    UNUSED(device);
    return 0;
}

/*! \brief Reset the yunsdr statistics
 * \param device the hardware to use
 * \returns  0 on success
 */
int trx_yunsdr_reset_stats(openair0_device_t* device)
{
    UNUSED(device);
    return 0;
}

/*! \brief Stop yunsdr
 * \param card the hardware to use
 * \returns 0 in success
 */
int trx_yunsdr_stop(openair0_device_t* device)
{
    UNUSED(device);
    //yunsdr_state_t *yunsdr = (yunsdr_state_t *)device->priv;
    return 0;
}

/*! \brief Set frequencies (TX/RX)
 * \param device the hardware to use
 * \param openair0_cfg RF frontend parameters set by application
 * \param dummy dummy variable not used
 * \returns 0 in success
 */
int trx_yunsdr_set_freq(openair0_device_t* device, openair0_config_t *openair0_cfg)
{
    int status;
    yunsdr_state_t *yunsdr = (yunsdr_state_t *)device->priv;

    if(yunsdr->tx_lo_freq != openair0_cfg->tx_freq[0]) {
        if ((status = yunsdr_set_tx_lo_freq(yunsdr->dev, 0, (uint64_t)(openair0_cfg->tx_freq[0]))) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set TX frequency\n");
        } else {
            LOG_I(HW, "[yunsdr] set TX frequency to %lu\n",(uint64_t)(openair0_cfg->tx_freq[0]));
            yunsdr->tx_lo_freq = openair0_cfg->tx_freq[0];
        }
    }

    if(yunsdr->rx_lo_freq != openair0_cfg->rx_freq[0]) {
        if ((status = yunsdr_set_rx_lo_freq(yunsdr->dev, 0, (uint64_t)(openair0_cfg->rx_freq[0]))) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set RX frequency\n");
        } else {
            LOG_I(HW, "[yunsdr] set RX frequency to %lu\n",(uint64_t)(openair0_cfg->rx_freq[0]));
            yunsdr->rx_lo_freq = openair0_cfg->rx_freq[0];
        }
    }

    return 0;
}

/*! \brief calibration table for YUNSDR */
rx_gain_calib_table_t calib_table_yunsdr[] = {
    {3500000000.0, 72.0},
    {2660000000.0, 72.0},
    {2300000000.0, 72.0},
    {1880000000.0, 72.0},
    {816000000.0,  72.0},
    {-1,0}
};

rx_gain_calib_table_t calib_table_none[] = {
    {3500000000.0, 0.0},
    {2660000000.0, 0.0},
    {2300000000.0, 0.0},
    {1880000000.0, 0.0},
    {816000000.0,  0.0},
    {-1,0}
};

/*! \brief set RX gain offset from calibration table
 * \param openair0_cfg RF frontend parameters set by application
 * \param chain_index RF chain ID
 */
void set_rx_gain_offset(openair0_config_t *openair0_cfg, int chain_index)
{
    int i = 0;
    // loop through calibration table to find best adjustment factor for RX frequency
    double min_diff = 6e9, diff;

    while (openair0_cfg->rx_gain_calib_table[i].freq > 0) {
        diff = fabs(openair0_cfg->rx_freq[chain_index] - openair0_cfg->rx_gain_calib_table[i].freq);
        printf("cal %d: freq %f, offset %f, diff %f\n",
                i,
                openair0_cfg->rx_gain_calib_table[i].freq,
                openair0_cfg->rx_gain_calib_table[i].offset, diff);
        if (min_diff > diff) {
            min_diff = diff;
            openair0_cfg->rx_gain_offset[chain_index] = openair0_cfg->rx_gain_calib_table[i].offset;
        }
        i++;
    }
}

/*! \brief Set Gains (TX/RX)
 * \param device the hardware to use
 * \param openair0_cfg openair0 Config structure
 * \returns 0 in success
 */
int trx_yunsdr_set_gains(openair0_device_t* device, openair0_config_t *openair0_cfg)
{
    int ret = 0;
    yunsdr_state_t *yunsdr = (yunsdr_state_t *)device->priv;

    if (openair0_cfg->rx_gain[0] > 65+openair0_cfg->rx_gain_offset[0]) {
        LOG_E(HW, "[yunsdr] Reduce RX Gain 0 by %f dB\n", openair0_cfg->rx_gain[0] - openair0_cfg->rx_gain_offset[0] - 65);
        return -1;
    }

    if ((ret = yunsdr_set_rx1_rf_gain(yunsdr->dev, 0, (uint32_t)(openair0_cfg->rx_gain[0] - openair0_cfg->rx_gain_offset[0]))) < 0) {
        LOG_I(HW, "[yunsdr] Failed to set RX1 gain\n");
    } else
        LOG_I(HW, "[yunsdr] set RX1 gain to %u\n",(uint32_t)(openair0_cfg->rx_gain[0]));

    if(yunsdr->rx_num_channels > 1) {
        if ((ret = yunsdr_set_rx2_rf_gain(yunsdr->dev, 0, (uint32_t)(openair0_cfg->rx_gain[1] - openair0_cfg->rx_gain_offset[0]))) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set RX2 gain\n");
        } else
            LOG_I(HW, "[yunsdr] set RX gain to %u\n",(uint32_t)(openair0_cfg->rx_gain[1]));
    }
#if 0
    if ((ret = yunsdr_set_tx1_attenuation(yunsdr->dev, 0, openair0_cfg->tx_gain[0] * 1000)) < 0) {
        LOG_E(HW, "[yunsdr] Failed to set TX1 gain\n");
    } else
        LOG_I(HW, "[yunsdr] set the TX1 gain to %d\n", 90 - (uint32_t)openair0_cfg->tx_gain[0]);

    if(yunsdr->tx_num_channels > 1) {
        if ((ret = yunsdr_set_tx2_attenuation(yunsdr->dev, 0, openair0_cfg->tx_gain[1] * 1000)) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set TX2 gain\n");
        } else
            LOG_I(HW, "[yunsdr] set the TX2 gain to %d\n", 90 - (uint32_t)openair0_cfg->tx_gain[1]);
    }
#endif
    return ret;
}

/*! \brief Initialize Openair yunsdr target. It returns 0 if OK
 * \param device the hardware to use
 * \param openair0_cfg RF frontend parameters set by application
 * \returns 0 on success
 */
int device_init(openair0_device_t *device, openair0_config_t *openair0_cfg)
{
    int status;

    yunsdr_state_t *yunsdr = (yunsdr_state_t*)malloc(sizeof(yunsdr_state_t));
    memset(yunsdr, 0, sizeof(yunsdr_state_t));

    LOG_I(HW, "[yunsdr] openair0_cfg[0].sdr_addrs == '%s'\n", openair0_cfg[0].sdr_addrs);
    LOG_I(HW, "[yunsdr] openair0_cfg[0].rx_num_channels == '%d'\n", openair0_cfg[0].rx_num_channels);
    LOG_I(HW, "[yunsdr] openair0_cfg[0].tx_num_channels == '%d'\n", openair0_cfg[0].tx_num_channels);

    yunsdr->sample_rate = (unsigned int)openair0_cfg->sample_rate;
    LOG_I(HW, "[yunsdr] sampling_rate %d\n", yunsdr->sample_rate);
    yunsdr->rx_num_channels = openair0_cfg[0].rx_num_channels;
    yunsdr->tx_num_channels = openair0_cfg[0].tx_num_channels;

    RX_MTU = openair0_cfg[0].sample_rate / 1000 / 2;
    if(RX_MTU >= 61440)
        RX_MTU = 61440;
    else if(!(RX_MTU % 5760))
        RX_MTU = 5760;
    else
        RX_MTU = 30720;

    bool rx_ant = true;
    bool pa_status = false;
    int auxdac1 = 0;
    char args[64];
    if (openair0_cfg[0].sdr_addrs == NULL) {
        strcpy(args, "dev=pciex:0");
    } else {
        strcpy(args, openair0_cfg[0].sdr_addrs);
    }

    char dev_str[64];
    const char dev_arg[] = "dev=";
    char *dev_ptr = strstr(args, dev_arg);
    if(dev_ptr) {
        copy_subdev_string(dev_str, dev_ptr + strlen(dev_arg));
        remove_substring(args, dev_arg);
        remove_substring(args, dev_str);
        LOG_I(HW, "[yunsdr] Using %s\n", dev_str);
    }

    const char auxdac1_arg[] = "auxdac1=";
    char auxdac1_str[64] = {0};
    char *auxdac1_ptr = strstr(args, auxdac1_arg);
    if(auxdac1_ptr) {
        copy_subdev_string(auxdac1_str, auxdac1_ptr + strlen(auxdac1_arg));
        remove_substring(args, auxdac1_arg);
        remove_substring(args, auxdac1_str);
        auxdac1 = atoi(auxdac1_str);
        LOG_I(HW, "[yunsdr] Setting auxdac1:%u\n", auxdac1);
    }

    const char pa_arg[] = "pa=";
    char pa_str[64] = {0};
    char *pa_ptr = strstr(args, pa_arg);
    if(pa_ptr) {
        copy_subdev_string(pa_str, pa_ptr + strlen(pa_arg));
        remove_substring(args, pa_arg);
        remove_substring(args, pa_str);
        pa_status = !strcmp(pa_str, "enabled");
        LOG_I(HW, "[yunsdr] PA Status:%s\n", pa_status?"Enabled":"Disabled");
    }

    const char ant_arg[] = "rx_ant=";
    char ant_str[64] = {0};
    char *ant_ptr = strstr(args, ant_arg);
    if(ant_ptr) {
        copy_subdev_string(ant_str, ant_ptr + strlen(ant_arg));
        remove_substring(args, ant_arg);
        remove_substring(args, ant_str);
        rx_ant = strcmp(ant_str, "trx");
    }

    char dstring[128];
    if(RX_MTU < 30720) {
        sprintf(dstring, ",nsamples_recv_frame:%u", RX_MTU);
        strcat(dev_str, dstring);
    }
    if ((yunsdr->dev = yunsdr_open_device(dev_str)) == NULL ) {
        LOG_E(HW, "[yunsdr] Failed to open yunsdr\n");
        free(yunsdr);
        return -1;
    }

    uint32_t model = 0;
    yunsdr_get_model_version(yunsdr->dev, &model);
    model &= 0xffff;
    LOG_I(HW, "[yunsdr] Device model: %d\n", model);
    yunsdr_cfg_result_t res = yunsdr_get_config(model, (int)openair0_cfg->sample_rate);
    if (res.success) {
        openair0_cfg[0].tx_sample_advance   = res.tx_adv;
        openair0_cfg[0].tx_bw               = res.bandwidth;
        openair0_cfg[0].rx_bw               = res.bandwidth;
    } else {
        LOG_I(HW, "[yunsdr] Unknown device or unsupported configuration\n");
        exit(-1);
    }

    if(yunsdr->BoardType == Y230) {
        openair0_cfg[0].rx_gain_calib_table = calib_table_yunsdr;
    } else {
        openair0_cfg[0].rx_gain_calib_table = calib_table_none;
    }
    set_rx_gain_offset(openair0_cfg, 0);
    if(yunsdr->rx_num_channels > 1)
        set_rx_gain_offset(openair0_cfg, 1);

    LOG_I(HW, "[yunsdr] Initializing openair0_device_t\n");
    switch (openair0_cfg[0].clock_source) {
    case external:
        LOG_I(HW, "[yunsdr] clock_source: external\n");
        yunsdr_set_ref_clock (yunsdr->dev, 0, EXTERNAL_REFERENCE);
        yunsdr_set_pps_select (yunsdr->dev, 0, PPS_EXTERNAL_EN);
        break;
    case gpsdo:
        LOG_I(HW, "[yunsdr] clock_source: gpsdo\n");
        yunsdr_set_ref_clock (yunsdr->dev, 0, INTERNAL_REFERENCE);
        yunsdr_set_pps_select (yunsdr->dev, 0, PPS_GPS_EN);
        break;
    case internal:
    default:
        yunsdr_set_ref_clock (yunsdr->dev, 0, INTERNAL_REFERENCE);
        yunsdr_set_pps_select (yunsdr->dev, 0, PPS_INTERNAL_EN);
        LOG_I(HW, "[yunsdr] clock_source: internal\n");
        break;
    }
    yunsdr_set_auxdac1 (yunsdr->dev, 0, auxdac1);
    if (pa_status) {
        yunsdr_set_duplex_select (yunsdr->dev, 0, RF_FDD);
        yunsdr_set_trxsw_fpga_enable(yunsdr->dev, 0, 1);
        yunsdr_set_rx_ant_enable (yunsdr->dev, 0, 0);
    } else if (openair0_cfg->duplex_mode == duplex_mode_TDD && !rx_ant) {
        yunsdr_set_duplex_select (yunsdr->dev, 0, RF_TDD);
        yunsdr_set_trxsw_fpga_enable(yunsdr->dev, 0, 1);
        yunsdr_set_rx_ant_enable (yunsdr->dev, 0, 0);
    } else {
        yunsdr_set_duplex_select (yunsdr->dev, 0, RF_FDD);
        yunsdr_set_trxsw_fpga_enable(yunsdr->dev, 0, 0);
        yunsdr_set_rx_ant_enable (yunsdr->dev, 0, 1);
    }
    LOG_I(HW, "[yunsdr] RX Ant:%s\n", rx_ant?"Enabled":"Disabled");
    yunsdr_set_tx_fir_en_dis (yunsdr->dev, 0, 0);
    yunsdr_set_rx_fir_en_dis (yunsdr->dev, 0, 0);

    int max_idx = ((yunsdr->rx_num_channels > 2 || yunsdr->tx_num_channels > 2)? 1 : 0);

    for(int chip_idx = 0; chip_idx <= max_idx; chip_idx++) {
        // RX port Initialize
        if ((status = yunsdr_set_rx_lo_freq(yunsdr->dev, chip_idx, (uint64_t)(openair0_cfg->rx_freq[0]))) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set RX frequency\n");
        } else
            LOG_I(HW, "[yunsdr] set RX frequency to %lu\n",(uint64_t)(openair0_cfg->rx_freq[0]));
        if ((status = yunsdr_set_rx_sampling_freq(yunsdr->dev, chip_idx, (uint32_t)(openair0_cfg->sample_rate))) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set RX sample rate\n");
        } else
            LOG_I(HW, "[yunsdr] set RX sample rate to %u\n", (uint32_t)(openair0_cfg->sample_rate));
        if ((status = yunsdr_set_rx_rf_bandwidth(yunsdr->dev, chip_idx, (uint32_t)(openair0_cfg->rx_bw))) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set RX bandwidth\n");
        } else
            LOG_I(HW, "[yunsdr] set RX bandwidth to %u\n",(uint32_t)(openair0_cfg->rx_bw));

        if ((status = yunsdr_set_rx1_gain_control_mode(yunsdr->dev, chip_idx, 0)) < 0){
            LOG_E(HW, "[yunsdr] Failed to set RX1 Gain Control Mode\n");
        } else
            LOG_I(HW, "[yunsdr] set RX1 Gain Control Mode MGC\n");

        uint32_t rxgain = openair0_cfg->rx_gain[0] - openair0_cfg->rx_gain_offset[0];
        if (openair0_cfg->rx_gain_offset[0] > 0) {
            if (rxgain > 30)
                rxgain = 60;
            else
                rxgain *= 2;
        } else {
            if (rxgain > 60)
                rxgain = 60;
        }
        if ((status = yunsdr_set_rx1_rf_gain(yunsdr->dev, chip_idx, rxgain)) < 0) {
            LOG_I(HW, "[yunsdr] Failed to set RX1 gain\n");
        } else
            LOG_I(HW, "[yunsdr] set RX1 gain to %u\n", rxgain);

        if ((status = yunsdr_set_rx2_gain_control_mode(yunsdr->dev, chip_idx, 0)) < 0){
            LOG_E(HW, "[yunsdr] Failed to set RX2 Gain Control Mode\n");
        } else
            LOG_I(HW, "[yunsdr] set RX2 Gain Control Mode MGC\n");

        if ((status = yunsdr_set_rx2_rf_gain(yunsdr->dev, chip_idx, rxgain)) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set RX2 gain\n");
        } else
            LOG_I(HW, "[yunsdr] set RX2 gain to %u\n", rxgain);

        // TX port Initialize
        if ((status = yunsdr_set_tx_lo_freq(yunsdr->dev, chip_idx, (uint64_t)openair0_cfg->tx_freq[0])) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set TX frequency\n");
        } else
            LOG_I(HW, "[yunsdr] set TX Frequency to %lu\n", (uint64_t)openair0_cfg->tx_freq[0]);

        if ((status = yunsdr_set_tx_sampling_freq(yunsdr->dev, chip_idx, (uint32_t)openair0_cfg->sample_rate)) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set TX sample rate\n");
        } else
            LOG_I(HW, "[yunsdr] set TX sampling rate to %u\n", (uint32_t)openair0_cfg->sample_rate);

        if ((status = yunsdr_set_tx_rf_bandwidth(yunsdr->dev, chip_idx, (uint32_t)openair0_cfg->tx_bw)) <0) {
            LOG_E(HW, "[yunsdr] Failed to set TX bandwidth\n");
        } else
            LOG_I(HW, "[yunsdr] set TX bandwidth to %u\n", (uint32_t)openair0_cfg->tx_bw);

        if ((status = yunsdr_set_tx1_attenuation(yunsdr->dev, chip_idx, openair0_cfg->tx_gain[0] * 1000)) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set TX1 gain\n");
        } else
            LOG_I(HW, "[yunsdr] set the TX1 gain to %d\n", 90 - (uint32_t)openair0_cfg->tx_gain[0]);

        if ((status = yunsdr_set_tx2_attenuation(yunsdr->dev, chip_idx, openair0_cfg->tx_gain[1] * 1000)) < 0) {
            LOG_E(HW, "[yunsdr] Failed to set TX2 gain\n");
        } else
            LOG_I(HW, "[yunsdr] set the TX2 gain to %d\n", 90 - (uint32_t)openair0_cfg->tx_gain[1]);
    }

    uint32_t depth = yunsdr->sample_rate / 100 * sizeof(int) * yunsdr->rx_num_channels;
    yunsdr_set_hwbuf_depth(yunsdr->dev, 0, depth);

    yunsdr_enable_timestamp(yunsdr->dev, 0, 0);
    sleep(2);
    yunsdr_enable_timestamp(yunsdr->dev, 0, 1);
    sleep(2);

    for(int i = 0; i < NCHAN_PER_DEV; i++) {
        int ret = posix_memalign((void **)&cache_buf[i], 4096, BUFFER_SIZE);
        if(ret) {
            LOG_I(HW, "Failed to alloc memory\n");
            return -1;
        }
        ret = posix_memalign((void **)&iq_buf[i], 4096, BUFFER_SIZE);
        if(ret) {
            LOG_I(HW, "Failed to alloc memory\n");
            return -1;
        }
    }

    device->host_type            = RAU_HOST;
    device->type                 = YUNSDR_DEV;
    device->trx_start_func       = trx_yunsdr_start;
    device->trx_end_func         = trx_yunsdr_end;
    device->trx_read_func        = trx_yunsdr_read;
    device->trx_write_func       = trx_yunsdr_write;
    device->trx_get_stats_func   = trx_yunsdr_get_stats;
    device->trx_reset_stats_func = trx_yunsdr_reset_stats;
    device->trx_stop_func        = trx_yunsdr_stop;
    device->trx_set_freq_func    = trx_yunsdr_set_freq;
    device->trx_set_gains_func   = trx_yunsdr_set_gains;
    device->trx_write_init       = trx_yunsdr_write_init;
    device->openair0_cfg         = openair0_cfg;
    device->priv                 = (void *)yunsdr;

    return 0;
}

/*@}*/
