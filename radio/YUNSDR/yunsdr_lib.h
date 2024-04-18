/*
 * SPDX-License-Identifier: LicenseRef-CSSL-1.0
 */

/** yunsdr_lib.h
 *
 * Author: eric
 */

#include <stdint.h>
#include <stdbool.h>
#include "yunsdr_api_ss.h"
#include "common_lib.h"

/** @addtogroup _YUNSDR_PHY_RF_INTERFACE_
 * @{
 */

typedef enum {
    Y230    = 230,
    Y240    = 240,
    Y380    = 380,
    Y390    = 390,
    Y550    = 550,
    Y590    = 590,
    Y750    = 750,
    Y790    = 790,
    IQX6000 = 6000,
    IQX7000 = 7000,
    IQX7100 = 7100,
    IQX7200 = 7200,
    IQX7400 = 7400,
    IQX7402 = 7402, //split
    IQX7600 = 7600,
    IQX8400 = 8400,
    IQX8402 = 8402,  //split
    IQX8800 = 8800,
    UNKNOWN = -1,
} YUNSDRBoardType;

typedef struct {
    uint64_t sample_rate;
    uint64_t bandwidth;
    int32_t  tx_adv;
} yunsdr_cfg_item_t;

#define YUNSDR_CFG_END  {0, 0, -1}

typedef struct {
    YUNSDRBoardType               model;
    const yunsdr_cfg_item_t *const items;
    const char *const             name;
} yunsdr_cfg_table_t;

typedef struct {
    uint64_t bandwidth;
    int32_t   tx_adv;
    bool     success;
} yunsdr_cfg_result_t;

// Y230
static const yunsdr_cfg_item_t cfg_y230[] = {
    {30720000, 20000000, 80},
    {23040000, 20000000, 80},
    {15360000, 10000000, 52},
    {7680000,  5000000,  34},
    {5760000,  3000000,  22},
    {3840000,  3000000,  18},
    {1920000,  1400000,   9},
    YUNSDR_CFG_END
};

// Y240
static const yunsdr_cfg_item_t cfg_y240[] = {
    {61440000, 50000000, 80},
    {46080000, 40000000, 80},
    {30720000, 20000000, 80},
    {23040000, 20000000, 80},
    {15360000, 10000000, 52},
    {7680000,  5000000,  34},
    {5760000,  3000000,  22},
    {3840000,  3000000,  18},
    {1920000,  1400000,   9},
    YUNSDR_CFG_END
};

// IQX8200 & IQX8400 & IQX8800 & IQX8402
static const yunsdr_cfg_item_t cfg_iqx8x00[] = {
    {491520000, 400000000, 151},
    {245760000, 200000000, 126},
    {122880000, 100000000, 111},
    {61440000,  40000000,  172},
    {30720000,  20000000,  169},
    {15360000,  10000000,  110},
    {7680000,   5000000,   110},
    {3840000,   3000000,   110},
    {1920000,   1400000,   110},
    YUNSDR_CFG_END
};

// IQX7100/IQX7400/IQX7402
static const yunsdr_cfg_item_t cfg_iqx7x00[] = {
    {122880000, 100000000, 573},
    {61440000,  40000000,  286},
    {30720000,  20000000,  143},
    {15360000,  10000000,   73},
    {7680000,   5000000,    36},
    {3840000,   3000000,    17},
    {1920000,   1400000,     9},
    YUNSDR_CFG_END
};

// IQX72000
static const yunsdr_cfg_item_t cfg_iqx7200[] = {
    {245760000, 200000000, 612},
    {122880000, 100000000, 306},
    {61440000,  40000000,  152},
    {30720000,  20000000,   76},
    {15360000,  10000000,   38},
    {7680000,   5000000,    19},
    {3840000,   3000000,    10},
    {1920000,   1400000,     5},
    YUNSDR_CFG_END
};

// Y590Neo
static const yunsdr_cfg_item_t cfg_y590[] = {
    {245760000, 200000000, 502},
    {122880000, 100000000, 251},
    {61440000,  40000000,  125},
    {30720000,  20000000,   63},
    {15360000,  10000000,   32},
    {7680000,   5000000,    16},
    {3840000,   3000000,     8},
    {1920000,   1400000,     4},
    YUNSDR_CFG_END
};

// Y790/Y750
static const yunsdr_cfg_item_t cfg_y790[] = {
    {491520000, 400000000, 149},
    {245760000, 200000000, 143},
    {122880000, 100000000, 130},
    {61440000,  40000000,  105},
    {30720000,  20000000,  102},
    YUNSDR_CFG_END
};

static const yunsdr_cfg_table_t g_board_tables[] = {
    {IQX8800,  cfg_iqx8x00,  "IQX8800/IQX8400"},
    {IQX8400,  cfg_iqx8x00,  "IQX8800/IQX8400"},
    {IQX8402,  cfg_iqx8x00,  "IQX8800/IQX8400"},
    {IQX7100,  cfg_iqx7x00,  "IQX7100/IQX7400"},
    {IQX7400,  cfg_iqx7x00,  "IQX7100/IQX7400"},
    {IQX7402,  cfg_iqx7x00,  "IQX7100/IQX7400"},
    {IQX7200,  cfg_iqx7200,  "IQX7200"},
    {Y230,     cfg_y230,     "Y230"},
    {Y240,     cfg_y240,     "Y240"},
    {Y380,     cfg_iqx7x00,  "Y380"},
    {Y390,     cfg_y590,     "Y390"}, // FDD is not supported.
    {Y590,     cfg_y590,     "Y590neo"},
    {Y790,     cfg_y790,     "Y790/Y750"},
    {Y750,     cfg_y790,     "Y790/Y750"},
    {UNKNOWN,  NULL,         "UNKNOWN"}
};


/*! \brief YunSDR specific data structure */
typedef struct {

  //! opaque YunSDR device struct. An empty ("") or NULL device identifier will result in the first encountered device being opened (using the first discovered backend)
  YUNSDR_DESCRIPTOR *dev;
  int16_t *rx_buffer;
  int16_t *tx_buffer;
  //! Sample rate
  unsigned int sample_rate;

  int rx_num_channels;
  int tx_num_channels;
  uint64_t tx_lo_freq;
  uint64_t rx_lo_freq;

  // --------------------------------
  // Debug and output control
  // --------------------------------
  //! Number of underflows
  int num_underflows;
  //! Number of overflows
  int num_overflows;
  //! number of RX errors
  int num_rx_errors;
  //! Number of TX errors
  int num_tx_errors;

  //! timestamp of current TX
  uint64_t tx_current_ts;
  //! timestamp of current RX
  uint64_t rx_current_ts;
  //! number of TX samples
  uint64_t tx_nsamps;
  //! number of RX samples
  uint64_t rx_nsamps;
  //! number of TX count
  uint64_t tx_count;
  //! number of RX count
  uint64_t rx_count;
  //! timestamp of RX packet
  openair0_timestamp_t rx_timestamp;
  YUNSDRBoardType BoardType;
} yunsdr_state_t;

/*@}*/
