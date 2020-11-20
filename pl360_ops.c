/*
Copyright (c) 2020 Eug Krashtan

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "ethplc.h"
#include <linux/kfifo.h>

/* ! \name G3 Modulation types */
enum mod_types {
	MOD_TYPE_BPSK = 0,
	MOD_TYPE_QPSK = 1,
	MOD_TYPE_8PSK = 2,
	MOD_TYPE_QAM = 3,
	MOD_TYPE_BPSK_ROBO = 4,
};

/* ! \name G3 Modulation schemes */
enum mod_schemes {
	MOD_SCHEME_DIFFERENTIAL = 0,
	MOD_SCHEME_COHERENT = 1,
};

/* ! \name G3 Frame Delimiter Types */
enum delimiter_types {
	DT_SOF_NO_RESP = 0,
	DT_SOF_RESP = 1,
	DT_ACK = 2,
	DT_NACK = 3,
};

/* ! Internal Memory Map */
typedef enum atpl360_mem_id {
	ATPL360_STATUS_INFO_ID = 0,
	ATPL360_TX_PARAM_ID,
	ATPL360_TX_DATA_ID,
	ATPL360_TX_CFM_ID,
	ATPL360_RX_PARAM_ID,
	ATPL360_RX_DATA_ID,
	ATPL360_REG_INFO_ID,
	ATPL360_IDS,
} pl360_mem_id_t;

/* Defines relatives to configuration parameter (G3) */
typedef enum atpl360_reg_id {
	ATPL360_REG_PRODID = 0x4000,
	ATPL360_REG_MODEL,
	ATPL360_REG_VERSION_STR,
	ATPL360_REG_VERSION_NUM,
	ATPL360_REG_TONE_MASK,
	ATPL360_REG_TONE_MAP_RSP_DATA,
	ATPL360_REG_TX_TOTAL,
	ATPL360_REG_TX_TOTAL_BYTES,
	ATPL360_REG_TX_TOTAL_ERRORS,
	ATPL360_REG_TX_BAD_BUSY_TX,
	ATPL360_REG_TX_BAD_BUSY_CHANNEL,
	ATPL360_REG_TX_BAD_LEN,
	ATPL360_REG_TX_BAD_FORMAT,
	ATPL360_REG_TX_TIMEOUT,
	ATPL360_REG_RX_TOTAL,
	ATPL360_REG_RX_TOTAL_BYTES,
	ATPL360_REG_RX_RS_ERRORS,
	ATPL360_REG_RX_EXCEPTIONS,
	ATPL360_REG_RX_BAD_LEN,
	ATPL360_REG_RX_BAD_CRC_FCH,
	ATPL360_REG_RX_FALSE_POSITIVE,
	ATPL360_REG_RX_BAD_FORMAT,
	ATPL360_REG_ENABLE_AUTO_NOISE_CAPTURE,
	ATPL360_REG_TIME_BETWEEN_NOISE_CAPTURES,
	ATPL360_REG_DELAY_NOISE_CAPTURE_AFTER_RX,
	ATPL360_REG_RRC_NOTCH_ACTIVE,
	ATPL360_REG_RRC_NOTCH_INDEX,
	ATPL360_REG_NOISE_PEAK_POWER,
	ATPL360_REG_RSV0,
	ATPL360_REG_RSV1,
	ATPL360_REG_CFG_AUTODETECT_IMPEDANCE,
	ATPL360_REG_CFG_IMPEDANCE,
	ATPL360_REG_ZC_PERIOD,
	ATPL360_REG_FCH_SYMBOLS,
	ATPL360_REG_PAY_SYMBOLS_TX,
	ATPL360_REG_PAY_SYMBOLS_RX,
	ATPL360_REG_RRC_NOTCH_AUTODETECT,
	ATPL360_REG_MAX_RMS_TABLE_HI,
	ATPL360_REG_MAX_RMS_TABLE_VLO,
	ATPL360_REG_THRESHOLDS_TABLE_HI,
	ATPL360_REG_THRESHOLDS_TABLE_LO,
	ATPL360_REG_THRESHOLDS_TABLE_VLO,
	ATPL360_REG_PREDIST_COEF_TABLE_HI,
	ATPL360_REG_PREDIST_COEF_TABLE_LO,
	ATPL360_REG_PREDIST_COEF_TABLE_VLO,
	ATPL360_REG_GAIN_TABLE_HI,
	ATPL360_REG_GAIN_TABLE_LO,
	ATPL360_REG_GAIN_TABLE_VLO,
	ATPL360_REG_DACC_TABLE_CFG,
	ATPL360_REG_RSV2,
	ATPL360_REG_NUM_TX_LEVELS,
	ATPL360_REG_CORRECTED_RMS_CALC,
	ATPL360_REG_RRC_NOTCH_THR_ON,
	ATPL360_REG_RRC_NOTCH_THR_OFF,
	ATPL360_REG_CURRENT_GAIN,
	ATPL360_REG_ZC_CONF_INV,
	ATPL360_REG_ZC_CONF_FREQ,
	ATPL360_REG_ZC_CONF_DELAY,
	ATPL360_REG_NOISE_PER_CARRIER,
	ATPL360_REG_END_ID,
} pl360_reg_id_t;

/* ! Defines relatives to some ATPL360 registers */
#define ATPL360_REG_ADC_MASK                    0x1000
#define ATPL360_REG_DAC_MASK                    0x2000
#define ATPL360_REG_MASK                        0x4000
#define ATPL360_FUSES_MASK                      0x8000
#define ATPL360_REG_ADC_BASE                    0x40000000
#define ATPL360_REG_DAC_BASE                    0x40004000
#define ATPL360_REG_BASE                        0x80000000
#define ATPL360_FUSES_BASE                      0x400E1800

/* ! FLAG MASKs for set events */
#define ATPL360_TX_CFM_FLAG_MASK                 0x0001
#define ATPL360_RX_DATA_IND_FLAG_MASK            0x0002
#define ATPL360_CD_FLAG_MASK                     0x0004
#define ATPL360_REG_RSP_MASK                     0x0008
#define ATPL360_RX_QPAR_IND_FLAG_MASK            0x0010

/* ! Event Info MASKs */
#define ATPL360_EV_DAT_LEN_MASK                  0x0000FFFF
#define ATPL360_EV_REG_LEN_MASK                  0xFFFF0000
#define ATPL360_GET_EV_DAT_LEN_INFO(x)           ((uint32_t)x & ATPL360_EV_DAT_LEN_MASK)
#define ATPL360_GET_EV_REG_LEN_INFO(x)           (((uint32_t)x & ATPL360_EV_REG_LEN_MASK) >> 16)

#define PL360_IF_DEFAULT_TONE_MAP									{ 0x03, 0xFF, 0xFF }
#define PL360_IF_TX_POWER										0
/* ! TX Mode: Delayed transmission */
#define TX_MODE_RELATIVE             (1 << 1)
/* ! Maximum number of subbands */
#define NUM_SUBBANDS_MAX                       24
/* ! Maximum number of tone map */
#define TONE_MAP_SIZE_MAX                      3
/* ! Maximum number of protocol carriers */
#define PROTOCOL_CARRIERS_MAX                  72

/* Init packet data size */
#define INIT_PKT_DATA_SIZE (8)

/**
 * @brief Format of packet part
 * max PLC transfer size 163 bytes
 * The first two bytes contains key,
 * followed by 1 byte - len and part number,
 * and up to 160 bytes for payload
 * +-----+--------+---- ... -------+
 * | Key | Length | Ethernet Frame |
 * +-----+--------+---- ... -------+
 * Ethernet MTU = 1500
 * Result part count  1500/160 = 10
 */
#define MAX_ETH_PLC_PARTS 10
#define MAX_PAYLOAD_LEN (MAX_PLC_PKT_LEN - 3)
#define PLC_MTU 1500
#define PKT_LEN_MASK (0xF8)
// How many incomplete packets allowed
#define MAX_ETH_PLC_SESSIONS 3
#define LAST_PART_INDICATOR 0xFF

/* ! \name G3 TX Result values */
enum tx_result_values {
	TX_RESULT_PROCESS = 0,                  /* Transmission result: already in process */
	TX_RESULT_SUCCESS = 1,                  /* Transmission result: end successfully */
	TX_RESULT_INV_LENGTH = 2,               /* Transmission result: invalid length error */
	TX_RESULT_BUSY_CH = 3,                  /* Transmission result: busy channel error */
	TX_RESULT_BUSY_TX = 4,                  /* Transmission result: busy in transmission error */
	TX_RESULT_BUSY_RX = 5,                  /* Transmission result: busy in reception error */
	TX_RESULT_INV_SCHEME = 6,               /* Transmission result: invalid modulation scheme error */
	TX_RESULT_TIMEOUT = 7,                  /* Transmission result: timeout error */
	TX_RESULT_INV_TONEMAP = 8,              /* Transmission result: invalid tone map error */
	TX_RESULT_INV_MODE = 9,                 /* Transmission result: invalid G3 Mode error */
	TX_RESULT_NO_TX = 255,                  /* Transmission result: No transmission ongoing */
};

#pragma pack(push,1)

typedef struct __attribute__((__packed__)) {
	uint32_t tx_time;
	uint16_t data_len;
	uint8_t tone_groups[NUM_SUBBANDS_MAX];
	uint8_t tone_map[TONE_MAP_SIZE_MAX];
	uint8_t tx_mode;                            /* Transmission Mode (absolute, relative, forced, continuous, cancel). Constants above */
	uint8_t tx_power;                           /* Power to transmit */
	uint8_t mod_type;                           /* Modulation type */
	uint8_t mod_scheme;                         /* Modulation scheme */
	uint8_t pdc;                                /* Phase Detector Counter */
	uint8_t rs_blocks;                          /* Flag to indicate whether 2 RS blocks have to be used (only used for FCC) */
	uint8_t uc_delimiter_type;                  /* DT field to be used in header */
} pl360_tx_config_t;

/* ! \name G3 Structure defining Rx message */
typedef struct rx_msg {
	uint32_t ul_rx_time;                           /* /< Instant when frame was received */
	uint32_t ul_frame_duration;                    /* /< Frame duration referred to 1ms PHY counter (FCH + Payload) */
	uint16_t us_rssi;                              /* /< Reception RSSI */
	uint16_t us_data_len;                          /* /< Length of the data buffer */
	uint8_t uc_zct_diff;                           /* /< ZCT info */
	uint8_t uc_rs_corrected_errors;                /* /< Errors corrected by RS */
	enum mod_types uc_mod_type;                    /* /< Modulation type of the last received message */
	enum mod_schemes uc_mod_scheme;                /* /< Modulation scheme of the last received message */
	uint32_t ul_agc_factor;                        /* /< Test data information */
	uint16_t us_agc_fine;                          /* /< Test data information */
	int16_t ss_agc_offset_meas;                    /* /< Test data information */
	uint8_t uc_agc_active;                         /* /< Test data information */
	uint8_t uc_agc_pga_value;                      /* /< Test data information */
	int16_t ss_snr_fch;                            /* /< Test data information */
	int16_t ss_snr_pay;                            /* /< Test data information */
	uint16_t us_payload_corrupted_carriers;        /* /< BER: Number of corrupted carriers */
	uint16_t us_payload_noised_symbols;            /* /< BER: Number of noised symbols */
	uint8_t uc_payload_snr_worst_carrier;          /* /< BER: SNR of the worst carrier */
	uint8_t uc_payload_snr_worst_symbol;           /* /< BER: SNR of the worst symbol */
	uint8_t uc_payload_snr_impulsive;              /* /< BER: SNR on impulsive noise */
	uint8_t uc_payload_snr_band;                   /* /< BER: Narrowband SNR */
	uint8_t uc_payload_snr_background;             /* /< BER: Background SNR */
	uint8_t uc_lqi;                                /* /< BER: Link Quality */
	enum delimiter_types uc_delimiter_type;        /* /< DT field coming in header */
	uint8_t uc_rsrv0;                              /* /< MAC CRC. 1: OK; 0: NOK (CRC capability can be enabled/disabled). 16 bits for allignement  */
	uint8_t puc_tone_map[TONE_MAP_SIZE_MAX];       /* /< Reception Tone Map */
	uint8_t puc_carrier_snr[PROTOCOL_CARRIERS_MAX]; /* /< SNR per carrier */
	uint8_t uc_rsrv1;                              /* /< Reserved byte */
	uint8_t *puc_data_buf;                         /* /< Pointer to data buffer */
} rx_msg_t;

/* ! \name G3 Structure defining result of a transmission */
typedef struct tx_cfm {
	uint32_t ul_rms_calc;                          /* RMS_CALC it allows to estimate tx power injected */
	uint32_t ul_tx_time;                           /* Instant when frame transmission ended referred to 1ms PHY counter */
	enum tx_result_values uc_tx_result;            /* Tx Result (see "TX Result values" above) */
} tx_cfm_t;

typedef struct __attribute__((__packed__)) {
	uint32_t time;
	uint32_t evt;
} status_t;

#pragma pack(pop)

typedef struct {
    uint16_t len;
    uint16_t addr;
    pl360_tx_config_t conf;
} txconf_t;
static txconf_t txcf;

#define ATPL360_CMF_PKT_SIZE                      sizeof(tx_cfm_t)
#define PL360_FIFO_SIZE 10

static struct kfifo tx_fifo; // Packet queued to transmit
static status_t status;

typedef struct {
    uint16_t len;
    uint16_t addr;
    uint8_t buf[8];
} plc_pkt8_t;

typedef struct __attribute__((packed)) {
	uint16_t key;
	uint8_t len;
	uint8_t payload[0];
} eth_pl360_part_t;

typedef struct {
	uint16_t key;
	uint8_t* parts[MAX_ETH_PLC_PARTS];
	uint8_t age;
} eth_pl360_rx_t;

static eth_pl360_rx_t rx_parts[MAX_ETH_PLC_SESSIONS];

/**
 * @brief Part length encoding
 * 0-159 last part indication, length of last piece
 * 0xF8-0xFF - part number (0-7)
 * @param[in] in - incoming length
 * @param[out] real_len - real packet Length
 * @return part number
 */
static uint8_t get_pos_and_len(uint8_t in, uint8_t* real_len) {
	if ((in&PKT_LEN_MASK) == PKT_LEN_MASK) {
		*real_len = MAX_PAYLOAD_LEN;
		return in & 7;
	} else if(in<(MAX_PAYLOAD_LEN)) {
		*real_len = in;
		return LAST_PART_INDICATOR;
	} else {
		return MAX_ETH_PLC_PARTS+1; // bad packet len - indicate 'part lost'
	}
}

static void clear_parts(eth_pl360_rx_t* pkt) {
	//printk("Clear part %p key 0x%04x", pkt, pkt->key);
	pkt->key = 0;
	for (int i = 0; i < MAX_ETH_PLC_PARTS; i++) {
		if (pkt->parts[i]) {
			kfree(pkt->parts[i]);
		}
		pkt->parts[i] = NULL;
	}
}

/**
 * @brief Looking for key and clear obsolete parts
 */
static int parts_check_key(uint16_t key) {
	int idx = -1;
	for (int i = 0; i < MAX_ETH_PLC_SESSIONS; i++) {
		if (rx_parts[i].age) rx_parts[i].age--;
		if (rx_parts[i].key == key) {
			idx = i;
			rx_parts[i].age += MAX_ETH_PLC_SESSIONS;
			continue;
		}
		// Remove obsolete parts
		if (rx_parts[i].age == 0 && rx_parts[i].key > 0 ) {
			clear_parts(&(rx_parts[i]));
		}
	}
	return idx;
}

static void pl360_update_status(struct pl360_local *lp) {
	plc_pkt8_t pkt = {
		.addr = ATPL360_STATUS_INFO_ID,
		.len = INIT_PKT_DATA_SIZE,
	};
	pl360_datapkt(lp, PLC_CMD_READ, (plc_pkt_t*)&pkt);
	memcpy(&status, pkt.buf, sizeof(status));
}

static int pl360_rx(struct pl360_local *lp, plc_pkt_t* pkt) {
	struct sk_buff *skb = NULL;
	uint8_t* skbuf;
	uint8_t real_len;
	int i, res;
	eth_pl360_part_t* part = (eth_pl360_part_t*) pkt->buf;
	uint8_t pos = get_pos_and_len(part->len, &real_len);
	int index = parts_check_key(part->key);
	/*printk(KERN_DEBUG "RX len %d part %d key 0x%04x idx %d", real_len,
				pos, part->key, index);*/

	if (pos == 0) { // Allocate new storage
		index = MAX_ETH_PLC_SESSIONS - 1; // If no free space - use last
		for (i = 0; i < MAX_ETH_PLC_SESSIONS; i++) {
			if (rx_parts[i].key == 0) {
				index = i;
			}
		}
		clear_parts(&rx_parts[index]);
		rx_parts[index].parts[0] = kmalloc(MAX_PAYLOAD_LEN, GFP_KERNEL);
		rx_parts[index].key = part->key;
		// Maximum time to wait parts
		rx_parts[index].age = MAX_ETH_PLC_PARTS*MAX_ETH_PLC_SESSIONS;
		memcpy(rx_parts[index].parts[0], part->payload, MAX_PAYLOAD_LEN);
		//printk(KERN_DEBUG "New part %p key 0x%04x", &rx_parts[index], part->key);
	} else if (pos == LAST_PART_INDICATOR) { // Packet ready to handle
		if (index == -1) {
			// Single part
			skb = netdev_alloc_skb(lp->netdev, real_len);
			if (!skb) {
				netdev_err(lp->netdev, "failed to allocate sk_buff\n");
				lp->netdev->stats.rx_errors++;
				return -ENOMEM;
			} else {
				skbuf = skb_put(skb, real_len);
				memcpy(skbuf, part->payload, real_len);
			}
		} else {
			skb = netdev_alloc_skb(lp->netdev, PLC_MTU);
			if (!skb) {
				netdev_err(lp->netdev, "failed to allocate sk_buff\n");
				clear_parts(&(rx_parts[index]));
				lp->netdev->stats.rx_errors++;
				return -ENOMEM;
			} else {
				for (int i = 0; i < MAX_ETH_PLC_PARTS; i++) {
					if (rx_parts[index].parts[i]) {
						skbuf = skb_put(skb, MAX_PAYLOAD_LEN);
						memcpy(skbuf, part->payload, MAX_PAYLOAD_LEN);
					} else {
						break;
					}
				}
				skbuf = skb_put(skb, real_len);
				memcpy(skbuf, part->payload, real_len);
			}
		}
	} else if (pos > MAX_ETH_PLC_PARTS) { // wrong packet
		lp->netdev->stats.rx_frame_errors++;
		return -1;
	} else { // add to sequence
		// Check if we lost previous piece
		if (rx_parts[index].parts[pos-1] == NULL) {
			/*printk("Lost part with key 0x%04x piece %d", 
							rx_parts[index].key, pos-1);*/
			clear_parts(&rx_parts[index]);
			lp->netdev->stats.rx_missed_errors++;
		} else {
			rx_parts[index].parts[pos] = kmalloc(MAX_PAYLOAD_LEN, GFP_KERNEL);
			memcpy(rx_parts[index].parts[pos], part->payload, MAX_PAYLOAD_LEN);
			/*printk(KERN_DEBUG "Add part with key 0x%04x piece %d", 
							part->key, pos);*/
		}
	}

	if (skb) {
		skb->protocol = eth_type_trans(skb, lp->netdev);
		int len = skb->len;
		res = netif_rx_ni(skb);
		if (res == NET_RX_SUCCESS) {
			lp->netdev->stats.rx_packets++;
			lp->netdev->stats.rx_bytes += len;
		} else {
			lp->netdev->stats.rx_dropped++;
		}
	}
	return 0;
}

void pl360_handle_rx_work(struct work_struct *work)
{
	struct pl360_local *lp =
		container_of(work, struct pl360_local, rxwork);

	typedef enum {
		TX_READY,
		TX_BUSY
	} tx_state_e;
	static tx_state_e txstate = TX_READY;

	do {
		pl360_update_status(lp);
		if (lp->events & ATPL360_TX_CFM_FLAG_MASK) {
			// Handle TX confirm
			plc_pkt_t* cfmpkt = kmalloc(sizeof(plc_pkt_t)
				+ ATPL360_CMF_PKT_SIZE, GFP_KERNEL);
			cfmpkt->addr = ATPL360_TX_CFM_ID;
			cfmpkt->len = ATPL360_CMF_PKT_SIZE;
			pl360_datapkt(lp, PLC_CMD_READ, cfmpkt);
			//tx_cfm_t* cfm = (tx_cfm_t*)cfmpkt->buf;
			txstate = TX_READY;
			kfree(cfmpkt);
		} else if (lp->events & ATPL360_REG_RSP_MASK) {
			// Handle RegResp
			plc_pkt_t* rsppkt;
			uint16_t evt_len = ATPL360_GET_EV_REG_LEN_INFO(status.evt);
			if ((evt_len == 0) || (evt_len > MAX_PLC_PKT_LEN)) {
				evt_len = 1;
			}
			rsppkt = (plc_pkt_t*)kmalloc(evt_len
				+ sizeof(plc_pkt_t), GFP_KERNEL);
			rsppkt->addr = ATPL360_REG_INFO_ID;
			rsppkt->len = evt_len;
			pl360_datapkt(lp, PLC_CMD_READ, rsppkt);
			kfree(rsppkt);
		} else if (lp->events & ATPL360_RX_QPAR_IND_FLAG_MASK ||
				lp->events & ATPL360_RX_DATA_IND_FLAG_MASK) {
			// Handle RX data, data length (15 bits) 
			uint16_t l = status.evt + PDC_SPI_HEADER_SIZE;
			plc_pkt_t* rxpkt = (plc_pkt_t*)kmalloc(l + sizeof(plc_pkt_t), 
				GFP_KERNEL);
			rxpkt->addr = ATPL360_RX_DATA_ID;
			rxpkt->len = l;
			pl360_datapkt(lp, PLC_CMD_READ, rxpkt);
			// Handle RX qpar
			plc_pkt_t* qpkt = (plc_pkt_t*)kmalloc(sizeof(rx_msg_t)
				- 4 + sizeof(plc_pkt_t), GFP_KERNEL);
			qpkt->addr = ATPL360_RX_PARAM_ID;
			qpkt->len = sizeof(rx_msg_t) - 4;
			pl360_datapkt(lp, PLC_CMD_READ, qpkt);
			rx_msg_t* rxq = (rx_msg_t *)qpkt->buf;
			lp->rssi = rxq->us_rssi;
			pl360_rx(lp, rxpkt);
			kfree(rxpkt);
			kfree(qpkt);
			txstate = TX_READY;
		}
	} while (lp->events);

	if(txstate == TX_READY &&
			!kfifo_is_empty(&tx_fifo)) {
		plc_pkt_t* pkt;
		if (kfifo_out(&tx_fifo, &pkt, sizeof(plc_pkt_t*)) != sizeof(plc_pkt_t*)) {
			dev_err(&lp->spi->dev,"test_interface: Wrong number of elements popped from upstream fifo\n");
			return;
		}
		txcf.conf.data_len = pkt->len;
		pl360_datapkt(lp, PLC_CMD_WRITE, (plc_pkt_t*)&txcf);
		pl360_datapkt(lp, PLC_CMD_WRITE, pkt);
		kfree(pkt);
		txstate = TX_BUSY;
	}
}

static uint32_t access_type(uint16_t param_id)
{
	uint32_t address = 0;

	if (param_id & ATPL360_REG_ADC_MASK) {
		address = (uint32_t)ATPL360_REG_ADC_BASE;
	} else if (param_id & ATPL360_REG_DAC_MASK) {
		address = (uint32_t)ATPL360_REG_DAC_BASE;
	} else if (param_id & ATPL360_FUSES_MASK) {
		address = (uint32_t)ATPL360_FUSES_BASE;
	} else if ((param_id & ATPL360_REG_MASK) && (param_id < ATPL360_REG_END_ID)) {
		address = (uint32_t)ATPL360_REG_BASE;
	}

	return address;
}

static void pl360_set_config(plc_pkt_t* pkt, uint16_t param_id, uint16_t value) {
	uint32_t reg_addr, reg_len;

	reg_addr = (uint32_t)(param_id & PL360_REG_OFFSET_MASK) + access_type(param_id);
	reg_len = PL360_REG_CMD_WR | (1 & PL360_REG_LEN_MASK);
	pkt->buf[0] = (uint8_t)(reg_addr >> 24);
	pkt->buf[1] = (uint8_t)(reg_addr >> 16);
	pkt->buf[2] = (uint8_t)(reg_addr >> 8);
	pkt->buf[3] = (uint8_t)(reg_addr);
	pkt->buf[4] = (uint8_t)(reg_len >> 8);
	pkt->buf[5] = (uint8_t)(reg_len);
	pkt->buf[6] = value&0xFF;
	pkt->buf[7] = value>>8;

	pkt->addr = ATPL360_REG_INFO_ID;
	pkt->len = 8;
}

void pl360_conrigure(struct pl360_local *lp) {
	plc_pkt_t* pkt = kmalloc(INIT_PKT_DATA_SIZE+sizeof(plc_pkt_t), GFP_KERNEL);
	/* Read Time Ref to get SPI status and boot if necessary */
	pkt->addr = ATPL360_STATUS_INFO_ID;
	pkt->len = INIT_PKT_DATA_SIZE;
	pl360_datapkt(lp, PLC_CMD_READ, pkt);

	// Restart IRQ after boot
	pl360_datapkt(lp, PLC_CMD_READ, pkt);

	/* Disable AUTO mode and set VLO behavior by default in order to maximize signal level in anycase */
	pl360_set_config(pkt, ATPL360_REG_CFG_AUTODETECT_IMPEDANCE, 0);
	pl360_datapkt(lp, PLC_CMD_WRITE, pkt);
	msleep(1);
	pl360_set_config(pkt, ATPL360_REG_CFG_IMPEDANCE, 2);
	pl360_datapkt(lp, PLC_CMD_WRITE, pkt);
	msleep(1);
    kfree(pkt);
}

static irqreturn_t pl360_irq(int irq, void *dev_id)
{
	struct pl360_local *lp = dev_id;

	/*
	 * Can't do anything in interrupt context because we need to
	 * block (spi_sync() is blocking) so fire of the interrupt
	 * handling workqueue.
	 * Remember that we access pl360 registers through SPI bus
	 * via spi_sync() call.
	 */
	queue_work(lp->wqueue, &lp->rxwork);

	return IRQ_HANDLED;
}

int ops_pl360_start(struct net_device *ndev)
{
	int ret = 0;
	struct pl360_local *lp = netdev_priv(ndev);

	ret = kfifo_alloc(&tx_fifo,PL360_FIFO_SIZE,GFP_KERNEL);
	if(ret) {
		goto err_ops_start;
	}

	/* Prepare default TX config */
	const uint8_t tonemap[] = PL360_IF_DEFAULT_TONE_MAP;
	txcf.conf.tx_time = 0;
	memcpy(txcf.conf.tone_map, tonemap, sizeof(tonemap));

	txcf.conf.tx_mode =  TX_MODE_RELATIVE; // uc_tx_mode
	txcf.conf.tx_time = 0x3E8;
	txcf.conf.tx_power =  PL360_IF_TX_POWER; // uc_tx_power

	txcf.conf.mod_type =  MOD_TYPE_BPSK; // uc_mod_type
	txcf.conf.mod_scheme =  MOD_SCHEME_DIFFERENTIAL; // uc_mod_scheme
	txcf.conf.uc_delimiter_type =  DT_SOF_NO_RESP; // uc_delimiter_type
	txcf.addr = ATPL360_TX_PARAM_ID;
    txcf.len = sizeof(pl360_tx_config_t);

	ret = request_irq(lp->irq_id, pl360_irq,
		IRQF_TRIGGER_FALLING, "pl360-irq", spi_get_drvdata(lp->spi)	);
	if(ret) {
		printk(KERN_INFO "ISR handler req fail %d\n", ret );
	}

	/* Update RX status */
	queue_work(lp->wqueue, &lp->rxwork);

err_ops_start:
	return ret;
}


int ops_pl360_stop(struct net_device *ndev)
{
	struct pl360_local *lp = netdev_priv(ndev);

	disable_irq(lp->spi->irq);
	free_irq(lp->irq_id, spi_get_drvdata(lp->spi));
	flush_workqueue(lp->wqueue);
	kfifo_free(&tx_fifo);
	return 0;
}

int ops_pl360_xmit(struct sk_buff *skb, struct net_device *dev) {
	static uint16_t index;
	plc_pkt_t* pkt;
	struct pl360_local *lp = netdev_priv(dev);
	short len = skb->len;

	/* Drop IPv4 packets, only 0x86dd type allowed */
	if (len>14 && skb->data[12] == 0x08 ) {
		return 0;
	}

	bool is_first = true;
	index += 3;

	uint8_t partnum = 0;
	while (len > 0) {
    	pkt = (plc_pkt_t*) kmalloc(sizeof(plc_pkt_t)
				+ MAX_PLC_PKT_LEN, GFP_KERNEL);
		pkt->addr = ATPL360_TX_DATA_ID;
		eth_pl360_part_t* part = (eth_pl360_part_t*) pkt->buf;
		part->key = index;
		int dataptr = 0;
		if (len > MAX_PAYLOAD_LEN) {
			part->len = PKT_LEN_MASK + partnum++;
			memcpy(part->payload, &(skb->data[dataptr]), MAX_PAYLOAD_LEN);
			len -= MAX_PAYLOAD_LEN;
			dataptr += MAX_PAYLOAD_LEN;
			pkt->len = MAX_PLC_PKT_LEN;
		} else {
			part->len = len;
			memcpy(part->payload, &(skb->data[dataptr]), len);
			pkt->len = len + sizeof(eth_pl360_part_t);
			len = 0;
		}
		kfifo_in(&tx_fifo, &pkt, sizeof(plc_pkt_t*));
		if (is_first) {
			queue_work(lp->wqueue, &lp->rxwork);
			is_first = false;
		}
	}
	/*printk(KERN_DEBUG "TX pkt len %u key 0x%04x parts %d", 
				skb->len, index, partnum+1);*/
	lp->netdev->stats.tx_packets++;
	lp->netdev->stats.tx_bytes += skb->len;
    return 0;
}
