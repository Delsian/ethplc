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
#include "pl360_firmware.h"

/* ! Reset pin of transceiver */
#define PLC_RST_DELAY                (100)
/** SPI Header field when bootload is in the other side of spi*/
#define PLC_SPI_HEADER_BOOT					0x5634
/** SPI Header MASK for bootloader heade*/
#define PLC_SPI_HEADER_BOOT_MASK				0xFFFe
/** SPI Header field when atpl360 is in the other side of spi*/
#define PLC_SPI_HEADER_CORTEX					0x1022
/** Bootloader Passwords for enable writing  */
#define ATPL360_BOOT_WRITE_KEY              0x5345ACBA
/** Bootloader Address of CPUWAIT */
#define ATPL360_BOOT_CMD_ENABLE_WRITE       0xDE05
/** Bootloader Address for writing program */
#define ATPL360_BOOT_PROGRAM_ADDR           0x00000000
/** Bootloader command: Write Word (32 bits) */
#define ATPL360_BOOT_CMD_WRITE_WORD         0x0000
/** Bootloader command: Write buffer */
#define ATPL360_BOOT_CMD_WRITE_BUF          0x0001
/** Bootloader command: Disable SPI control to bootloader */
#define ATPL360_BOOT_CMD_DIS_SPI_CTRL       0xA55A


static int pl360_reset(struct pl360_local *lp) {
	int err;

    gpio_set_value(lp->gpio_ldo, 0);
	msleep(PLC_RST_DELAY);
    gpio_set_value(lp->gpio_nrst, 0);
    gpio_set_value(lp->gpio_ldo, 1);

	lp->spi_transfer.len = 1;
	err = spi_sync(lp->spi, &lp->spi_msg);
	if(err<0) {
		dev_crit(&lp->spi->dev,"SPI error %d\n", err);
		return err;
	}
	msleep(10);
	gpio_set_value(lp->gpio_nrst, 1);
	msleep(50);
	return 0;
}

static int pl360_boot_pkt(struct pl360_local *lp, 
		uint16_t cmd, uint32_t addr, uint16_t data_len, uint8_t *data_buf) {
	struct spi_device *spi = lp->spi;
	int err;

	memcpy(lp->tx_buffer, &addr, sizeof(uint32_t));
	memcpy(&(lp->tx_buffer[4]), &cmd, sizeof(uint16_t));
	memcpy(&(lp->tx_buffer[6]), data_buf, data_len);
	err = spi_write(spi, lp->tx_buffer, data_len + 6);
	if(err<0)
		dev_crit(&spi->dev,"SPI boot err %d \n", err);
	return err;
}

static int pl360_booload(struct pl360_local *lp) {
	int err;
	uint8_t buf[4];
	pl360_reset(lp);

	buf[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	buf[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	buf[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	buf[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	err = pl360_boot_pkt(lp, ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(buf), buf);
	if (err < 0) goto err_boot;
	buf[3] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 8);
	buf[2] = (uint8_t)(ATPL360_BOOT_WRITE_KEY);
	buf[1] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 24);
	buf[0] = (uint8_t)(ATPL360_BOOT_WRITE_KEY >> 16);
	err = pl360_boot_pkt(lp, ATPL360_BOOT_CMD_ENABLE_WRITE, 0, sizeof(buf), buf);
	if (err < 0) goto err_boot;
/* Send CPU Wait Cmd */
 	uint32_t reg_value = ATPL360_MISCR_CPUWAIT | ATPL360_MISCR_PPM_CALIB_OFF | ATPL360_MISCR_MEM_96_96_CFG |
						 ATPL360_MISCR_EN_ACCESS_ERROR | ATPL360_MISCR_SET_GPIO_12_ZC;
	buf[3] = (uint8_t)(reg_value >> 24);
	buf[2] = (uint8_t)(reg_value >> 16);
	buf[1] = (uint8_t)(reg_value >> 8);
	buf[0] = (uint8_t)(reg_value);
	err = pl360_boot_pkt(lp, ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_MISCR, sizeof(buf), buf);
	if (err < 0) goto err_boot;
	// Load FW
	uint32_t fw_pending_len = sizeof(pl360_firmware);
	uint8_t *fw_ptr = (uint8_t *)pl360_firmware;
	uint32_t fw_prog_addr  = ATPL360_BOOT_PROGRAM_ADDR;
	uint16_t fw_fragment_len;
	while (fw_pending_len) {
		if (fw_pending_len > PDC_SPI_FUP_BUFFER_SIZE) {
			fw_fragment_len = PDC_SPI_FUP_BUFFER_SIZE;
			fw_pending_len -= fw_fragment_len;
		} else {
			fw_fragment_len = fw_pending_len;
			fw_pending_len = 0;
			fw_fragment_len += (4 - (fw_fragment_len & 3)) & 3; // padding
		}
		/* Write fw block data */
		err = pl360_boot_pkt(lp, ATPL360_BOOT_CMD_WRITE_BUF, fw_prog_addr, fw_fragment_len, fw_ptr);
		if (err < 0) goto err_boot;
		/* Update counters */
		fw_ptr += fw_fragment_len;
		fw_prog_addr += fw_fragment_len;
	}

	// Disable CpuWait
	reg_value = ATPL360_MISCR_PPM_CALIB_OFF | ATPL360_MISCR_MEM_96_96_CFG | ATPL360_MISCR_EN_ACCESS_ERROR | ATPL360_MISCR_SET_GPIO_12_ZC;
	buf[3] = (uint8_t)(reg_value >> 24);
	buf[2] = (uint8_t)(reg_value >> 16);
	buf[1] = (uint8_t)(reg_value >> 8);
	buf[0] = (uint8_t)(reg_value);
	err = pl360_boot_pkt(lp, ATPL360_BOOT_CMD_WRITE_WORD, ATPL360_MISCR, sizeof(uint32_t), buf);
	if (err < 0) goto err_boot;
	// Give control of the MISO signal to M7-SPI
	err = pl360_boot_pkt(lp, ATPL360_BOOT_CMD_DIS_SPI_CTRL, 0, 0, NULL);

err_boot:
	if (err < 0) {
		dev_crit(&lp->spi->dev,	"Error during bootload: %d\n", err);
	} else {
		msleep(10);
	}
	return err;
}

static void pl360_check_status(struct pl360_local *lp, uint32_t st) {
	uint16_t id;
	id = st & 0xFEFF;

	/* Check who is in the other side (bootloader / atpl360) */
	if (PLC_SPI_HEADER_BOOT == id) {
		//printk("*** PLC Status boot\n");
		pl360_booload(lp);
	} else if (PLC_SPI_HEADER_CORTEX == id) {
		lp->events = st >> 16;
	} else {
		/* Unexpected ID value -> Reset HW ATPL360 */
		//printk("*** PLC Status unknown %04x\n", id);
		pl360_booload(lp);
	}
}

static void pktwr(struct pl360_local *lp, plc_pkt_t* pkt) {
    uint16_t ptr = 0;
    // change word seq to bytes
    while( ptr < pkt->len) {
        lp->tx_buffer[ptr+4] = pkt->buf[ptr+1];
        lp->tx_buffer[ptr+5] = pkt->buf[ptr];
        ptr += 2;
    }
}

static void pktrd(struct pl360_local *lp, plc_pkt_t* pkt) {
    uint16_t ptr = 0;
    // change word seq to bytes
    while( ptr < pkt->len) {
        pkt->buf[ptr+1] = lp->rx_buffer[ptr+4];
        pkt->buf[ptr] = lp->rx_buffer[ptr+5];
        ptr += 2;
    }
}

int pl360_hw_init(struct pl360_local *lp)
{
    int ret;
    struct spi_device *spi = lp->spi;
	dev_dbg(&spi->dev, "%s called\n", __func__);
    
	spi_message_init(&lp->spi_msg);
	lp->spi_transfer.len = 1;
	lp->spi_transfer.tx_buf = lp->tx_buffer;
	lp->spi_transfer.rx_buf = lp->rx_buffer;

	spi_message_add_tail(&lp->spi_transfer, &lp->spi_msg);

	lp->gpio_nrst = of_get_named_gpio(spi->dev.of_node,	"nrst-gpio",0);
    ret = gpio_direction_output(lp->gpio_nrst, 1);
	if (ret < 0) {
		dev_crit(&spi->dev,	"Reset GPIO %d did not set to output mode\n",
			lp->gpio_nrst);
		goto err_gpio;
	}
    lp->gpio_ldo = of_get_named_gpio(spi->dev.of_node,"ldo-gpio",0);
    ret = gpio_direction_output(lp->gpio_ldo, 1);
	if (ret < 0) {
		dev_crit(&spi->dev,	"LDO GPIO %d did not set to output mode\n",
			lp->gpio_ldo);
		goto err_gpio;
	}

	lp->gpio_irq = of_get_named_gpio(spi->dev.of_node,"irq-gpio",0);

	lp->irq_id = gpio_to_irq(lp->gpio_irq);
	if (lp->irq_id < 0) {
		dev_crit(&spi->dev,"Could not get irq for gpio pin %d\n",
			lp->gpio_irq);
		gpio_free(lp->gpio_irq);
		ret = -EIO;
		goto err_gpio;
	}
    //printk("GPIOS %d %d %d\n",lp->gpio_nrst,lp->gpio_ldo,lp->gpio_irq);

    pl360_reset(lp);
	pl360_conrigure(lp);
err_gpio:
    return ret;
}

void pl360_datapkt(struct pl360_local *lp, bool cmd, plc_pkt_t* pkt)
{
	int err;
	uint16_t us_len_wr_rd = (((pkt->len + 1) >> 1) & PLC_LEN_MASK) | (cmd << PLC_WR_RD_POS);

	//printk("PLC len %d cmd %x addr %x", pkt->len, cmd, pkt->addr);
	/* Check length */
	if (!pkt->len) {
		return;
	}

	/** Configure PLC Tx buffer **/
	/* Address */
	lp->tx_buffer[0] = (uint8_t)(pkt->addr >> 8);
	lp->tx_buffer[1] = (uint8_t)(pkt->addr);
	/* Length & read/write */
	lp->tx_buffer[2] = (uint8_t)(us_len_wr_rd >> 8);
	lp->tx_buffer[3] = (uint8_t)(us_len_wr_rd);

	if (cmd == PLC_CMD_WRITE) {
		pktwr(lp, pkt);
	} else {
		memset(&lp->tx_buffer[PDC_SPI_HEADER_SIZE], 0, pkt->len);
	}

	lp->spi_transfer.len = pkt->len + PDC_SPI_HEADER_SIZE;
	if (lp->spi_transfer.len % 2)	{
		lp->spi_transfer.len++;
	}
	err = spi_sync(lp->spi, &lp->spi_msg);
	if(err<0) {
		printk("SPI error %d\n", err);
	}

	if (cmd == PLC_CMD_READ) {
		pktrd(lp, pkt);
	}

	uint32_t status = (lp->rx_buffer[2] << 24) + (lp->rx_buffer[3] << 16) +
		(lp->rx_buffer[0] << 8) + lp->rx_buffer[1];
	pl360_check_status(lp, status);
}
