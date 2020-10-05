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

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/slab.h>     /* kmalloc() */
#include <linux/ethtool.h>
#include "ethplc.h"

#define DRV_NAME	"pl360"
#define DRV_VERSION	"1.01a"

/* use ethtool to change the level for any given device */
static struct {
	u32 msg_enable;
} debug = { -1 };

static const struct net_device_ops pl360_netdev_ops = {
	.ndo_open		= ops_pl360_start,
	.ndo_stop		= ops_pl360_stop,
	.ndo_start_xmit		= ops_pl360_xmit,
};

/* ......................... ETHTOOL SUPPORT ........................... */

static void
pl360_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRV_NAME, sizeof(info->driver));
	strlcpy(info->version, DRV_VERSION, sizeof(info->version));
	strlcpy(info->bus_info,
		dev_name(dev->dev.parent), sizeof(info->bus_info));
}

static int
pl360_get_link_ksettings(struct net_device *dev,
			    struct ethtool_link_ksettings *cmd)
{
	struct pl360_local *priv = netdev_priv(dev);

	ethtool_link_ksettings_zero_link_mode(cmd, supported);
	ethtool_link_ksettings_add_link_mode(cmd, supported, 10baseT_Half);
	ethtool_link_ksettings_add_link_mode(cmd, supported, TP);

	cmd->base.speed = SPEED_10;
	cmd->base.duplex = DUPLEX_HALF;
	cmd->base.port	= PORT_TP;
	cmd->base.autoneg = AUTONEG_DISABLE;

	return 0;
}

static u32 pl360_get_msglevel(struct net_device *dev)
{
	struct pl360_local *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void pl360_set_msglevel(struct net_device *dev, u32 val)
{
	struct pl360_local *priv = netdev_priv(dev);
	priv->msg_enable = val;
}

static int
pl360_set_link_ksettings(struct net_device *dev,
			    const struct ethtool_link_ksettings *cmd)
{
	return 0;
}

static const struct ethtool_ops pl360_ethtool_ops = {
	.get_drvinfo	= pl360_get_drvinfo,
	.get_msglevel	= pl360_get_msglevel,
	.set_msglevel	= pl360_set_msglevel,
	.get_link_ksettings = pl360_get_link_ksettings,
	.set_link_ksettings = pl360_set_link_ksettings,
};

static int pl360_probe(struct spi_device *spi)
{
	const char *mac_addr;
	struct net_device *dev;
	struct pl360_local *priv;
	int ret;

    dev = alloc_etherdev(sizeof(struct pl360_local));
	if (!dev) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	priv = netdev_priv(dev);

	priv->netdev = dev;	/* priv to netdev reference */
	priv->spi = spi;	/* priv to spi reference */

	mutex_init(&priv->bmux);

	spi_set_drvdata(spi, priv);
    SET_NETDEV_DEV(dev, &spi->dev);

	/* set kernel MAC address to dev */
	mac_addr = of_get_mac_address(dev->of_node);
	if (!IS_ERR(mac_addr))
		ether_addr_copy(dev->dev_addr, mac_addr);
	else
		eth_hw_addr_random(dev);

	priv->wqueue = create_singlethread_workqueue(dev_name(&priv->spi->dev));
	if (unlikely(!priv->wqueue)) {
		ret = -ENOMEM;
		goto err_hw_init;
	}
	INIT_WORK(&priv->rxwork, pl360_handle_rx_work);

	ret = pl360_hw_init(priv);
	if (ret) {
		printk("pl360 HW err %d\n", ret);
		goto err_hw_init;
	}

	dev->if_port = IF_PORT_10BASET;
	dev->irq = spi->irq;
	dev->netdev_ops = &pl360_netdev_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->ethtool_ops = &pl360_ethtool_ops;

	ret = register_netdev(dev);
	if (ret) {
		if (netif_msg_probe(priv))
			dev_err(&spi->dev, "register netdev failed (ret = %d)\n",
				ret);
		goto error_register;
	}

	//pl360_debugfs_init(lp);

	return ret;

error_register:
	free_irq(spi->irq, priv);
err_hw_init:
	mutex_destroy(&lp->bmux);
	free_netdev(lp->netdev);

error_alloc:
	return ret;
}

static int pl360_remove(struct spi_device *spi)
{
	struct pl360_local *lp = spi_get_drvdata(spi);

	dev_info(&spi->dev,"remove pl360\n");

	debugfs_remove_recursive(lp->debugfs_root);

	unregister_netdev(lp->netdev);
    destroy_workqueue(lp->wqueue);
	mutex_destroy(&lp->bmux);
	free_netdev(lp->netdev);

	return 0;
}

static const struct of_device_id pl360_of_match[] = {
	{ .compatible = "atmel,pl360", },
	{ .compatible = "atmel,pl360a", },
	{ },
};
MODULE_DEVICE_TABLE(of, pl360_of_match);

static const struct spi_device_id pl360_device_id[] = {
	{ .name = "pl360", },
    { .name = "pl360a", },
	{ },
};
MODULE_DEVICE_TABLE(spi, pl360_device_id);

static struct spi_driver pl360_driver = {
	.id_table = pl360_device_id,
	.driver = {
		   .of_match_table = of_match_ptr(pl360_of_match),
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = pl360_probe,
	.remove = pl360_remove,
};

module_spi_driver(pl360_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Eug Krashtan <eug.krashtan@gmail.com>");
MODULE_DESCRIPTION(DRV_NAME "Ethernet emulator Driver");