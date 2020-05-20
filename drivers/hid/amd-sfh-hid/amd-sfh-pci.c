// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * AMD Sensor Fusion Hub (SFH) PCIe driver
 *
 * Authors: Shyam Sundar S K <Shyam-sundar.S-k@amd.com>
 *          Nehal Bakulchandra Shah <Nehal-bakulchandra.Shah@amd.com>
 *          Richard Neumann <mail@richard-neumann.de>
 */

#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/types.h>

#include "amd-sfh-pci.h"

/**
 * amd_sfh_register_msg - Registers a message.
 * @privdata:	The driver data
 * @msg:	The message
 *
 * Registers a to-be-processed message with the device data.
 */
static void amd_sfh_register_msg(struct amd_sfh_dev *privdata,
				 struct amd_sfh_msg *msg)
{
	mutex_lock(&privdata->lock);
	privdata->msg = msg;
}

/**
 * amd_sfh_unregister_msg - Unregisters a message.
 * @privdata:	The driver data
 *
 * Un-registers a processed message from the device data.
 */
static void amd_sfh_unregister_msg(struct amd_sfh_dev *privdata)
{
	privdata->msg = NULL;
	mutex_unlock(&privdata->lock);
}

/**
 * amd_sfh_causes_interrupt
 * @cmd_id:	The command to check
 *
 * Checks if the command is expected to cause an interrupt.
 */
static bool amd_sfh_causes_interrupt(unsigned int cmd_id)
{
	switch (cmd_id) {
	case AMD_SFH_CMD_NOOP:
	case AMD_SFH_CMD_ENABLE_SENSOR:
	case AMD_SFH_CMD_DISABLE_SENSOR:
	case AMD_SFH_CMD_DUMP_SENSOR_INFO:
		return false;
	case AMD_SFH_CMD_NUMBER_OF_SENSORS_DISCOVERED:
	case AMD_SFH_CMD_WHOAMI_REGCHIPID:
	case AMD_SFH_CMD_SET_DCD_DATA:
	case AMD_SFH_CMD_GET_DCD_DATA:
	case AMD_SFH_CMD_STOP_ALL_SENSORS:
		return true;
	default:
		return false;
	}
}

/**
 * amd_sfh_run_callback - Runs the callback on a message.
 * @msg:	The message
 *
 * If no callback was set, the message will be freed.
 */
static void amd_sfh_run_callback(struct amd_sfh_msg *msg)
{
	if (msg->callback)
		msg->callback(msg);
	else
		kfree(msg);
}

/**
 * amd_sfh_exec - Executes a command on the SFH.
 * @msg:	The message to send to the PCI device
 *
 * Acquires lock to prevent race conditions while writing to the device.
 * If the sent command is AMD_SFH_CMD_ENABLE_SENSOR or
 * AMD_SFH_CMD_DISABLE_SENSOR, writes the shipped DMA address to
 * AMD_C2P_MSG2 first.
 * Writes the command parameters to AMD_C2P_MSG1 and the command
 * to AMD_C2P_MSG0.
 *
 * If the command is not expected to cause an interrupt, the lock is
 * released immediately after writing to the device and the message's
 * callback run.
 */
static void amd_sfh_exec(struct amd_sfh_dev *privdata, struct amd_sfh_msg *msg)
{
	amd_sfh_register_msg(privdata, msg);

	if (amd_sfh_causes_interrupt(msg->cmd.s.cmd_id))
		writel(1, privdata->mmio + AMD_P2C_MSG_INTEN);

	switch (msg->cmd.s.cmd_id) {
	case AMD_SFH_CMD_ENABLE_SENSOR:
	case AMD_SFH_CMD_DISABLE_SENSOR:
		writeq(msg->dma_handle, privdata->mmio + AMD_C2P_MSG2);
		break;
	}

	writel(msg->parm.ul, privdata->mmio + AMD_C2P_MSG1);
	writel(msg->cmd.ul, privdata->mmio + AMD_C2P_MSG0);

	if (!amd_sfh_causes_interrupt(msg->cmd.s.cmd_id)) {
		amd_sfh_unregister_msg(privdata);
		amd_sfh_run_callback(msg);
	}
}

/**
 * amd_sfh_get_sensor_mask - Returns the sensors mask.
 * @pci_dev:	The Sensor Fusion Hub PCI device
 *
 * Returns an integer representing the bitmask to match
 * the sensors connected to the Sensor Fusion Hub.
 */
int amd_sfh_get_sensor_mask(struct pci_dev *pci_dev)
{
	int sensor_mask;
	struct amd_sfh_dev *privdata = pci_get_drvdata(pci_dev);

	sensor_mask = readl(privdata->mmio + AMD_P2C_MSG3);
	/* Correct bit shift in firmware register */
	sensor_mask = sensor_mask >> 4;

	if (!sensor_mask)
		pci_err(pci_dev, "[Firmware Bug]: No sensors marked active!\n");

	return sensor_mask;
}
EXPORT_SYMBOL_GPL(amd_sfh_get_sensor_mask);

/**
 * amd_sfh_start_sensor- Starts the respective sensor.
 * @pci_dev:	The Sensor Fusion Hub PCI device
 * @sensor_idx:	The sensor's index
 * @dma_handle:	The DMA handle
 *
 * Return 0 on success and nonzero on errors.
 */
int amd_sfh_start_sensor(struct pci_dev *pci_dev, enum sensor_idx sensor_idx,
			 dma_addr_t dma_handle, unsigned int interval)
{
	struct amd_sfh_dev *privdata;
	struct amd_sfh_msg *msg;

	privdata = pci_get_drvdata(pci_dev);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->cmd.ul = 0;
	msg->cmd.s.cmd_id = AMD_SFH_CMD_ENABLE_SENSOR;
	msg->cmd.s.interval = interval;
	msg->cmd.s.sensor_id = sensor_idx;

	msg->parm.ul = 0;
	msg->parm.s.buffer_layout = 1;
	msg->parm.s.buffer_length = 16;

	msg->dma_handle = dma_handle;
	msg->callback = NULL;

	amd_sfh_exec(privdata, msg);
	return 0;
}
EXPORT_SYMBOL_GPL(amd_sfh_start_sensor);

/**
 * amd_sfh_stop_sensor- Stops the respective sensor.
 * @pci_dev:	The Sensor Fusion Hub PCI device
 * @sensor_idx:	The sensor's index
 *
 * Return 0 on success and nonzero on errors.
 */
int amd_sfh_stop_sensor(struct pci_dev *pci_dev, enum sensor_idx sensor_idx)
{
	struct amd_sfh_dev *privdata;
	struct amd_sfh_msg *msg;

	privdata = pci_get_drvdata(pci_dev);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->cmd.ul = 0;
	msg->cmd.s.cmd_id = AMD_SFH_CMD_DISABLE_SENSOR;
	msg->cmd.s.interval = 0;
	msg->cmd.s.sensor_id = sensor_idx;

	msg->parm.ul = 0;
	msg->dma_handle = 0x0;
	msg->callback = NULL;

	amd_sfh_exec(privdata, msg);
	return 0;
}
EXPORT_SYMBOL_GPL(amd_sfh_stop_sensor);

/**
 * amd_sfh_pci_probed - Checks whether the PCI driver was probed successfully.
 * @pci_dev:	The Sensor Fusion Hub PCI device
 */
bool amd_sfh_pci_probed(struct pci_dev *pci_dev)
{
	struct amd_sfh_dev *privdata = pci_get_drvdata(pci_dev);

	if (privdata)
		return true;

	return false;
}
EXPORT_SYMBOL_GPL(amd_sfh_pci_probed);

/**
 * amd_sfh_stop_all_sensors- Stops all sensors on the SFH.
 * @pci_dev:	The Sensor Fusion Hub PCI device
 *
 * Return 0 on success and nonzero on errors.
 */
static int amd_sfh_stop_all_sensors(struct pci_dev *pci_dev)
{
	struct amd_sfh_dev *privdata;
	struct amd_sfh_msg *msg;

	privdata = pci_get_drvdata(pci_dev);

	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;

	msg->cmd.ul = 0;
	msg->cmd.s.cmd_id = AMD_SFH_CMD_STOP_ALL_SENSORS;
	msg->cmd.s.interval = 0;
	msg->cmd.s.sensor_id = 0;

	msg->parm.ul = 0;
	msg->dma_handle = 0;
	msg->callback = NULL;

	amd_sfh_exec(privdata, msg);
	return 0;
}

/**
 * amd_sfh_reset_interrupts - Resets the interrupt registers.
 * @privdata:	The driver data
 */
static void amd_sfh_reset_interrupts(struct amd_sfh_dev *privdata)
{
	int inten;

	inten = readl(privdata->mmio + AMD_P2C_MSG_INTEN);
	if (inten)
		writel(0, privdata->mmio + AMD_P2C_MSG_INTEN);
}

/**
 * amd_sfh_clear_registers - Clears the C2P and P2C registers.
 * @privdata:	The driver data
 */
static void amd_sfh_clear_registers(struct amd_sfh_dev *privdata)
{
	unsigned int reg;

	/* Clear C2P registers */
	for (reg = AMD_C2P_MSG0; reg <= AMD_C2P_MSG9; reg += 4)
		writel(0, privdata->mmio + reg);

	/* Clear P2C registers */
	for (reg = AMD_P2C_MSG0; reg <= AMD_P2C_MSG2; reg += 4)
		writel(0, privdata->mmio + reg);
}

/**
 * amd_sfh_irq_isr - Handles interrupts.
 * @irq:	The IRQ received
 * @dev:	The driver data
 *
 * XXX: Disables IRQ handling to prevent IRQ flooding.
 * Reads response information from relevant P2C registers.
 * Releases lock to allow next command to be executed.
 */
static irqreturn_t amd_sfh_irq_isr(int irq, void *dev)
{
	struct amd_sfh_msg *msg;
	struct amd_sfh_response *response;
	struct amd_sfh_dev *privdata = dev;

	amd_sfh_reset_interrupts(privdata);

	if (!privdata->msg) {
		pci_warn(privdata->pci_dev,
			 "Received interrupt, but no message was sent.\n");
		return IRQ_HANDLED;
	}

	/* Update message data */
	msg = privdata->msg;
	msg->pci_dev = privdata->pci_dev;
	response = &msg->response;

	/* Read response registers */
	response->event.ul = readl(privdata->mmio + AMD_P2C_MSG0);
	response->debuginfo1 = readl(privdata->mmio + AMD_P2C_MSG1);
	response->debuginfo2 = readl(privdata->mmio + AMD_P2C_MSG2);
	response->activecontrolstatus = readl(privdata->mmio + AMD_P2C_MSG3);

	amd_sfh_clear_registers(privdata);
	amd_sfh_unregister_msg(privdata);
	amd_sfh_run_callback(msg);

	return IRQ_HANDLED;
}

/**
 * amd_sfh_pci_init - Initializes the PCI device.
 * @privdata:	The PCI driver data
 * @pci_dev:	The PCI device
 *
 * Enables the PCI device, performs io mapping and sets up the IRQ handler.
 * Returns 0 on success or nonzero on errors.
 */
static int amd_sfh_pci_init(struct amd_sfh_dev *privdata,
			    struct pci_dev *pci_dev)
{
	int rc;

	rc = pcim_enable_device(pci_dev);
	if (rc)
		goto err_pci_enable;

	rc = pcim_iomap_regions(pci_dev, BIT(2), pci_name(pci_dev));
	if (rc)
		goto err_pci_enable;

	privdata->pci_dev = pci_dev;
	privdata->mmio = pcim_iomap_table(pci_dev)[2];
	mutex_init(&privdata->lock);
	privdata->msg = NULL;
	pci_set_master(pci_dev);

	rc = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(64));
	if (rc) {
		rc = pci_set_dma_mask(pci_dev, DMA_BIT_MASK(32));
		if (rc)
			goto clear_master;
	}

	amd_sfh_reset_interrupts(privdata);
	rc = devm_request_irq(&pci_dev->dev, pci_dev->irq, amd_sfh_irq_isr,
			      IRQF_SHARED, pci_name(pci_dev), privdata);
	if (rc) {
		pci_err(pci_dev, "Failure requesting irq %i: %d\n",
			pci_dev->irq, rc);
		goto clear_master;
	}

	pci_set_drvdata(pci_dev, privdata);
	pci_info(pci_dev, "AMD SFH device initialized\n");

	return 0;

clear_master:
	pci_clear_master(pci_dev);
err_pci_enable:
	pci_set_drvdata(pci_dev, NULL);
	return rc;
}

/**
 * amd_sfh_pci_probe - Probes the PCI device driver.
 * @pci_dev:	The handled PCI device
 * @id:		The PCI device ID
 *
 * Returns 0 on success or nonzero on errors.
 */
static int amd_sfh_pci_probe(struct pci_dev *pci_dev,
			     const struct pci_device_id *id)
{
	struct amd_sfh_dev *privdata;

	privdata = devm_kzalloc(&pci_dev->dev, sizeof(*privdata), GFP_KERNEL);
	if (!privdata)
		return -ENOMEM;

	return amd_sfh_pci_init(privdata, pci_dev);
}

/**
 * amd_sfh_pci_remove - Unloads the PCI device driver.
 * @pci_dev:	The PCI device
 */
static void amd_sfh_pci_remove(struct pci_dev *pci_dev)
{
	int rc;
	struct amd_sfh_dev *privdata = pci_get_drvdata(pci_dev);

	rc = amd_sfh_stop_all_sensors(privdata->pci_dev);
	if (rc)
		pci_err(pci_dev, "Could not stop all sensors!\n");

	pci_clear_master(pci_dev);
	amd_sfh_reset_interrupts(privdata);
	amd_sfh_clear_registers(privdata);
}

static const struct pci_device_id amd_sfh_pci_tbl[] = {
	{ PCI_VDEVICE(AMD, PCI_DEVICE_ID_AMD_SFH) },
	{ }
};
MODULE_DEVICE_TABLE(pci, amd_sfh_pci_tbl);

static struct pci_driver amd_sfh_pci_driver = {
	.name		= "amd-sfh-pci",
	.id_table	= amd_sfh_pci_tbl,
	.probe		= amd_sfh_pci_probe,
	.remove		= amd_sfh_pci_remove,
};
module_pci_driver(amd_sfh_pci_driver);

MODULE_DESCRIPTION("AMD(R) Sensor Fusion Hub PCI driver");
MODULE_AUTHOR("Shyam Sundar S K <Shyam-sundar.S-k@amd.com>");
MODULE_AUTHOR("Nehal Bakulchandra Shah <Nehal-bakulchandra.Shah@amd.com>");
MODULE_AUTHOR("Richard Neumann <mail@richard-neumann.de>");
MODULE_LICENSE("Dual BSD/GPL");
