// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 *  AMD Sensor Fusion Hub PCIe driver interface
 *
 *  Authors: Nehal Bakulchandra Shah <Nehal-Bakulchandra.Shah@amd.com>
 *           Richard Neumann <mail@richard-neumann.de>
 */

#ifndef AMD_SFH_PCI_H
#define AMD_SFH_PCI_H

#include <linux/pci.h>
#include <linux/types.h>

#define PCI_DEVICE_ID_AMD_SFH	0x15E4

/**
 * Sensor Fusion Hub communication registers
 */
enum {
	/* MP2 C2P Message Registers */
	AMD_C2P_MSG0 = 0x10500,		/* MP2 Message for I2C0 */
	AMD_C2P_MSG1 = 0x10504,		/* MP2 Message for I2C1 */
	AMD_C2P_MSG2 = 0x10508,		/* DRAM Address Lo / Data 0 */
	AMD_C2P_MSG3 = 0x1050c,		/* DRAM Address HI / Data 1 */
	AMD_C2P_MSG4 = 0x10510,		/* Data 2 */
	AMD_C2P_MSG5 = 0x10514,		/* Data 3 */
	AMD_C2P_MSG6 = 0x10518,		/* Data 4 */
	AMD_C2P_MSG7 = 0x1051c,		/* Data 5 */
	AMD_C2P_MSG8 = 0x10520,		/* Data 6 */
	AMD_C2P_MSG9 = 0x10524,		/* Data 7 */

	/* MP2 P2C Message Registers */
	AMD_P2C_MSG0 = 0x10680,		/* Do not use */
	AMD_P2C_MSG1 = 0x10684,		/* I2C0 interrupt register */
	AMD_P2C_MSG2 = 0x10688,		/* I2C1 interrupt register */
	AMD_P2C_MSG3 = 0x1068C,		/* MP2 debug info */
	AMD_P2C_MSG_INTEN = 0x10690,	/* MP2 interrupt gen register */
	AMD_P2C_MSG_INTSTS = 0x10694,	/* Interrupt status */
};

/**
 * The sensor indices on the AMD SFH device
 * @ACCEL_IDX:	Index of the accelerometer
 * @GYRO_IDX:	Index of the gyroscope
 * @MAG_IDX:	Index of the magnetometer
 * @ALS_IDX:	Index of the ambient light sensor
 */
enum sensor_idx {
	ACCEL_IDX = 0,
	GYRO_IDX,
	MAG_IDX,
	ALS_IDX = 19,
};

/**
 * SFH command IDs
 */
enum {
	AMD_SFH_CMD_NOOP = 0,
	AMD_SFH_CMD_ENABLE_SENSOR,
	AMD_SFH_CMD_DISABLE_SENSOR,
	AMD_SFH_CMD_DUMP_SENSOR_INFO,
	AMD_SFH_CMD_NUMBER_OF_SENSORS_DISCOVERED,
	AMD_SFH_CMD_WHOAMI_REGCHIPID,
	AMD_SFH_CMD_SET_DCD_DATA,
	AMD_SFH_CMD_GET_DCD_DATA,
	AMD_SFH_CMD_STOP_ALL_SENSORS,
	AMD_SFH_CMD_INVALID = 0xF,
};

/**
 * SFH command registers
 */
union amd_sfh_cmd {
	u32 ul;
	struct {
		u32 cmd_id : 8;
		u32 sensor_id : 8;
		u32 interval : 16;
	} s;
};

/**
 * SFH command parameters
 */
union amd_sfh_parm {
	u32 ul;
	struct {
		u32 buffer_layout : 2;
		u32 buffer_length : 6;
		u32 rsvd : 24;
	} s;
};

/**
 * SFH response types
 */
enum {
	AMD_SFH_RESP_NOOP,
	AMD_SFH_RESP_SUCCESS,
	AMD_SFH_RESP_FAILURE,
	AMD_SFH_RESP_SFI_DATA_READY,
	AMD_SFH_RESP_INVALID = 0xff,
};

/**
 * SFH status types
 */
enum {
	AMD_SFH_STATUS_SUCCESS,
	AMD_SFH_STATUS_INVALID_PAYLOAD_DATA,
	AMD_SFH_STATUS_INVALID_PAYLOAD_LENGTH,
	AMD_SFH_STATUS_INVALID_SENSOR_ID,
	AMD_SFH_STATUS_INVALID_DRAM_ADDR,
	AMD_SFH_STATUS_INVALID_CMD,
	AMD_SFH_STATUS_SENSOR_ENABLED,
	AMD_SFH_STATUS_SENSOR_DISABLED,
	AMD_SFH_STATUS_END,
};

/**
 * SFH event response data
 */
union amd_sfh_event {
	u32 ul;
	struct {
		u32 response : 4; /*!< bit: 0..3 SFI response_type */
		u32 status : 3; /*!< bit: 6..5 status_type */
		u32 out_in_c2p : 1; /*!< bit: 5 0- output in DRAM,1-in C2PMsg */
		u32 length : 6; /*!< bit: 8..13 length */
		u32 dbg : 2; /*!< bit: 14.15 dbg msg include in p2c msg 1-2 */
		u32 sensor_id : 8; /*!< bit: 16..23 Sensor ID */
		u32 rsvd : 8; /*!< bit: 24..31 Reservered for future use */
	} s;
};

/**
 * amd_sfh_response - Stores response data from the SFH PCI device.
 * @event:	The event data
 * @debuginfo1:			Debug information from AMD_P2C_MSG1
 * @debuginfo2:			Debug information from AMD_P2C_MSG2
 * @activecontrolstatus:	Debug information from AMD_P2C_MSG3
 */
struct amd_sfh_response {
	union amd_sfh_event event;
	u32 debuginfo1;
	u32 debuginfo2;
	u32 activecontrolstatus;
};

/**
 * struct amd_sfh_msg - A message interchanged with the PCI device.
 * @cmd:		The command to execute
 * @parm:		The command parameter
 * @dma_handle:		A DMA address to map
 * @response:		The SFH device's response
 * @pci_dev:		The PCI device that handled the message
 * @callback:		Callback to process the response
 */
struct amd_sfh_msg {
	union amd_sfh_cmd cmd;
	union amd_sfh_parm parm;
	dma_addr_t dma_handle;
	struct amd_sfh_response response;
	struct pci_dev *pci_dev;
	void (*callback)(struct amd_sfh_msg *msg);
};

/**
 * struct amd_sfh_dev - AMD SFH PCI device data
 * @pci_dev:		Handled PCI device
 * @mmio:		iommapped registers
 * @lock:		Mutex to lock the device for r/w actions
 */
struct amd_sfh_dev {
	struct pci_dev *pci_dev;
	void __iomem *mmio;
	struct mutex lock;
	struct amd_sfh_msg *msg;
};

/* SFH PCI driver interface functions */
int amd_sfh_get_sensor_mask(struct pci_dev *pci_dev);
int amd_sfh_start_sensor(struct pci_dev *pci_dev, enum sensor_idx sensor_idx,
			 dma_addr_t dma_handle, unsigned int interval);
int amd_sfh_stop_sensor(struct pci_dev *pci_dev, enum sensor_idx sensor_idx);
bool amd_sfh_pci_probed(struct pci_dev *pci_dev);

#endif
