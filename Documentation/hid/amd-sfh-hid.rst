.. SPDX-License-Identifier: GPL-2.0

AMD Sensor Fusion Hub:-
======================
AMD Sensor Fusion Hub is part of a SOC starting from Ryzen based platforms.
The solution is working well on several OEM products. AMD SFH uses HID over PCIe bus.
In terms of architecture it resembles ISH .However the major difference is all
The hid reports are generated as part of kernel driver.

Block Diagram:-
=============
	-------------------------------
	|  HID User Space Applications  |
	-------------------------------
---------------------------------------------
	 ---------------------------------
	|		HID Core          |
	 ---------------------------------

	 ---------------------------------
	|     AMD HID Transport           |
	 ---------------------------------

	 --------------------------------
	|             AMD HID Client     |
	|	with HID Report Generator|
	 --------------------------------

	 --------------------------------
	|     AMD MP2 PCIe Driver         |
	 --------------------------------
---------------------------------------------
	  -------------------------------
	|     SFH MP2 Processor         |
	 --------------------------------


AMD HID Transport Layer :-
***************************
AMD SFH transport is also implemented as a bus. Each client application executing in the AMD MP2
is registered as a device on this bus. MP2 which is an ARM® Cortex-M4 core based co-processor to
x86. The driver, which binds each device (AMD SFH HID driver) identifies the device type and
registers with the hid core. Transport drivers attach a constant "struct hid_ll_driver" object with
each device. Once a device is registered with HID core, the callbacks provided via this struct are
used by HID core to communicate with the device. AMD HID Transport driver implements the synchronouscalls.

AMD HID Client Layer:-
**********************
This layer is responsible to implement HID request and descriptors. As firmware is OS agnostic,
HID client layer fills the HID request structure and descriptors. HID client layer is in complex
in nature as it is interface between MP2 PCIe driver and HID. HID client layer initialized
the MP2 PCIe driver and holds the instance of MP2 driver.It identified the number of sensors
connected using MP2-PCIe driver and based on that allocate the DRAM address for each and every
sensor and pass it to MP2-PCIe driver.On enumeration of each sensor, client layer fills out the HID
Descriptor structure and HID input report structure. HID Feature report structure can be optional.
The report descriptor structure varies sensor to sensor. Now on enumeration client layer does
two major things
1.	Register the HID sensor client to virtual bus (Platform driver) and bind it.
2.	Probes the AMD HID transport driver. Which in turns register device to the core.

AMD MP2 PCIe layer:-
********************
MP2 PCIe Layer is responsible for making all transaction with the firmware over PCIe.
The connection establishment between firmware and PCIe happens here.

The communication between X86 and MP2 is spilt into three parts.
1. Command Transfer => C2P Mailbox Register are used
2. Data Transfer => DRAM
3. Supported sensor info => P2C Register

Commands are sent to MP2 using C2P Mail Box registers. These C2P  registers are common between x86
and MP2. Writing into C2P Message register generate interrupt to MP2.  The client layer allocates
the physical memory and send the same to MP2 for data transfer. MP2 firmware uses HUBIF interface
to access DRAM memory. For Firmware always write minimum 32 bytes into DRAM.So it is expected that
driver shall allocate minimum 32 bytes DRAM space.

Enumeration and Probing flow:-
*****************************
       HID             AMD            AMD                       AMD -PCIe             MP2
       Core         Transport      Client layer                   layer                FW
	|		|	       |                           |                 |
	|		|              |                 on Boot Driver Loaded       |
	|		|	       |                           |                 |
	|		|	       |                        MP2-PCIe Int         |
	|		|              |			   |                 |
	|		|	       |---Get Number of sensors-> |                 |
	|		|              |                       Read P2C              |
	|		|	       |			Register             |
	|		|              |                           |                 |
	|               |              | Loop(for No of Sensors)   |                 |
	|		|	       |----------------------|    |                 |
	|		|              | Create HID Descriptor|    |                 |
	|		|	       | Create Input  report |    |                 |
	|		|              |  Descriptor Map      |    |                 |
	|		|	       |  the MP2 FW Index to |    |                 |
	|		|              |   HID Index          |    |                 |
	|		|	       | Allocate the DRAM    |  Enable              |
        |		|	       |	address       |  Sensors             |
	|		|              |----------------------|    |                 |
	|		| HID transport|                           |    Enable       |
	|	        |<--Probe------|                           |---Sensor CMD--> |
	|		| Create the   |			   |                 |
	|		| HID device   |                           |                 |
	|               |    (MFD)     |                           |                 |
	|		| by Populating|			   |                 |
        |               |  the HID     |                           |                 |
	|               |  ll_driver   |                           |                 |
	| HID           |	       |			   |                 |
	|  add          |              |                           |                 |
	|Device         |              |                           |                 |
	|<------------- |	       |			   |                 |


Data Flow from Application to the AMD SFH Driver:-
*************************************************

	|	       |              |	  	 	          |		    |
Get   	|	       |	      |			          |                 |
Input 	|	       |	      |			          |                 |
Report	|              |              |                           |                 |
--->  	|              |              |                           |                 |
	|HID_req       |              |                           |                 |
	|get_report    |              |                           |                 |
	|------------->|              |                           |                 |
	|              | HID_get_input|                           |                 |
	|              |  report      |                           |                 |
	|              |------------->|------------------------|  |                 |
	|              |              |  Read the DRAM data for|  |                 |
	|              |              |  requested sensor and  |  |                 |
	|              |              |  create the HID input  |  |                 |
	|              |              |  report                |  |                 |
	|              |              |------------------------|  |                 |
	|              |Data received |                           |                 |
	|              | in HID report|                           |                 |
 To	|<-------------|<-------------|                           |                 |
Applications           |              |                           |                 |
<-------|              |              |                           |                 |


Data Flow from AMD SFH Driver to Application:-
**********************************************
      |		  |               |	            	          |		    |
      |           |               |------------------------|      |                 |
      |           |               |Periodically Read       |      |                 |
      |           |               |the data for all        |      |                 |
      |           |               |enumerated sensors      |      |                 |
      |           |               |from the dram and create|      |                 |
      |           |               | HID Input reports      |      |                 |
      |           |               |------------------------|      |                 |
      |           |HID Input      |                               |                 |
      |           |Input report   |                               |                 |
   <----submit to Application-----|                               |                 |
      |           |               |                               |                 |
