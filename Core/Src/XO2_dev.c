/*
 *  COPYRIGHT (c) 2012 by Lattice Semiconductor Corporation
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/** @file XO2_dev.c 
 * Database of needed parameters per XO2 device type for use in programming
 * the Configuration and UFM sectors.
 */
#define XO2DEVLIST_DECLARATION

#include "XO2_dev.h"

MachSTM_Handle_t machSTM_hdl;

/**
 * Database of XO2 device parameters needed for accessing, erasing and programming 
 * the Configuration and UFM sectors in different sized XO2 parts.
 */
const XO2DevInfo_t XO2DevList[LATTICE_XO2_NUM_DEVS] = 
{
	// Name            Cfg pgs 
	//                  |     UFM pgs 
	//                  |       |   Cfg erase time  
	//                  |       |    |     UFM erase time
	//                  |       |    |      |     Trefresh time
	{"MachXO2-256",    575,     0,  700,    0,      1},
	{"MachXO2-640",   1152,   191,  1100, 600,      1},
	{"MachXO2-640U",  2175,   512,  1400, 700,      1},
	{"MachXO2-1200",  2175,   512,  1400, 700,      1},
	{"MachXO2-1200U", 3200,   639,  1900, 900,      2},
	{"MachXO2-2000",  3200,   639,  1900, 900,		2},
	{"MachXO2-2000U", 5760,   767,  3100, 1000,		3},
	{"MachXO2-4000",  5760,   767,  3100, 1000,		3},
	{"MachXO2-7000",  9216,  2046,  4800, 1600,		4}
}; 

const XO2FpgaInfo_t XO2FpgaList[LATTICE_FPGA_INSTALLED_NUM_DEVS] = 
{
	// Name            MUX IDX 
	//                   |     MUX IDX
	//                   |             FPGA DEV IDX  
	//                   |        |       |   
	//                   |        |       |   
	{"SEED FPGA",        0,       0,      5 },
	{"TA FPGA",          0,       1,      5 },
	{"SAFETY EE FPGA",   0,       2,      5 },
	{"SAFETY OPT FPGA",  0,       3,      5 }
}; 


