/*
 *  COPYRIGHT (c) 2012 by Lattice Semiconductor Corporation
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

/** @file XO2_dev.h 
 * Definitions for the structures and types used by the API and Command
 * routines.
 * <p>
 * The XO2Handle_t contains the references to the specific hardware interface
 * that the commands and APIs will use (passed to them as an arg) for
 * accessing the specific XO2 device.
 * <p>
 * The XO2_JEDEC_t structure contains data from the converted JEDEC file,
 * using a utility to parse out the Configuration, UFM and other fields
 * from the JEDEC data representation to C variables.  The instance(s) of
 * this JEDEC file data are then passed to the program APIs to program
 * the configuration and/or UFM sectors with the binary data extracted
 * from the JEDEC file.
 */
 
#ifndef LATTICE_XO2_H
#define LATTICE_XO2_H

#include "stm32h7xx_hal.h"

#define OK     0
#define ERROR -1

#define TRUE  1
#define FALSE 0

#define XO2_FLASH_PAGE_SIZE (16)   /**< 16 bytes per page in Cfg and UFM sectors */
#define XO2_FLASH_PAGES_LEN(n) (n * XO2_FLASH_PAGE_SIZE)   /**< Number of bytes in that many pages */
#define LATTICE_XO2_NUM_DEVS 9

// #define DEBUG_ECA 1

typedef int Boolean_t;

#define MACHXO_I2C_ADDR_7BIT   (0x40u)

/**
 * List of the various XO2 device types.
 * Used for indexing into the XO2 device data base for finding information
 * such as number of UFM pages, erase times, etc.
 *  @note this order must match the initialized order in XO2DevList.
 */
typedef enum  
{
	MachXO2_256,       /**< XO2 256 LUT */
	MachXO2_640,       /**< XO2 640 LUT */
	MachXO2_640U,      /**< XO2 640U LUT with larger UFM size */ 
	MachXO2_1200,      /**< XO2 1200 LUT */ 
	MachXO2_1200U,     /**< XO2 1200 LUT with larger UFM size */  
	MachXO2_2000,      /**< XO2 2000 LUT  */  
	MachXO2_2000U,     /**< XO2 2000 LUT with larger UFM size */  
	MachXO2_4000,      /**< XO2 4000 LUT device */
	MachXO2_7000       /**< XO2 7000 LUT device */
} XO2Devices_t;

typedef enum
{
	MUX_FPGA_SEED = 0,
	MUX_FPGA_TA = 1,
	MUX_FPGA_SAFE_EE = 2,
	MUX_FPGA_SAFE_OPT = 3
} XO2MuxChannel_t;

/**
 * Mode of XO2 device operation during programming and erasing operations.
 */
typedef enum  
{
	TRANSPARENT_MODE,   /**< Erase and program sector while device is still operating. */
	OFFLINE_MODE        /**< Halt user mode during erase and program sector. */
} XO2CfgMode_t;


/**
 * Which sector to operate on.
 */
typedef enum  
{
	CFG_SECTOR,   /**< Flash Configuration sector (holds design) */
	UFM_SECTOR,    /**< User Flash Memory sector (holds EBR values and general purpose usage) */
	FEATURE_ROW,
	SRAM
} XO2SectorMode_t;



/* Handle describing one MachXO2 target on an I2C bus */
typedef struct
{
    I2C_HandleTypeDef *hi2c;   /* e.g. &hi2c1 */
    uint8_t            verbose;
    uint8_t            addr7;  /* 7-bit address, e.g. 0x40 */
	uint8_t			   mux_idx;
	uint8_t 		   mux_ch;
} MachXO_Handle_t;


/**
 * Device parameters needed for accessing and programming.
 */
typedef struct 
{
	const char *pName;      /**< String for printing the XO2 devic epart number. */
	int        Cfgpages;    /**<  Number of Configuration pages in an XO2 device. */
	int        UFMpages;    /**<  Number of UFM pages in an XO2 device. */
	int	       CfgErase;    /**< How long to wait (msec) for Configuration sector to erase. */
	int	       UFMErase;    /**< How long to wait (msec) for UFM sector to erase. */
	int	       Trefresh;    /**< Used for pausing after FlashCheck command or Refresh command. */
} XO2DevInfo_t;


typedef struct
{
	Boolean_t		cfgEn;    /**< 1=Configuration Logic access is enabled, 0=not */
	XO2Devices_t 	devType;     /**< XO2 part number for information about sizes and programming times */
	MachXO_Handle_t  *pI2CDrvrCalls; /**< specific driver routines for framing commands and reading/writing commands. */
	void             (*pfuSecDelay)(uint32_t us);
	void             (*pfmSecDelay)(uint32_t us);
} XO2Handle_t;


/**
 * The bitfields of the Feature Row, extracted from the JEDEC file.
 * These bits fields are very important for device operation, so only
 * use values properly parsed from a valid XO2 JEDEC file.
 */
typedef struct
{
	unsigned char feature[8];
	unsigned char feabits[2];
} XO2FeatureRow_t;


/**
 * The extracted data from the JEDEC file.
 */
typedef struct
{
	XO2Devices_t devID;
	unsigned int pageCnt;
	unsigned int CfgDataSize;
	unsigned int UFMDataSize;
	unsigned int UserCode;
	unsigned int SecurityFuses;
	const unsigned char *pCfgData;
	const unsigned char *pUFMData;
	XO2FeatureRow_t *pFeatureRow;
} XO2_JEDEC_t;



/**
 * Struture to return register information read from the XO2 configuration block.
 * @see XO2ECA_apiGetHdwInfo
 */
typedef struct
{
	unsigned int  devID;      /**< Lattice Device ID code for this XO2 device */ 
	unsigned int  UserCode;   /**< 32 bit user code set in XO2 */
	unsigned char TraceID[8]; /**< 8 byte trace ID prorammed into XO2 */
	int		  devInfoIndex;  /**< to index into the pXO2DevList[] for more info */
} XO2RegInfo_t;



#ifndef XO2DEVLIST_DECLARATION
// Prototype for device data base
extern const XO2DevInfo_t XO2DevList[LATTICE_XO2_NUM_DEVS];

#endif


#endif
