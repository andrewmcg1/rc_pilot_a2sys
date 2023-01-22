/**
 * <pni_rm3100.h>
 *
 * @brief       Code for reading from the PNI RM3100 magnetometers. Some comments copy-pasted
 * from the PNI RM3100 datasheet.
 *
 * 
 * @author     Prince Kuevor 
 * @date       11/02/2019 (MM/DD/YYYY)
 * 
 * 
 * @addtogroup  PNI_RM3100
 * @{
 */

#ifndef __PNI_RM3100__
#define __PNI_RM3100__

#include <stdint.h>
#include <stdbool.h>
/** Terminology
 * Setting a bit or pin means to assign that bit/pin to be HIGH (1)
 * Clearing a bit or pin means to assign that bit/pin to be LOW (0)
 */

/** I2C Address for PNI RM3100
 * The two pins SA1 (SSN) and SA0 (MISO/SO) dictate the two least significant bits of the address
 * These pins can be pulled HIGH or LOW. 
 * DEFAULT: ADDR = 0x40  ->  SA1 = SA0 = LOW
 */
#define RM_3100_I2C_ADDR_LL 0x20    //SA1(SSN) pulled LOW,  SA0(MISO) pulled LOW
#define RM_3100_I2C_ADDR_LH 0x21    //SA1(SSN) pulled LOW,  SA0(MISO) pulled HIGH
#define RM_3100_I2C_ADDR_HL 0x22    //SA1(SSN) pulled HIGH, SA0(MISO) pulled LOW
#define RM_3100_I2C_ADDR_HH 0x23    //SA1(SSN) pulled HIGH, SA0(MISO) pulled HIGH


/** CCR (Cycle Count Register) - pg28 of datasheet
 * From Datasheet: Number of sensor oscillation cycles (cycle counts) during a measurment sequence. 
 * Datasheet Recommendation: greater than ~30 Cycle Counts and less than ~400 Cycle Counts
 * DEFAULT: 200 (0x00C8) Cycle Counts which gives 75 LSB/uT = 7.5 LSB/mG and a max sampling frequency of 440Hz. 
 */
#define RM_3100_CCR_REGISTER    0x04

#define RM_3100_CCR_DEFAULT 0x00C8


/** CMM (Continuous Measurement Mode) Register - pg30 of datasheet
 * These five bits are options when using Continuous Measurement Mode
 */
#define RM_3100_CMM_REGISTER  0x01  //Memory register for CMM

#define RM_3100_CMM_CMZ 0x40    //Set this bit to read from the Z-axis magnetometer during CMM
#define RM_3100_CMM_CMY 0x20    //Set this bit to read from the Y-axis magnetometer during CMM
#define RM_3100_CMM_CMX 0x10    //Set this bit to read from the X-axis magnetometer during CMM

#define RM_3100_CMM_DRDM 0x04   //DRDY (Data Ready) Mode: Controls when DRDY pin will be set (Table 5-3)
                                //Set this bit for the DRDY pin to be set after a measurement on any axis
                                //Clear this bit for DRDY pin to be set after a full measurement as ...
                                //      dictated by the CMX, CMY, and CMZ bits

#define RM_3100_CMM_START 0x01  //Set this bit (HIGH) to activate CMM
                                //Note: Writing to TMRC (below) will end CMM, so only start after TMRC is at desired value
                                //Note: Reading from the CMM register will end CMM



/** TMRC (Continuous Mode Timer) Register - pg31 of datasheet
 * Sets the amount of time between measurements when using CMM (Continuous Measurement Mode)
 * 
 * Note:CCR (Count Control Registers) determine the maximum sampling rate of sensors. 
 *      If the sampling frequency selected by TMRC is higher than CCR, then the actual sampling rate will be CCR's value
 *      (See note at end of pg31 of datasheet for an example)
 */
#define RM_3100_TMRC_REGISTER   0x0B

/**
 * @brief      RM3100 Timer Control Options (Continuous Measurement Mode)
 *
 * 
 */
typedef enum pni_rm3100_tmrc_t{
	TMRC_600HZ =    0x92,   //~600  Hz
	TMRC_300HZ =    0x93,   //~300  Hz
	TMRC_150HZ =    0x94,   //~150  Hz
	TMRC_75HZ =     0x95,   //~75   Hz
	TMRC_37HZ =     0x96,   //~37   Hz (DEFAULT)
	TMRC_18HZ =     0x97,   //~18   Hz
	TMRC_9HZ =      0x98,   //~9    Hz
	TMRC_4_5HZ =    0x99,   //~4.5  Hz
	TMRC_2_3HZ =    0x9A,   //~2.3  Hz
	TMRC_1_2HZ =    0x9B,   //~1.2  Hz
	TMRC_0_6HZ =    0x9C,   //~0.6  Hz
	TMRC_0_3HZ =    0x9D,   //~0.3  Hz
	TMRC_0_015HZ =  0x9E,   //~0.015  Hz
	TMRC_0_0075HZ = 0x9F,   //~0.0075 Hz
} pni_rm3100_tmrc_t;


/** POLL for a single measurement - pg 32 of datasheet
 * Request a single measurement from the magnetometers
 * DRDY will be HIGH after measurements from all requested axes have been completed
 * 
 * Note: The POLL request will be ignore if in CMM (Continuous Measurement Mode)
 */
#define RM_3100_POLL_REGISTER 0x00

#define RM_3100_POLL_PMZ 0x40 //Set this bit to request a single measurement from the Z-axis magnetometer
#define RM_3100_POLL_PMY 0x20 //Set this bit to request a single measurement from the Y-axis magnetometer
#define RM_3100_POLL_PMX 0x10 //Set this bit to request a single measurement from the X-axis magnetometer


/** Status Register - pg33 of datsheet
 * Read the DRDY (Data Ready) bit
 * This is an alternative to reading the DRDY status from the DRDY pin
 * Also, if in SPI mode, MISO will be pulled HIGH when DRDY goes HIGH
 * 
 * Note: If in CMM (Continuous Measurement Mode), RM_3100_CCM_DRDM dictates when DRDY goes high
 */
#define RM_3100_STATUS_REGISTER 0x34

#define RM_3100_STATUS_DRDY 0x80

/** Measurement Result Registers - pg33 of datsheet
 * These nine consecutive registers (from 0x24 to 0x2C) hold the magnetic field reading from the three magnetometers. 
 * Each axis has three bytes of data stored in 2's complement format
 * 
 * Note: Conversion to these integer values to physical units depends on the CCR value chosen
 * Note: You can read all 9 registers with a single 9-byte read request to 0x24
 * Note: There's no sense in writing to these addresses
 */
#define RM_3100_MEAS_REGISTER 0x24


/** BIST( Built-In Self Test) Register - pg 34 of datasheet
 * 
 */

#define RM_3100_BIST_REGISTER 0x33

#define RM_3100_BIST_STE 0x80   //Self-Test Enable: Set this bit to run the BIST (built-in self test) when POLL register is written to
                                //                  DRDY will go HIGH when the BIST is complete

//The following bits are set to HIGH if the corresponding LR osccilator functioned correctly during the last self test
//The value in these bits are only valid if STE is is HIGH and the self test has been completed
#define RM_3100_BIST_ZOK 0x40   //Read-only. Status of Z-axis LR oscillators 
#define RM_3100_BIST_YOK 0x20   //Read-only. Status of Y-axis LR oscillators 
#define RM_3100_BIST_XOK 0x10   //Read-only. Status of X-axis LR oscillators 

//Together, these two bits determine timeout period for the LR oscillator periods (Table 5-6)
#define RM_3100_BIST_BW1 0x08
#define RM_3100_BIST_BW0 0x04

/**
 * @brief      RM3100 Built-In Self Test: Timeout (Table 5-6)
 *
 * 
 */
typedef enum pni_rm3100_bist_to_t{
	//Timeout is 30us (1 Sleep Oscillation Cycle)
	BIST_TO_30us = RM_3100_BIST_BW0,   
	//Timeout is 60us (2 Sleep Oscillation Cycles)
	BIST_TO_60us = RM_3100_BIST_BW1,   
	//Timeout is 120us (3 Sleep Oscillation Cycles)
	BIST_TO_120us = RM_3100_BIST_BW1 + RM_3100_BIST_BW0,   
} pni_rm3100_bist_to_t;

//Together, these two bits define number of LR periods for measurment during the BIST (Table 5-7)
#define RM_3100_BIST_BP1 0x02
#define RM_3100_BIST_BP0 0x01

/**
 * @brief      RM3100 Built-In Self Test: LR Periods (Table 5-7)
 *
 * 
 */
typedef enum pni_rm3100_bist_lrp_t{
	//1 LR Period
	BIST_LRP_1 = RM_3100_BIST_BP0,   
	//2 LR Periods
	BIST_LRP_2 = RM_3100_BIST_BP1,   
	//4 LR Periods
	BIST_LRP_4 = RM_3100_BIST_BP1 + RM_3100_BIST_BP0,   
} pni_rm3100_bist_lrp_t;

/** HSHAKE (Handshake) Register - pg 36 of datasheet
 * Determines when DRDY pin will be cleared (LOW)
 * Also gives debugging information fora  few erros (see pg 36)
 */
#define RM_3100_HSHAKE_REGISTER 0x35

#define RM_3100_HSHAKE_NACK2 0x40   //Read-only. Assigned HIGH when attempting to read from a Measurement Result register when DRDY is LOW (i.e. data is not ready)
#define RM_3100_HSHAKE_NACK1 0x20   //Read-only. Assigned HIGH when writing to POLL when CMM is active or writing to CMM when POLL is active
#define RM_3100_HSHAKE_NACK0 0x10   //Read-only. Assigned HIGH when writing to an undefined register

#define RM_3100_HSHAKE_DRC1 0x02    //Setting this bit means DRDY is cleared when reading any Measurement Result Regisers (DEFAULT: HIGH)
#define RM_3100_HSHAKE_DRC0 0x01    //Setting this bit means DRDY is cleared when writing to any RM3100 register (DEFAULT: HIGH)

/** REVID (REvision ID) Register - pg 36 of datasheet
 * Read-only. Single byte register that returns revision identification of the MagI2C.
 */

#define RM_3100_REVID_REGISTER 0x36

/**
 * @brief      configuration of the PNI RM3100 Sensor
 *
 * Configuration struct passed to rc_mpu_initialize and rc_mpu_initialize_dmp.
 * It is best to get the default config with rc_mpu_default_config() function
 * first and modify from there.
 */
typedef struct pni_rm3100_config_t{
	/** @name physical connection configuration */
	///@{
	int i2c_bus;			///< which bus to use, default is 4-pin JST-SH I2C bus DEFAULT: 1
	uint8_t i2c_addr;		///< 0x40 if SA1 and SA0 are LOW. DEFAULT: 0x40
	int debug_statements;	///< set to 1 to print i2c_bus warnings for debug. DEFAULT: 0
	///@}

	/** @name  magnetometer configuration */
	///@{
	pni_rm3100_tmrc_t tmrc;	///< Continuous mode sampling frequency. DEFAULT: TMRC_600HZ
    uint16_t x_ccr;         ///< Desired Cycle Count for x-axis magnetometer. DEFAULT: 0x00C8
    uint16_t y_ccr;         ///< Desired Cycle Count for y-axis magnetometer. DEFAULT: 0x00C8
    uint16_t z_ccr;         ///< Desired Cycle Count for z-axis magnetometer. DEFAULT: 0x00C8
    double x_scaling;       ///< Scale factor for x-axis magnetometer. (depends on x_ccr) DEFAULT: 1/75 uT/LSB
    double y_scaling;       ///< Scale factor for y-axis magnetometer. (depends on y_ccr) DEFAULT: 1/75 uT/LSB
    double z_scaling;       ///< Scale factor for z-axis magnetometer. (depends on z_ccr) DEFAULT: 1/75 uT/LSB
    bool cmx;               ///< True to read from x-axis magnetometer during continuous mode. DEFAULT: True
    bool cmy;               ///< True to read from y-axis magnetometer during continuous mode. DEFAULT: True
    bool cmz;               ///< True to read from z-axis magnetometer during continuous mode. DEFAULT: True
    bool drdm;              ///< True to set this bit. False to clear it. See Table 5-3. DEFAULT: True
    bool cmm;               ///< True to use CMM (continuous measurement mode). DEFAULT: True
	pni_rm3100_bist_to_t to;	///< Built-in self test Timeout (Table 5-6). DEFAULT: BIST_TO_120us
	pni_rm3100_bist_lrp_t lrp;	///< Built-in self test LR Period (Table 5-7). DEFAULT: BIST_LRP_4
	bool drc1;				///< True to clear DRDY when writing to any register. DEFAULT: True
	bool drc0;				///< True to clear DRDY when reading any Measurement Result. DEFAULT: True

	///@}

} pni_rm3100_config_t;

/**
 * @brief      PNI RM3100 Data Structure
 *
 */
typedef struct pni_rm3100_data_t{
	/** @name sensor readings in real units. 
	 */
	///@{
	uint16_t ccrx;		///< x-axis Cycle Count Register Value
	uint16_t ccry;		///< y-axis Cycle Count Register Value
	uint16_t ccrz;		///< z-axis Cycle Count Register Value
	double mag[3];		///< magnetometer (XYZ) in units of uT
	double cal_mag[3];	///< calibrated magnetometer (XYZ) in units of uT
    uint8_t status;     ///< STATUS register (MSB is DRDY)
	uint8_t bist;		///< BIST register
	uint8_t hshake;		///< HSHAKE register 
	uint8_t revid;		///< REVID register
	uint64_t timestamp; ///< Time data was read from sensor
	///@}
} pni_rm3100_data_t;

extern pni_rm3100_data_t rm3100_data_extern;
extern double rm3100_calibration_params[9];

int pni_rm3100_init();

pni_rm3100_config_t rm3100_default_config(void);
int rm3100_init(pni_rm3100_config_t conf);
int rm3100_set_tmrc(pni_rm3100_tmrc_t tmrc);
int rm3100_set_cmm(bool start_cmm);
int rm3100_set_ccr();
int rm3100_read_ccr(pni_rm3100_data_t* data);
int rm3100_set_poll();
int rm3100_read_status(pni_rm3100_data_t* data);
int rm3100_read_meas(pni_rm3100_data_t* data);	
int rm3100_set_bist(bool start_self_test);
int rm3100_read_bist(pni_rm3100_data_t* data);
int rm3100_set_hshake();
int rm3100_read_hshake(pni_rm3100_data_t* data);
int rm3100_read_revid(pni_rm3100_data_t* data);
double rm3100_convert_mag_meas(uint8_t raw[3], uint8_t axis);
int rm3100_load_springmann_calibration_params(char *filename, double params[9]);
int rm3100_apply_springmann_calibration(double params[9], double raw[3], double calibrated[3]);




#endif

/** @}  end group PNI_RM3100*/