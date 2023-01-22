/**
 * @file pni_rm3100.c
 */

#include <pni_rm3100.h>
#include <stdio.h>
#include <rc/i2c.h>
#include <string.h>
#include <rc/time.h>

//I2C bus from the 4-pin JST-SH connector on the BeagleBone Blue
#define JST_I2C_BUS 1
/**
*	Local variables
**/
static pni_rm3100_config_t config;
pni_rm3100_data_t rm3100_data_extern;
double rm3100_calibration_params[9];

pni_rm3100_config_t rm3100_default_config(void)
{
	pni_rm3100_config_t conf;

    conf.i2c_bus = JST_I2C_BUS;
    conf.i2c_addr = RM_3100_I2C_ADDR_HH;
    conf.debug_statements = 0;
    conf.tmrc = TMRC_600HZ;
    conf.x_ccr = RM_3100_CCR_DEFAULT;
    conf.y_ccr = RM_3100_CCR_DEFAULT;
    conf.z_ccr = RM_3100_CCR_DEFAULT;
    conf.x_scaling = 1.0/75.0; // uT/LSB: conversion is 1/75 if x_ccr is 0x00C8
    conf.y_scaling = 1.0/75.0; // uT/LSB: conversion is 1/75 if y_ccr is 0x00C8
    conf.z_scaling = 1.0/75.0; // uT/LSB: conversion is 1/75 if z_ccr is 0x00C8
    conf.cmx = true;
    conf.cmy = true;
    conf.cmz = true;
    conf.drdm = true;
    conf.cmm = true;
    config.to = BIST_TO_120us;
    config.lrp = BIST_LRP_4;
    config.drc1 = true;
    config.drc0 = true;

	return conf;
}

uint16_t endian_swap_int16(uint16_t input_int16)
{
    uint16_t output_int16 = ((input_int16 << 8) & 0xFF00) |((input_int16 >> 8) & 0x00FF);
    return output_int16;
}


/**
__cycle_count_to_scaling "Cycle Count to Gain (uT/LSB)
    Table 3.1 (pg. 5 of datasheet) Gives the gain/scaling for three Cycle Count values
    The following link on the PNI Help Center confirms the relationship between Cycle Count and gain is linear
        https://pnisensor.zendesk.com/hc/en-us/articles/360016009534-WHAT-IS-THE-RELATIONSHIP-BETWEEN-CYCLE-COUNT-AND-GAIN-
        https://pnisensor.zendesk.com/hc/en-us/articles/360058129973-RM3100-Frequently-Asked-Questions
**/
double cycle_count_to_scaling(uint16_t cycle_count)
{
    double gain = (150.0 / 400.0) * (double) cycle_count;    // [LSB / uT]
    double scaling = 1.0 / gain;                           // [uT / LSB]
    return scaling;
}

int rm3100_set_ccr(){
    uint16_t ccrAll[3];
    ccrAll[0] = endian_swap_int16(config.x_ccr);
    ccrAll[1] = endian_swap_int16(config.y_ccr);
    ccrAll[2] = endian_swap_int16(config.z_ccr);

    // Ensure scaling factors are appropriate for these CCR values
    config.x_scaling = cycle_count_to_scaling(config.x_ccr);
    config.y_scaling = cycle_count_to_scaling(config.y_ccr);
    config.z_scaling = cycle_count_to_scaling(config.z_ccr);


    return rc_i2c_write_bytes(config.i2c_bus, RM_3100_CCR_REGISTER, 6, (uint8_t*) ccrAll);
}

int rm3100_read_ccr(pni_rm3100_data_t* data){
    uint8_t raw[6];
    
    // set the device address
	rc_i2c_set_device_address(config.i2c_bus, config.i2c_addr);

    if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_CCR_REGISTER, 6, &raw[0])<0){
        return -1;
    }

    //Store results in 'data' struct
    memcpy(&(data->ccrx), raw, 6);

    if(config.debug_statements){
        printf("----- CCR REGISTER -----\n");

        //Print Raw Bytes
        printf("CCR: 0x");
        for(unsigned i = 0; i < 6; ++i){
            printf("%02X", raw[i]);
            if(i % 2 - 1 == 0) printf(" "); //Space after every 2 bytes
        }
        printf("\n");
    }

    return 0;
}

int rm3100_set_tmrc(pni_rm3100_tmrc_t tmrc)
{
    uint8_t c = 0x00;
    
    switch(tmrc){
    case TMRC_600HZ:
        c = TMRC_600HZ;
        break;
    case TMRC_300HZ:
        c = TMRC_300HZ;
        break;
    case TMRC_150HZ:
        c = TMRC_150HZ;
        break;
    case TMRC_75HZ:
        c = TMRC_75HZ;
        break;
    case TMRC_37HZ:
        c = TMRC_37HZ;
        break;
    case TMRC_18HZ:
        c = TMRC_18HZ;
        break;
    case TMRC_9HZ:
        c = TMRC_9HZ;
        break;
    case TMRC_4_5HZ:
        c = TMRC_4_5HZ;
        break;
    case TMRC_2_3HZ:
        c = TMRC_2_3HZ;
        break;
    case TMRC_1_2HZ:
        c = TMRC_1_2HZ;
        break;
    case TMRC_0_6HZ:
        c = TMRC_0_6HZ;
        break;
    case TMRC_0_3HZ:
        c = TMRC_0_3HZ;
        break;
    case TMRC_0_015HZ:
        c = TMRC_0_015HZ;
        break;
    case TMRC_0_0075HZ:
        c = TMRC_0_0075HZ;
        break;
    default:
        fprintf(stderr,"invalid RM3100 TMRC\n");
        return -1;
    }

    return rc_i2c_write_byte(config.i2c_bus, RM_3100_TMRC_REGISTER, c);
}

int rm3100_set_cmm(bool start_cmm){
    uint8_t c = 0x00;

    if(config.cmz){
        c |= RM_3100_CMM_CMZ;
    }
    if(config.cmy){
        c |= RM_3100_CMM_CMY;
    }
    if(config.cmx){
        c |= RM_3100_CMM_CMX;
    }
    if(config.drdm){
        c |= RM_3100_CMM_DRDM;
    }
    if(start_cmm){
        c |= RM_3100_CMM_START;
    }

    return rc_i2c_write_byte(config.i2c_bus, RM_3100_CMM_REGISTER, c);
}


int rm3100_set_poll(){
    uint8_t c = 0x00;

    if(config.cmz){
        c |= RM_3100_POLL_PMZ;
    }
    if(config.cmy){
        c |= RM_3100_POLL_PMY;
    }
    if(config.cmx){
        c |= RM_3100_POLL_PMX;
    }

    return rc_i2c_write_byte(config.i2c_bus, RM_3100_POLL_REGISTER, c);

}

int rm3100_read_status(pni_rm3100_data_t* data){
    uint8_t status_value = 0xFF;

    // set the device address
    rc_i2c_set_device_address(config.i2c_bus, config.i2c_addr);
	// Read the STATUS byte
	if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_STATUS_REGISTER, 1, &status_value)<0){
		return -1;
	}

    data->status = status_value;

    if(config.debug_statements){
        printf("----- STATUS REGISTER -----\n");
        //If first bit is set, then DRDY is HIGH
        if(status_value & RM_3100_STATUS_DRDY)
            printf("DRDY is HIGH\n");
        else
            printf("DRDY is LOW\n");
    }

    return 0;

}

int rm3100_read_meas(pni_rm3100_data_t* data){
    uint8_t raw[9];

    // set the device address
	rc_i2c_set_device_address(config.i2c_bus, config.i2c_addr);

    //If all three measurements were requested, rad all 9 bytes cosecutively
    if(config.cmx && config.cmy && config.cmz){
        if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_MEAS_REGISTER, 9, &raw[0])<0){
            //Set all data values to zero to indicate no data was actually read
            data->mag[0] = data->mag[1] = data->mag[2] = 0;
		    return -1;
	    }

        //Convert X Value
        data->mag[0] = rm3100_convert_mag_meas(raw, 0);
        
        //Convert Y Value
        data->mag[1] = rm3100_convert_mag_meas(raw+3, 1);

        //Convert Z Value
        data->mag[2] = rm3100_convert_mag_meas(raw+6, 2);
    }
    //Otherwise, check each bool and read each magnetometer individually
    else{
        //Read X-axis magnetometer
        if(config.cmx){
            if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_MEAS_REGISTER, 3, &raw[0])<0){
                //Set all data values to zero to indicate no data was actually read
                data->mag[0] = data->mag[1] = data->mag[2] = 0;
                return -1;
            }
            
            //Convert X Value
            data->mag[0] = rm3100_convert_mag_meas(raw, 0);
        }
        //Read Y-axis magnetometer
        if(config.cmy){
            if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_MEAS_REGISTER + 3, 3, &raw[3])<0){
                //Set all data values to zero to indicate no data was actually read
                data->mag[0] = data->mag[1] = data->mag[2] = 0;
                return -1;
            }

            //Convert Y Value
            data->mag[1] = rm3100_convert_mag_meas(raw+3, 1);
        }
        //Read Z-axis magnetometer
        if(config.cmz){
            if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_MEAS_REGISTER + 6, 3, &raw[6])<0){
                //Set all data values to zero to indicate no data was actually read
                data->mag[0] = data->mag[1] = data->mag[2] = 0;
                return -1;
            }

            //Convert Z Value
            data->mag[2] = rm3100_convert_mag_meas(raw+6, 2);
        }
    }

    //Save timestamp
    data->timestamp = rc_nanos_since_epoch();

    //Print debug stuff if requested
    if(config.debug_statements){
        printf("----- MEAS REGISTER -----\n");

        //Print Raw Bytes
        printf("MEAS: 0x");
        for(unsigned i = 0; i < 9; ++i){
            printf("%02X", raw[i]);
            if(i % 3 == 2) printf(" "); //Space after every 3 bytes
        }
        printf("\n");

        // printf("4-byte Ints: %08X %08X %08X\n", rawX, rawY, rawZ);
        printf("X: %lf, Y: %lf, Z: %lf\n",data->mag[0], data->mag[1], data->mag[2]);

    }
    return 0;
}

int rm3100_set_bist(bool start_self_test){
    uint8_t c = 0x00;

    //Toggle to initiate self-test
    if(start_self_test)
        c |= RM_3100_BIST_STE;
    
    //Set BW1 and BW0 bits to select Timeout (Table 5-6)
    c|= config.to;

    //Set BP1 and BP0 bits to select number of LR Period (Table 5-7)
    c|= config.lrp;

    return rc_i2c_write_byte(config.i2c_bus, RM_3100_BIST_REGISTER, c);
}

int rm3100_read_bist(pni_rm3100_data_t* data){
    uint8_t bist_value = 0x00;

    // set the device address
    rc_i2c_set_device_address(config.i2c_bus, config.i2c_addr);

    //Read 1 byte from BIST register
    if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_BIST_REGISTER, 1, &bist_value)<0){
        return -1;
    }

    data->bist = bist_value;

    if(config.debug_statements){
        printf("----- BIST REGISTER -----\n");\
        printf("BIST: 0x%02X", bist_value);
        printf("\t STE: %d\n", bist_value & RM_3100_BIST_STE);
        printf("\t ZOK: %d YOK: %d XOK: %d\n", 
                bist_value & RM_3100_BIST_ZOK, 
                bist_value & RM_3100_BIST_YOK,
                bist_value & RM_3100_BIST_XOK);
        printf("\t Timeout: %d\n",
                bist_value & (RM_3100_BIST_BW1+RM_3100_BIST_BW0));
        printf("\t LR P: %d\n",
                bist_value & (RM_3100_BIST_BP1+RM_3100_BIST_BP0));
    }

    return 0;
}

int rm3100_set_hshake(){
    uint8_t c = 0x00;

    //Setting this bit means DRDY is cleared when reading any Measurement Result Regisers 
    if(config.drc1)
        c |= RM_3100_HSHAKE_DRC1;

    //Setting this bit means DRDY is cleared when writing to any RM3100 register
    if(config.drc0)
        c |= RM_3100_HSHAKE_DRC0;

    return rc_i2c_write_byte(config.i2c_bus, RM_3100_HSHAKE_REGISTER, c);
}

int rm3100_read_hshake(pni_rm3100_data_t* data){
    uint8_t hshake_value = 0x00;

    // set the device address
    rc_i2c_set_device_address(config.i2c_bus, config.i2c_addr);

    //Read 1 byte from BIST register
    if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_HSHAKE_REGISTER, 1, &hshake_value)<0){
        return -1;
    }

    data->hshake = hshake_value;

    if(config.debug_statements){
        printf("----- HSHAKE REGISTER -----\n");\
        printf("HSHAKE: 0x%02X", hshake_value);

        if(hshake_value & RM_3100_HSHAKE_NACK2)
            printf("NACK2: Read from Measurement Register before DRDY was HIGH\n"); 

        if(hshake_value & RM_3100_HSHAKE_NACK1)
            printf("NACK1: Wrote to POLL Register while in CMM\n"); 

        if(hshake_value & RM_3100_HSHAKE_NACK0)
            printf("NACK0: Wrote to unidentified Register\n"); 
    }

    return 0;
}

int rm3100_read_revid(pni_rm3100_data_t* data){
    uint8_t revid_value = 0x00;

    // set the device address
    rc_i2c_set_device_address(config.i2c_bus, config.i2c_addr);

    //Read 1 byte from BIST register
    if(rc_i2c_read_bytes(config.i2c_bus, RM_3100_REVID_REGISTER, 1, &revid_value)<0){
        return -1;
    }

    data->revid = revid_value;

    if(config.debug_statements){
        printf("----- REVID REGISTER -----\n");\
        printf("REVID: 0x%02X = %d" , revid_value, revid_value);
    }

    return 0;
}


int rm3100_init(pni_rm3100_config_t conf)
{
	// update local copy of config struct with new values
	config=conf;

    // Initialize timestamp to current time
    rm3100_data_extern.timestamp = rc_nanos_since_epoch();

    // Initialize calibrated values to 0 (in case we don't apply calibration real-time)
    rm3100_data_extern.cal_mag[0] = rm3100_data_extern.cal_mag[1] = rm3100_data_extern.cal_mag[2] = 0;

	// make sure the bus is not currently in use by another thread
	// do not proceed to prevent interfering with that process
	if(rc_i2c_get_lock(config.i2c_bus)){
		printf("i2c bus claimed by another process\n");
		printf("Continuing with rc_mpu_initialize() anyway.\n");
	}

	// if it is not claimed, start the i2c bus
	if(rc_i2c_init(config.i2c_bus, config.i2c_addr)<0){
		fprintf(stderr,"failed to initialize i2c bus\n");
		return -1;
	}
	// claiming the bus does no guarantee other code will not interfere
	// with this process, but best to claim it so other code can check
	// like we did above
	rc_i2c_lock_bus(config.i2c_bus);

	// restart the device so we start with clean registers
	if(rm3100_set_tmrc(config.tmrc)<0){
		fprintf(stderr,"ERROR: failed to set RM3100 TMRC for CMM\n");
        fprintf(stderr,"I2C Bus: %d. Slave ADDR: %02X\n", 
        config.i2c_bus, config.i2c_addr);
        
		rc_i2c_unlock_bus(config.i2c_bus);
		return -1;
	}
	if(rm3100_set_cmm(true)){
        fprintf(stderr,"ERROR: failed to start RM3100 CMM\n");
		rc_i2c_unlock_bus(config.i2c_bus);
		return -1;
	}

    if(rm3100_set_ccr()){
        fprintf(stderr,"ERROR: failed to start RM3100 CCR\n");
		rc_i2c_unlock_bus(config.i2c_bus);
		return -1;
	}

	// all done!!
	rc_i2c_unlock_bus(config.i2c_bus);
	return 0;
}

//Convert RM3100 Magnetometer Value to physical units
double rm3100_convert_mag_meas(uint8_t raw[3], uint8_t axis)
{
    // Useful constants
    int32_t signBit = 0x00800000;
    int32_t rawInt = 0;

    // Select appropriate scaling factor
    double scaling_factor = 0;
    switch(axis)
    {
        case 0:
            scaling_factor = config.x_scaling;
            break;
        case 1:
            scaling_factor = config.y_scaling;
            break;
        case 2:
            scaling_factor = config.z_scaling;
            break;
    }

    //Read in the 3-byte integer
    rawInt = (int32_t) (raw[0]<<16 | raw[1]<<8 | raw[2]);
    
    //Compute Two Complement then multiply by scaling factor
    if(rawInt & signBit)
    {
        return scaling_factor * ((rawInt & ~signBit) - signBit);
    }
    else
    {
        return scaling_factor * rawInt;
    }
}

/**
 * @brief This function loads calibration parameters from '~/rc_pilot_a2sys/calibration/rm3100_mag.cal'
 * to be used in the 'rm3100_apply_springmann_calibration()' function
 * @description 
 * The format of the calibration file must be
 *      Number of rm3100 magnetometers on board
 *      a, b, c, x0, y0, z0, rho, phi, lam #First rm3100 magnetometer
 *      a, b, c, x0, y0, z0, rho, phi, lam #Second rm3100 magnetometer
 *      ...
 *      a, b, c, x0, y0, z0, rho, phi, lam #Nth rm3100 magnetometer
 * 
 * @param[in] filename Name of the file (including _absolute_ filepath) containing calibration parameters
 * @param[out] params Array holding the calibration parameters 
 * @todo Make this work for multiple magnetometers if that ever comes up
 * @return 0 if no errors. 1 if cannot open 'filename' successfully. 2 if parameter file ends too early.
 */
int rm3100_load_springmann_calibration_params(char *filename, double params[9])
{
    FILE *fp;
    fp = fopen(filename, "r");

    // Check for null pointer
    if(!fp)
    {
        return 1;
    }

    // Read number of magnetometers
    int fxn_ret_val = 0;
    int num_magnetometers = 0;
    if(fscanf(fp, "%d", &num_magnetometers) == EOF)
    {
        fxn_ret_val = 2;
    }

    // Read in parameters (@todo, make this work for multiple magnetometers if needed)
    for (int i= 0; i < 9 && fxn_ret_val==0; ++i)
    {
        // If we've reached end of file before we expect to, stop reading and set error value
        if(fscanf(fp, "%lf", params+i) == EOF)
        {
            fxn_ret_val = 2;
            break;
        }
    }

    //Cleanup and leave
    fclose(fp);
    return fxn_ret_val;
}

/**
 * @brief This function takes in raw (uncalibrated) magnetometer data and 
 * returns calibrated magnetometer values.
 * 
 * The Calibration method assumes the model used by John Springmann in the following paper.
 * This function ony performs the time-invariant calibration 
 * and does not account for nearby electrical current sources.
 * https://arc.aiaa.org/doi/10.2514/1.56726
 * 
 * @param[in] params The 9 calibration parameters [a,b,c, x0, y0, z0, rho, phi, lam]
 * @param[in] raw Raw magnetometer data [magX, magY, magZ] (returned values from 'rm3100_convert_mag_meas()') 
 * @param[out] calibrated Magnetometer measurements corrected by the given 'params' and Springmann measurement model
 * @return 0 if no errors. 1 if input 'params' are default parameters
 */
int rm3100_apply_springmann_calibration(double params[9], double raw[3], double calibrated[3])
{
    
    // Unpack parameters
    double a = params[0], b = params[1], c = params[2];
    double x0 = params[3], y0 = params[4], z0 = params[5];
    double rho = params[6], phi = params[7], lam = params[8];

    //Check if parameters are default values
    bool scale_factors_unchanged = a == 1.0 && b == 1.0 && c == 1.0;
    bool biases_unchanged = x0 == 0.0 && y0 == 0.0 && z0 == 0.0;
    bool non_orthogonalities_unchanged = rho == 0.0 && phi == 0.0 && lam == 0.0;

    if(scale_factors_unchanged && biases_unchanged && non_orthogonalities_unchanged)
    {
        for (int i = 0; i < 3; ++i)
        {
            calibrated[i] = raw[i];
        }
        return 1;
    }

    //Apply calibration parameters to measurement model
    //In effect, rearrange equations (1), (2), and (3) for B_x, B_y, and B_z (non-tilde)
    //And make small angle approximation where...
    //cos(smallAngle) = 1, and sin(smallAngle) = smallAngle
    calibrated[0] = -(x0 - raw[0])/a;
    calibrated[1] = -(y0 - raw[1] + b*rho*calibrated[0])/b;
    calibrated[2] = -(z0 - raw[2] + c*lam*calibrated[0] + c*phi*calibrated[1])/c;
    return 0;
}