#ifndef ak8963_H_
#define ak8963_H_
#include <stdbool.h>
#include "i2cdev.h"
#define AK8963_ADDRESS_00         0x0C 
#define ak8963_ADDRESS            0x0C 
#define ak8963_DEFAULT_ADDRESS    0x0C
#define AK8963_RA_WIA             0x00 
#define AK8963_RA_INFO            0x01 
#define AK8963_RA_ST1             0x02 
#define AK8963_RA_HXL             0x03 
#define AK8963_RA_HXH             0x04 
#define AK8963_RA_HYL             0x05 
#define AK8963_RA_HYH             0x06 
#define AK8963_RA_HZL             0x07 
#define AK8963_RA_HZH             0x08 
#define AK8963_RA_ST2             0x09 
#define AK8963_RA_CNTL            0x0A 
#define AK8963_RA_CNTL2           0x0B 
#define AK8963_RA_ASTC            0x0C 
#define AK8963_RA_I2CDIS          0x0F 
#define AK8963_RA_ASAX            0x10 
#define AK8963_RA_ASAY            0x11 
#define AK8963_RA_ASAZ            0x12 
#define ak8963_RA_CONFIG_A        AK8963_RA_CNTL
#define ak8963_RA_CONFIG_B        AK8963_RA_CNTL2
#define ak8963_RA_MODE            AK8963_RA_CNTL
#define ak8963_RA_DATAX_H         AK8963_RA_HXH
#define ak8963_RA_DATAX_L         AK8963_RA_HXL
#define ak8963_RA_DATAZ_H         AK8963_RA_HZH
#define ak8963_RA_DATAZ_L         AK8963_RA_HZL
#define ak8963_RA_DATAY_H         AK8963_RA_HYH
#define ak8963_RA_DATAY_L         AK8963_RA_HYL
#define ak8963_RA_STATUS          AK8963_RA_ST1
#define ak8963_RA_ID_A            AK8963_RA_WIA
#define ak8963_RA_ID_B            AK8963_RA_INFO
#define ak8963_RA_ID_C            AK8963_RA_ST2
#define ak8963_CRA_AVERAGE_BIT    6
#define ak8963_CRA_AVERAGE_LENGTH 2
#define ak8963_CRA_RATE_BIT       4
#define ak8963_CRA_RATE_LENGTH    3
#define ak8963_CRA_BIAS_BIT       1
#define ak8963_CRA_BIAS_LENGTH    2
#define ak8963_AVERAGING_1        0x00
#define ak8963_AVERAGING_2        0x01
#define ak8963_AVERAGING_4        0x02
#define ak8963_AVERAGING_8        0x03
#define ak8963_RATE_0P75          0x00
#define ak8963_RATE_1P5           0x01
#define ak8963_RATE_3             0x02
#define ak8963_RATE_7P5           0x03
#define ak8963_RATE_15            0x04
#define ak8963_RATE_30            0x05
#define ak8963_RATE_75            0x06
#define ak8963_BIAS_NORMAL        0x00
#define ak8963_BIAS_POSITIVE      0x01
#define ak8963_BIAS_NEGATIVE      0x02
#define ak8963_CRB_GAIN_BIT       7
#define ak8963_CRB_GAIN_LENGTH    3
#define ak8963_GAIN_1370          0x00
#define ak8963_GAIN_1090          0x01
#define ak8963_GAIN_820           0x02
#define ak8963_GAIN_660           0x03
#define ak8963_GAIN_440           0x04
#define ak8963_GAIN_390           0x05
#define ak8963_GAIN_330           0x06
#define ak8963_GAIN_220           0x07
#define ak8963_MODEREG_BIT        1
#define ak8963_MODEREG_LENGTH     2
#define AK8963_MODE_POWER_DOWN    0x00 
#define AK8963_MODE_SINGLE        0x01 
#define AK8963_MODE_CONT1         0x02 
#define AK8963_MODE_CONT2         0x06 
#define AK8963_MODE_EXTERNAL      0x04 
#define AK8963_MODE_SELF_TEST     0x08 
#define AK8963_MODE_FUSE_ROM      0x0F 
#define AK8963_MODE_14BIT         0x00 
#define AK8963_MODE_16BIT         0x10 
#define ak8963_MODE_CONTINUOUS    AK8963_MODE_CONT2
#define ak8963_MODE_SINGLE        AK8963_MODE_SINGLE
#define ak8963_MODE_IDLE          AK8963_MODE_POWER_DOWN
#define AK8963_ST1_DRDY_BIT       0 
#define AK8963_ST1_DOR_BIT        1 
#define AK8963_ST2_HOFL_BIT       3 
#define AK8963_ST2_BITM_BIT       4 
#define ak8963_STATUS_LOCK_BIT    AK8963_ST1_DOR_BIT
#define ak8963_STATUS_READY_BIT   AK8963_ST1_DRDY_BIT
#define ak8963_ST_GAIN            ak8963_GAIN_440  
#define ak8963_ST_GAIN_NBR        440
#define ak8963_ST_ERROR           0.2                
#define ak8963_ST_DELAY_MS        250                
#define ak8963_ST_X_NORM          (int32_t)(1.16 * ak8963_ST_GAIN_NBR)
#define ak8963_ST_X_MIN           (int32_t)(ak8963_ST_X_NORM - (ak8963_ST_X_NORM * ak8963_ST_ERROR))
#define ak8963_ST_X_MAX           (int32_t)(ak8963_ST_X_NORM + (ak8963_ST_X_NORM * ak8963_ST_ERROR))
#define ak8963_ST_Y_NORM          (int32_t)(1.16 * ak8963_ST_GAIN_NBR)
#define ak8963_ST_Y_MIN           (int32_t)(ak8963_ST_Y_NORM - (ak8963_ST_Y_NORM * ak8963_ST_ERROR))
#define ak8963_ST_Y_MAX           (int32_t)(ak8963_ST_Y_NORM + (ak8963_ST_Y_NORM * ak8963_ST_ERROR))
#define ak8963_ST_Z_NORM          (int32_t)(1.08 * ak8963_ST_GAIN_NBR)
#define ak8963_ST_Z_MIN           (int32_t)(ak8963_ST_Z_NORM - (ak8963_ST_Z_NORM * ak8963_ST_ERROR))
#define ak8963_ST_Z_MAX           (int32_t)(ak8963_ST_Z_NORM + (ak8963_ST_Z_NORM * ak8963_ST_ERROR))
void ak8963Init(I2C_Dev *i2cPort);
bool ak8963TestConnection();
bool ak8963SelfTest();
bool ak8963EvaluateSelfTest(int16_t min, int16_t max, int16_t value, char *string);
uint8_t ak8963GetSampleAveraging();
void ak8963SetSampleAveraging(uint8_t averaging);
uint8_t ak8963GetDataRate();
void ak8963SetDataRate(uint8_t rate);
uint8_t ak8963GetMeasurementBias();
void ak8963SetMeasurementBias(uint8_t bias);
uint8_t ak8963GetGain();
void ak8963SetGain(uint8_t gain);
uint8_t ak8963GetMode();
void ak8963SetMode(uint8_t mode);
void ak8963GetHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t ak8963GetHeadingX();
int16_t ak8963GetHeadingY();
int16_t ak8963GetHeadingZ();
bool ak8963GetLockStatus();
bool ak8963GetReadyStatus();
uint8_t ak8963GetIDA();
uint8_t ak8963GetIDB();
uint8_t ak8963GetIDC();
#endif 