#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "eprintf.h"
#include "ak8963.h"
#include "i2cdev.h"
#define DEBUG_MODULE "ak8963"
#include "debug_cf.h"
static uint8_t devAddr;
static uint8_t buffer[8];
static uint8_t mode;
static I2C_Dev *I2Cx;
static bool isInit;
void ak8963Init(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }
    I2Cx = i2cPort;
    devAddr = AK8963_ADDRESS_00;
    i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL2, 0x01); 
    vTaskDelay(M2T(10)); 
    uint8_t asa[3];
    i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_FUSE_ROM); 
    vTaskDelay(M2T(10));
    i2cdevReadReg8(I2Cx, devAddr, AK8963_RA_ASAX, 3, asa);
    i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_POWER_DOWN); 
    vTaskDelay(M2T(10));
    i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, AK8963_MODE_POWER_DOWN);
    vTaskDelay(M2T(10));
    isInit = true;
}
bool ak8963TestConnection()
{
    uint8_t wia;
    if (i2cdevReadByte(I2Cx, devAddr, AK8963_RA_WIA, &wia)) {
        return (wia == 0x48);
    }
    return false;
}
bool ak8963SelfTest()
{
    bool testStatus = true;
    int16_t mxp, myp, mzp;  
    int16_t mxn, myn, mzn;  
    struct {
        uint8_t configA;
        uint8_t configB;
        uint8_t mode;
    } regSave;
    if (i2cdevReadReg8(I2Cx, devAddr, ak8963_RA_CONFIG_A, sizeof(regSave), (uint8_t *)&regSave) == false) {
        return false;
    }
    ak8963SetGain(ak8963_ST_GAIN);
    i2cdevWriteByte(I2Cx, devAddr, ak8963_RA_CONFIG_A,
                    (ak8963_AVERAGING_1 << (ak8963_CRA_AVERAGE_BIT - ak8963_CRA_AVERAGE_LENGTH + 1)) |
                    (ak8963_RATE_15 << (ak8963_CRA_RATE_BIT - ak8963_CRA_RATE_LENGTH + 1)) |
                    (ak8963_BIAS_POSITIVE << (ak8963_CRA_BIAS_BIT - ak8963_CRA_BIAS_LENGTH + 1)));
    ak8963SetMode(ak8963_MODE_SINGLE);
    vTaskDelay(M2T(ak8963_ST_DELAY_MS));
    ak8963GetHeading(&mxp, &myp, &mzp);
    i2cdevWriteByte(I2Cx, devAddr, ak8963_RA_CONFIG_A,
                    (ak8963_AVERAGING_1 << (ak8963_CRA_AVERAGE_BIT - ak8963_CRA_AVERAGE_LENGTH + 1)) |
                    (ak8963_RATE_15 << (ak8963_CRA_RATE_BIT - ak8963_CRA_RATE_LENGTH + 1)) |
                    (ak8963_BIAS_NEGATIVE << (ak8963_CRA_BIAS_BIT - ak8963_CRA_BIAS_LENGTH + 1)));
    ak8963SetMode(ak8963_MODE_SINGLE);
    vTaskDelay(M2T(ak8963_ST_DELAY_MS));
    ak8963GetHeading(&mxn, &myn, &mzn);
    if (ak8963EvaluateSelfTest(ak8963_ST_X_MIN, ak8963_ST_X_MAX, mxp, "pos X") &&
            ak8963EvaluateSelfTest(ak8963_ST_Y_MIN, ak8963_ST_Y_MAX, myp, "pos Y") &&
            ak8963EvaluateSelfTest(ak8963_ST_Z_MIN, ak8963_ST_Z_MAX, mzp, "pos Z") &&
            ak8963EvaluateSelfTest(-ak8963_ST_X_MAX, -ak8963_ST_X_MIN, mxn, "neg X") &&
            ak8963EvaluateSelfTest(-ak8963_ST_Y_MAX, -ak8963_ST_Y_MIN, myn, "neg Y") &&
            ak8963EvaluateSelfTest(-ak8963_ST_Z_MAX, -ak8963_ST_Z_MIN, mzn, "neg Z")) {
        DEBUG_PRINTD("ak8963 Self test [OK].\n");
    } else {
        testStatus = false;
    }
    if (i2cdevWriteReg8(I2Cx, devAddr, ak8963_RA_CONFIG_A, sizeof(regSave), (uint8_t *)&regSave) == false) {
        return false;
    }
    return testStatus;
}
bool ak8963EvaluateSelfTest(int16_t min, int16_t max, int16_t value, char *string)
{
    if (value < min || value > max) {
        DEBUG_PRINTD("Self test %s [FAIL]. low: %d, high: %d, measured: %d\n",
                     string, min, max, value);
        return false;
    }
    return true;
}
uint8_t ak8963GetSampleAveraging()
{
    i2cdevReadBits(I2Cx, devAddr, ak8963_RA_CONFIG_A, ak8963_CRA_AVERAGE_BIT, ak8963_CRA_AVERAGE_LENGTH, buffer);
    return buffer[0];
}
void ak8963SetSampleAveraging(uint8_t averaging)
{
    i2cdevWriteBits(I2Cx, devAddr, ak8963_RA_CONFIG_A, ak8963_CRA_AVERAGE_BIT, ak8963_CRA_AVERAGE_LENGTH, averaging);
}
uint8_t ak8963GetDataRate()
{
    i2cdevReadBits(I2Cx, devAddr, ak8963_RA_CONFIG_A, ak8963_CRA_RATE_BIT, ak8963_CRA_RATE_LENGTH, buffer);
    return buffer[0];
}
void ak8963SetDataRate(uint8_t rate)
{
    i2cdevWriteBits(I2Cx, devAddr, ak8963_RA_CONFIG_A, ak8963_CRA_RATE_BIT, ak8963_CRA_RATE_LENGTH, rate);
}
uint8_t ak8963GetMeasurementBias()
{
    i2cdevReadBits(I2Cx, devAddr, ak8963_RA_CONFIG_A, ak8963_CRA_BIAS_BIT, ak8963_CRA_BIAS_LENGTH, buffer);
    return buffer[0];
}
void ak8963SetMeasurementBias(uint8_t bias)
{
    i2cdevWriteBits(I2Cx, devAddr, ak8963_RA_CONFIG_A, ak8963_CRA_BIAS_BIT, ak8963_CRA_BIAS_LENGTH, bias);
}
uint8_t ak8963GetGain()
{
    i2cdevReadBits(I2Cx, devAddr, ak8963_RA_CONFIG_B, ak8963_CRB_GAIN_BIT, ak8963_CRB_GAIN_LENGTH, buffer);
    return buffer[0];
}
void ak8963SetGain(uint8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, ak8963_RA_CONFIG_B, gain << (ak8963_CRB_GAIN_BIT - ak8963_CRB_GAIN_LENGTH + 1));
}
uint8_t ak8963GetMode()
{
    uint8_t cntl;
    if (i2cdevReadByte(I2Cx, devAddr, AK8963_RA_CNTL, &cntl)) {
        return cntl;
    }
    return 0;
}
void ak8963SetMode(uint8_t newMode)
{
    i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, newMode);
    mode = newMode; 
    vTaskDelay(M2T(1)); 
}
void ak8963GetHeading(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, AK8963_RA_ST1, 8, buffer);
    if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) {
        *x = ((int16_t)buffer[2] << 8) | buffer[1]; 
        *y = ((int16_t)buffer[4] << 8) | buffer[3]; 
        *z = ((int16_t)buffer[6] << 8) | buffer[5]; 
    } else {
        *x = *y = *z = 0;
    }
    if ((mode & 0x1F) == AK8963_MODE_SINGLE) {
        i2cdevWriteByte(I2Cx, devAddr, AK8963_RA_CNTL, mode);
    }
}
int16_t ak8963GetHeadingX()
{
    uint8_t data[8];
    i2cdevReadReg8(I2Cx, devAddr, AK8963_RA_ST1, 8, data);
    if (!(data[0] & (1 << AK8963_ST1_DRDY_BIT))) {
        return 0;
    }
    return ((int16_t)data[2] << 8) | data[1];
}
int16_t ak8963GetHeadingY()
{
    uint8_t data[8];
    i2cdevReadReg8(I2Cx, devAddr, AK8963_RA_ST1, 8, data);
    if (!(data[0] & (1 << AK8963_ST1_DRDY_BIT))) {
        return 0;
    }
    return ((int16_t)data[4] << 8) | data[3];
}
int16_t ak8963GetHeadingZ()
{
    uint8_t data[8];
    i2cdevReadReg8(I2Cx, devAddr, AK8963_RA_ST1, 8, data);
    if (!(data[0] & (1 << AK8963_ST1_DRDY_BIT))) {
        return 0;
    }
    return ((int16_t)data[6] << 8) | data[5];
}
bool ak8963GetLockStatus()
{
    i2cdevReadBit(I2Cx, devAddr, ak8963_RA_STATUS, ak8963_STATUS_LOCK_BIT, buffer);
    return buffer[0];
}
bool ak8963GetReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, ak8963_RA_STATUS, ak8963_STATUS_READY_BIT, buffer);
    return buffer[0];
}
uint8_t ak8963GetIDA()
{
    i2cdevReadByte(I2Cx, devAddr, ak8963_RA_ID_A, buffer);
    return buffer[0];
}
uint8_t ak8963GetIDB()
{
    i2cdevReadByte(I2Cx, devAddr, ak8963_RA_ID_B, buffer);
    return buffer[0];
}
uint8_t ak8963GetIDC()
{
    i2cdevReadByte(I2Cx, devAddr, ak8963_RA_ID_C, buffer);
    return buffer[0];
}