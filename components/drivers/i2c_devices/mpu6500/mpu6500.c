#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cdev.h"
#include "mpu6500.h"
#include "eprintf.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "mpu6500"
#include "debug_cf.h"
static uint8_t devAddr;
static I2C_Dev *I2Cx;
static uint8_t buffer[14];
static bool isInit;
void mpu6500Init(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }
    I2Cx = i2cPort;
#ifdef CONFIG_MPU_I2C_ADDR
    devAddr = CONFIG_MPU_I2C_ADDR;
#else
    devAddr = mpu6500_ADDRESS_AD0_LOW;
#endif
    isInit = true;
}
bool mpu6500Test(void)
{
    bool testStatus;
    if (!isInit) {
        return false;
    }
    testStatus = mpu6500TestConnection();
    return testStatus;
}
bool mpu6500TestConnection()
{
    vTaskDelay(M2T(100));
    uint8_t deviceId = mpu6500GetDeviceID();
    return (deviceId == 0x70 || deviceId == 0x71 || deviceId == 0b110100);
}
bool mpu6500SelfTest()
{
    bool testStatus = true;
    int16_t axi16, ayi16, azi16;
    int16_t gxi16, gyi16, gzi16;
    float axf, ayf, azf;
    float gxf, gyf, gzf;
    float axfTst, ayfTst, azfTst;
    float gxfTst, gyfTst, gzfTst;
    float axfDiff, ayfDiff, azfDiff;
    float gxfDiff, gyfDiff, gzfDiff;
    float gRange, aRange;
    uint32_t scrap;
    aRange = mpu6500GetFullScaleAccelGPL();
    gRange = mpu6500GetFullScaleGyroDPL();
    for (scrap = 0; scrap < 5; scrap++) {
        mpu6500GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
        vTaskDelay(M2T(2));
    }
    gxf = gxi16 * gRange;
    gyf = gyi16 * gRange;
    gzf = gzi16 * gRange;
    axf = axi16 * aRange;
    ayf = ayi16 * aRange;
    azf = azi16 * aRange;
    mpu6500SetGyroXSelfTest(true);
    mpu6500SetGyroYSelfTest(true);
    mpu6500SetGyroZSelfTest(true);
    mpu6500SetAccelXSelfTest(true);
    mpu6500SetAccelYSelfTest(true);
    mpu6500SetAccelZSelfTest(true);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mpu6500GetMotion6(&axi16, &ayi16, &azi16, &gxi16, &gyi16, &gzi16);
    gxfTst = gxi16 * gRange;
    gyfTst = gyi16 * gRange;
    gzfTst = gzi16 * gRange;
    axfTst = axi16 * aRange;
    ayfTst = ayi16 * aRange;
    azfTst = azi16 * aRange;
    mpu6500SetGyroXSelfTest(false);
    mpu6500SetGyroYSelfTest(false);
    mpu6500SetGyroZSelfTest(false);
    mpu6500SetAccelXSelfTest(false);
    mpu6500SetAccelYSelfTest(false);
    mpu6500SetAccelZSelfTest(false);
    gxfDiff = gxfTst - gxf;
    gyfDiff = gyfTst - gyf;
    gzfDiff = gzfTst - gzf;
    axfDiff = axfTst - axf;
    ayfDiff = ayfTst - ayf;
    azfDiff = azfTst - azf;
    if (mpu6500EvaluateSelfTest(mpu6500_ST_GYRO_LOW, mpu6500_ST_GYRO_HIGH, gxfDiff, "gyro X") &&
            mpu6500EvaluateSelfTest(-mpu6500_ST_GYRO_HIGH, -mpu6500_ST_GYRO_LOW, gyfDiff, "gyro Y") &&
            mpu6500EvaluateSelfTest(mpu6500_ST_GYRO_LOW, mpu6500_ST_GYRO_HIGH, gzfDiff, "gyro Z") &&
            mpu6500EvaluateSelfTest(mpu6500_ST_ACCEL_LOW, mpu6500_ST_ACCEL_HIGH, axfDiff, "acc X") &&
            mpu6500EvaluateSelfTest(mpu6500_ST_ACCEL_LOW, mpu6500_ST_ACCEL_HIGH, ayfDiff, "acc Y") &&
            mpu6500EvaluateSelfTest(mpu6500_ST_ACCEL_LOW, mpu6500_ST_ACCEL_HIGH, azfDiff, "acc Z")) {
        DEBUG_PRINTD("mpu6500 Self test [OK].\n");
    } else {
        testStatus = false;
    }
    return testStatus;
}
bool mpu6500EvaluateSelfTest(float low, float high, float value, char *string)
{
    if (value < low || value > high) {
        DEBUG_PRINTD("Self test %s [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
                     string, (double)low, (double)high, (double)value);
        return false;
    }
    return true;
}
uint8_t mpu6500GetAuxVDDIOLevel()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_YG_OFFS_TC, mpu6500_TC_PWR_MODE_BIT, buffer);
    return buffer[0];
}
void mpu6500SetAuxVDDIOLevel(uint8_t level)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_YG_OFFS_TC, mpu6500_TC_PWR_MODE_BIT, level);
}
uint8_t mpu6500GetRate()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_SMPLRT_DIV, buffer);
    return buffer[0];
}
void mpu6500SetRate(uint8_t rate)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_SMPLRT_DIV, rate);
}
uint8_t mpu6500GetExternalFrameSync()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_CONFIG, mpu6500_CFG_EXT_SYNC_SET_BIT,
                   mpu6500_CFG_EXT_SYNC_SET_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetExternalFrameSync(uint8_t sync)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_CONFIG, mpu6500_CFG_EXT_SYNC_SET_BIT,
                    mpu6500_CFG_EXT_SYNC_SET_LENGTH, sync);
}
uint8_t mpu6500GetDLPFMode()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_CONFIG, mpu6500_CFG_DLPF_CFG_BIT,
                   mpu6500_CFG_DLPF_CFG_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetDLPFMode(uint8_t mode)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_CONFIG, mpu6500_CFG_DLPF_CFG_BIT,
                    mpu6500_CFG_DLPF_CFG_LENGTH, mode);
}
uint8_t mpu6500GetFullScaleGyroRangeId()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_GYRO_CONFIG, mpu6500_GCONFIG_FS_SEL_BIT,
                   mpu6500_GCONFIG_FS_SEL_LENGTH, buffer);
    return buffer[0];
}
float mpu6500GetFullScaleGyroDPL()
{
    int32_t rangeId;
    float range;
    rangeId = mpu6500GetFullScaleGyroRangeId();
    switch (rangeId) {
        case mpu6500_GYRO_FS_250:
            range = mpu6500_DEG_PER_LSB_250;
            break;
        case mpu6500_GYRO_FS_500:
            range = mpu6500_DEG_PER_LSB_500;
            break;
        case mpu6500_GYRO_FS_1000:
            range = mpu6500_DEG_PER_LSB_1000;
            break;
        case mpu6500_GYRO_FS_2000:
            range = mpu6500_DEG_PER_LSB_2000;
            break;
        default:
            range = mpu6500_DEG_PER_LSB_1000;
            break;
    }
    return range;
}
void mpu6500SetFullScaleGyroRange(uint8_t range)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_GYRO_CONFIG, mpu6500_GCONFIG_FS_SEL_BIT,
                    mpu6500_GCONFIG_FS_SEL_LENGTH, range);
}
void mpu6500SetGyroXSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_GYRO_CONFIG, mpu6500_GCONFIG_XG_ST_BIT, enabled);
}
void mpu6500SetGyroYSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_GYRO_CONFIG, mpu6500_GCONFIG_YG_ST_BIT, enabled);
}
void mpu6500SetGyroZSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_GYRO_CONFIG, mpu6500_GCONFIG_ZG_ST_BIT, enabled);
}
bool mpu6500GetAccelXSelfTest()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_XA_ST_BIT, buffer);
    return buffer[0];
}
void mpu6500SetAccelXSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_XA_ST_BIT, enabled);
}
bool mpu6500GetAccelYSelfTest()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_YA_ST_BIT, buffer);
    return buffer[0];
}
void mpu6500SetAccelYSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_YA_ST_BIT, enabled);
}
bool mpu6500GetAccelZSelfTest()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_ZA_ST_BIT, buffer);
    return buffer[0];
}
void mpu6500SetAccelZSelfTest(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_ZA_ST_BIT, enabled);
}
uint8_t mpu6500GetFullScaleAccelRangeId()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_AFS_SEL_BIT,
                   mpu6500_ACONFIG_AFS_SEL_LENGTH, buffer);
    return buffer[0];
}
float mpu6500GetFullScaleAccelGPL()
{
    int32_t rangeId;
    float range;
    rangeId = mpu6500GetFullScaleAccelRangeId();
    switch (rangeId) {
        case mpu6500_ACCEL_FS_2:
            range = mpu6500_G_PER_LSB_2;
            break;
        case mpu6500_ACCEL_FS_4:
            range = mpu6500_G_PER_LSB_4;
            break;
        case mpu6500_ACCEL_FS_8:
            range = mpu6500_G_PER_LSB_8;
            break;
        case mpu6500_ACCEL_FS_16:
            range = mpu6500_G_PER_LSB_16;
            break;
        default:
            range = mpu6500_DEG_PER_LSB_1000;
            break;
    }
    return range;
}
void mpu6500SetFullScaleAccelRange(uint8_t range)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_AFS_SEL_BIT,
                    mpu6500_ACONFIG_AFS_SEL_LENGTH, range);
}
uint8_t mpu6500GetDHPFMode()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_ACCEL_HPF_BIT,
                   mpu6500_ACONFIG_ACCEL_HPF_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetDHPFMode(uint8_t bandwidth)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_ACCEL_CONFIG, mpu6500_ACONFIG_ACCEL_HPF_BIT,
                    mpu6500_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}
uint8_t mpu6500GetFreefallDetectionThreshold()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_FF_THR, buffer);
    return buffer[0];
}
void mpu6500SetFreefallDetectionThreshold(uint8_t threshold)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_FF_THR, threshold);
}
uint8_t mpu6500GetFreefallDetectionDuration()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_FF_DUR, buffer);
    return buffer[0];
}
void mpu6500SetFreefallDetectionDuration(uint8_t duration)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_FF_DUR, duration);
}
uint8_t mpu6500GetMotionDetectionThreshold()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_MOT_THR, buffer);
    return buffer[0];
}
void mpu6500SetMotionDetectionThreshold(uint8_t threshold)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_MOT_THR, threshold);
}
uint8_t mpu6500GetMotionDetectionDuration()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_MOT_DUR, buffer);
    return buffer[0];
}
void mpu6500SetMotionDetectionDuration(uint8_t duration)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_MOT_DUR, duration);
}
uint8_t mpu6500GetZeroMotionDetectionThreshold()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_ZRMOT_THR, buffer);
    return buffer[0];
}
void mpu6500SetZeroMotionDetectionThreshold(uint8_t threshold)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_ZRMOT_THR, threshold);
}
uint8_t mpu6500GetZeroMotionDetectionDuration()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_ZRMOT_DUR, buffer);
    return buffer[0];
}
void mpu6500SetZeroMotionDetectionDuration(uint8_t duration)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_ZRMOT_DUR, duration);
}
bool mpu6500GetTempFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_TEMP_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetTempFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_TEMP_FIFO_EN_BIT, enabled);
}
bool mpu6500GetXGyroFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_XG_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetXGyroFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_XG_FIFO_EN_BIT, enabled);
}
bool mpu6500GetYGyroFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_YG_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetYGyroFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_YG_FIFO_EN_BIT, enabled);
}
bool mpu6500GetZGyroFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_ZG_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetZGyroFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_ZG_FIFO_EN_BIT, enabled);
}
bool mpu6500GetAccelFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_ACCEL_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetAccelFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_ACCEL_FIFO_EN_BIT, enabled);
}
bool mpu6500GetSlave2FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_SLV2_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave2FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_SLV2_FIFO_EN_BIT, enabled);
}
bool mpu6500GetSlave1FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_SLV1_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave1FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_SLV1_FIFO_EN_BIT, enabled);
}
bool mpu6500GetSlave0FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_SLV0_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave0FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_FIFO_EN, mpu6500_SLV0_FIFO_EN_BIT, enabled);
}
bool mpu6500GetMultiMasterEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_MULT_MST_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetMultiMasterEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_MULT_MST_EN_BIT, enabled);
}
bool mpu6500GetWaitForExternalSensorEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_WAIT_FOR_ES_BIT, buffer);
    return buffer[0];
}
void mpu6500SetWaitForExternalSensorEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_WAIT_FOR_ES_BIT, enabled);
}
bool mpu6500GetSlave3FIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_SLV_3_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave3FIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_SLV_3_FIFO_EN_BIT, enabled);
}
bool mpu6500GetSlaveReadWriteTransitionEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_I2C_MST_P_NSR_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlaveReadWriteTransitionEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_I2C_MST_P_NSR_BIT, enabled);
}
uint8_t mpu6500GetMasterClockSpeed()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_I2C_MST_CLK_BIT,
                   mpu6500_I2C_MST_CLK_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetMasterClockSpeed(uint8_t speed)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_I2C_MST_CTRL, mpu6500_I2C_MST_CLK_BIT,
                    mpu6500_I2C_MST_CLK_LENGTH, speed);
}
uint8_t mpu6500GetSlaveAddress(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_ADDR + num * 3, buffer);
    return buffer[0];
}
void mpu6500SetSlaveAddress(uint8_t num, uint8_t address)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_ADDR + num * 3, address);
}
uint8_t mpu6500GetSlaveRegister(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_REG + num * 3, buffer);
    return buffer[0];
}
void mpu6500SetSlaveRegister(uint8_t num, uint8_t reg)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_REG + num * 3, reg);
}
bool mpu6500GetSlaveEnabled(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlaveEnabled(uint8_t num, bool enabled)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_EN_BIT,
                   enabled);
}
bool mpu6500GetSlaveWordByteSwap(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_BYTE_SW_BIT,
                  buffer);
    return buffer[0];
}
void mpu6500SetSlaveWordByteSwap(uint8_t num, bool enabled)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_BYTE_SW_BIT,
                   enabled);
}
bool mpu6500GetSlaveWriteMode(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_REG_DIS_BIT,
                  buffer);
    return buffer[0];
}
void mpu6500SetSlaveWriteMode(uint8_t num, bool mode)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_REG_DIS_BIT,
                   mode);
}
bool mpu6500GetSlaveWordGroupOffset(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_GRP_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlaveWordGroupOffset(uint8_t num, bool enabled)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_GRP_BIT,
                   enabled);
}
uint8_t mpu6500GetSlaveDataLength(uint8_t num)
{
    if (num > 3) {
        return 0;
    }
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_LEN_BIT,
                   mpu6500_I2C_SLV_LEN_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetSlaveDataLength(uint8_t num, uint8_t length)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_CTRL + num * 3, mpu6500_I2C_SLV_LEN_BIT,
                    mpu6500_I2C_SLV_LEN_LENGTH, length);
}
uint8_t mpu6500GetSlave4Address()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_ADDR, buffer);
    return buffer[0];
}
void mpu6500SetSlave4Address(uint8_t address)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_ADDR, address);
}
uint8_t mpu6500GetSlave4Register()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_REG, buffer);
    return buffer[0];
}
void mpu6500SetSlave4Register(uint8_t reg)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_REG, reg);
}
void mpu6500SetSlave4OutputByte(uint8_t data)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_DO, data);
}
bool mpu6500GetSlave4Enabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave4Enabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_EN_BIT, enabled);
}
bool mpu6500GetSlave4InterruptEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_INT_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave4InterruptEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_INT_EN_BIT, enabled);
}
bool mpu6500GetSlave4WriteMode()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_REG_DIS_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlave4WriteMode(bool mode)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_REG_DIS_BIT, mode);
}
uint8_t mpu6500GetSlave4MasterDelay()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_MST_DLY_BIT,
                   mpu6500_I2C_SLV4_MST_DLY_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetSlave4MasterDelay(uint8_t delay)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_CTRL, mpu6500_I2C_SLV4_MST_DLY_BIT,
                    mpu6500_I2C_SLV4_MST_DLY_LENGTH, delay);
}
uint8_t mpu6500GetSlate4InputByte()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV4_DI, buffer);
    return buffer[0];
}
bool mpu6500GetPassthroughStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_PASS_THROUGH_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetSlave4IsDone()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_SLV4_DONE_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetLostArbitration()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_LOST_ARB_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetSlave4Nack()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_SLV4_NACK_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetSlave3Nack()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_SLV3_NACK_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetSlave2Nack()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_SLV2_NACK_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetSlave1Nack()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_SLV1_NACK_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetSlave0Nack()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_STATUS, mpu6500_MST_I2C_SLV0_NACK_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetInterruptMode()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_INT_LEVEL_BIT, buffer);
    return buffer[0];
}
void mpu6500SetInterruptMode(bool mode)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_INT_LEVEL_BIT, mode);
}
bool mpu6500GetInterruptDrive()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_INT_OPEN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetInterruptDrive(bool drive)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_INT_OPEN_BIT, drive);
}
bool mpu6500GetInterruptLatch()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_LATCH_INT_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetInterruptLatch(bool latch)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_LATCH_INT_EN_BIT, latch);
}
bool mpu6500GetInterruptLatchClear()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_INT_RD_CLEAR_BIT, buffer);
    return buffer[0];
}
void mpu6500SetInterruptLatchClear(bool clear)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_INT_RD_CLEAR_BIT, clear);
}
bool mpu6500GetFSyncInterruptLevel()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_FSYNC_INT_LEVEL_BIT, buffer);
    return buffer[0];
}
void mpu6500SetFSyncInterruptLevel(bool level)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_FSYNC_INT_LEVEL_BIT, level);
}
bool mpu6500GetFSyncInterruptEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_FSYNC_INT_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetFSyncInterruptEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_FSYNC_INT_EN_BIT, enabled);
}
bool mpu6500GetI2CBypassEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_I2C_BYPASS_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetI2CBypassEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}
bool mpu6500GetClockOutputEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_CLKOUT_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetClockOutputEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_PIN_CFG, mpu6500_INTCFG_CLKOUT_EN_BIT, enabled);
}
uint8_t mpu6500GetIntEnabled()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, buffer);
    return buffer[0];
}
void mpu6500SetIntEnabled(uint8_t enabled)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, enabled);
}
bool mpu6500GetIntFreefallEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntFreefallEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_FF_BIT, enabled);
}
bool mpu6500GetIntMotionEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntMotionEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_MOT_BIT, enabled);
}
bool mpu6500GetIntZeroMotionEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntZeroMotionEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_ZMOT_BIT, enabled);
}
bool mpu6500GetIntFIFOBufferOverflowEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntFIFOBufferOverflowEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_FIFO_OFLOW_BIT, enabled);
}
bool mpu6500GetIntI2CMasterEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntI2CMasterEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_I2C_MST_INT_BIT, enabled);
}
bool mpu6500GetIntDataReadyEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntDataReadyEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_DATA_RDY_BIT, enabled);
}
uint8_t mpu6500GetIntStatus()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_INT_STATUS, buffer);
    return buffer[0];
}
bool mpu6500GetIntFreefallStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_FF_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntMotionStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_MOT_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntZeroMotionStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_ZMOT_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntFIFOBufferOverflowStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntI2CMasterStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_I2C_MST_INT_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntDataReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_DATA_RDY_BIT, buffer);
    return buffer[0];
}
void mpu6500GetMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz,
                       int16_t *mx, int16_t *my, int16_t *mz)
{
    mpu6500GetMotion6(ax, ay, az, gx, gy, gz);
}
void mpu6500GetMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t) buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t) buffer[2]) << 8) | buffer[3];
    *az = (((int16_t) buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t) buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t) buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t) buffer[12]) << 8) | buffer[13];
}
void mpu6500GetAcceleration(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ACCEL_XOUT_H, 6, buffer);
    *x = (((int16_t) buffer[0]) << 8) | buffer[1];
    *y = (((int16_t) buffer[2]) << 8) | buffer[3];
    *z = (((int16_t) buffer[4]) << 8) | buffer[5];
}
int16_t mpu6500GetAccelerationX()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ACCEL_XOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
int16_t mpu6500GetAccelerationY()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ACCEL_YOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
int16_t mpu6500GetAccelerationZ()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ACCEL_ZOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
int16_t mpu6500GetTemperature()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500GetRotation(int16_t *x, int16_t *y, int16_t *z)
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_GYRO_XOUT_H, 6, buffer);
    *x = (((int16_t) buffer[0]) << 8) | buffer[1];
    *y = (((int16_t) buffer[2]) << 8) | buffer[3];
    *z = (((int16_t) buffer[4]) << 8) | buffer[5];
}
int16_t mpu6500GetRotationX()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_GYRO_XOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
int16_t mpu6500GetRotationY()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_GYRO_YOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
int16_t mpu6500GetRotationZ()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_GYRO_ZOUT_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
uint8_t mpu6500GetExternalSensorByte(int position)
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_EXT_SENS_DATA_00 + position, buffer);
    return buffer[0];
}
uint16_t mpu6500GetExternalSensorWord(int position)
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_EXT_SENS_DATA_00 + position, 2, buffer);
    return (((uint16_t) buffer[0]) << 8) | buffer[1];
}
uint32_t mpu6500GetExternalSensorDWord(int position)
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_EXT_SENS_DATA_00 + position, 4, buffer);
    return (((uint32_t) buffer[0]) << 24) | (((uint32_t) buffer[1]) << 16)
           | (((uint16_t) buffer[2]) << 8) | buffer[3];
}
bool mpu6500GetXNegMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_XNEG_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetXPosMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_XPOS_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetYNegMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_YNEG_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetYPosMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_YPOS_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetZNegMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_ZNEG_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetZPosMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_ZPOS_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetZeroMotionDetected()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_STATUS, mpu6500_MOTION_MOT_ZRMOT_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSlaveOutputByte(uint8_t num, uint8_t data)
{
    if (num > 3) {
        return;
    }
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_I2C_SLV0_DO + num, data);
}
bool mpu6500GetExternalShadowDelayEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_DELAY_CTRL, mpu6500_DELAYCTRL_DELAY_ES_SHADOW_BIT,
                  buffer);
    return buffer[0];
}
void mpu6500SetExternalShadowDelayEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_DELAY_CTRL,
                   mpu6500_DELAYCTRL_DELAY_ES_SHADOW_BIT, enabled);
}
bool mpu6500GetSlaveDelayEnabled(uint8_t num)
{
    if (num > 4) {
        return 0;
    }
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_DELAY_CTRL, num, buffer);
    return buffer[0];
}
void mpu6500SetSlaveDelayEnabled(uint8_t num, bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_I2C_MST_DELAY_CTRL, num, enabled);
}
void mpu6500ResetGyroscopePath()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_SIGNAL_PATH_RESET, mpu6500_PATHRESET_GYRO_RESET_BIT, 1);
}
void mpu6500ResetAccelerometerPath()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_SIGNAL_PATH_RESET, mpu6500_PATHRESET_ACCEL_RESET_BIT, 1);
}
void mpu6500ResetTemperaturePath()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_SIGNAL_PATH_RESET, mpu6500_PATHRESET_TEMP_RESET_BIT, 1);
}
uint8_t mpu6500GetAccelerometerPowerOnDelay()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_CTRL, mpu6500_DETECT_ACCEL_ON_DELAY_BIT,
                   mpu6500_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetAccelerometerPowerOnDelay(uint8_t delay)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_CTRL, mpu6500_DETECT_ACCEL_ON_DELAY_BIT,
                    mpu6500_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}
uint8_t mpu6500GetFreefallDetectionCounterDecrement()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_CTRL, mpu6500_DETECT_FF_COUNT_BIT,
                   mpu6500_DETECT_FF_COUNT_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetFreefallDetectionCounterDecrement(uint8_t decrement)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_CTRL, mpu6500_DETECT_FF_COUNT_BIT,
                    mpu6500_DETECT_FF_COUNT_LENGTH, decrement);
}
uint8_t mpu6500GetMotionDetectionCounterDecrement()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_CTRL, mpu6500_DETECT_MOT_COUNT_BIT,
                   mpu6500_DETECT_MOT_COUNT_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetMotionDetectionCounterDecrement(uint8_t decrement)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_MOT_DETECT_CTRL, mpu6500_DETECT_MOT_COUNT_BIT,
                    mpu6500_DETECT_MOT_COUNT_LENGTH, decrement);
}
bool mpu6500GetFIFOEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_FIFO_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetFIFOEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_FIFO_EN_BIT, enabled);
}
bool mpu6500GetI2CMasterModeEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_I2C_MST_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetI2CMasterModeEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_I2C_MST_EN_BIT, enabled);
}
void mpu6500SwitchSPIEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_I2C_IF_DIS_BIT, enabled);
}
void mpu6500ResetFIFO()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_FIFO_RESET_BIT, 1);
}
void mpu6500ResetI2CMaster()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_I2C_MST_RESET_BIT, 1);
}
void mpu6500ResetSensors()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_SIG_COND_RESET_BIT, 1);
}
void mpu6500Reset()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_DEVICE_RESET_BIT, 1);
}
bool mpu6500GetSleepEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_SLEEP_BIT, buffer);
    return buffer[0];
}
void mpu6500SetSleepEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_SLEEP_BIT, enabled);
}
bool mpu6500GetWakeCycleEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_CYCLE_BIT, buffer);
    return buffer[0];
}
void mpu6500SetWakeCycleEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_CYCLE_BIT, enabled);
}
bool mpu6500GetTempSensorEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_TEMP_DIS_BIT, buffer);
    return buffer[0] == 0; 
}
void mpu6500SetTempSensorEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_TEMP_DIS_BIT, !enabled);
}
uint8_t mpu6500GetClockSource()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_CLKSEL_BIT,
                   mpu6500_PWR1_CLKSEL_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetClockSource(uint8_t source)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_1, mpu6500_PWR1_CLKSEL_BIT,
                    mpu6500_PWR1_CLKSEL_LENGTH, source);
}
uint8_t mpu6500GetWakeFrequency()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_LP_WAKE_CTRL_BIT,
                   mpu6500_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetWakeFrequency(uint8_t frequency)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_LP_WAKE_CTRL_BIT,
                    mpu6500_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}
bool mpu6500GetStandbyXAccelEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_XA_BIT, buffer);
    return buffer[0];
}
void mpu6500SetStandbyXAccelEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_XA_BIT, enabled);
}
bool mpu6500GetStandbyYAccelEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_YA_BIT, buffer);
    return buffer[0];
}
void mpu6500SetStandbyYAccelEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_YA_BIT, enabled);
}
bool mpu6500GetStandbyZAccelEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_ZA_BIT, buffer);
    return buffer[0];
}
void mpu6500SetStandbyZAccelEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_ZA_BIT, enabled);
}
bool mpu6500GetStandbyXGyroEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_XG_BIT, buffer);
    return buffer[0];
}
void mpu6500SetStandbyXGyroEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_XG_BIT, enabled);
}
bool mpu6500GetStandbyYGyroEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_YG_BIT, buffer);
    return buffer[0];
}
void mpu6500SetStandbyYGyroEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_YG_BIT, enabled);
}
bool mpu6500GetStandbyZGyroEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_ZG_BIT, buffer);
    return buffer[0];
}
void mpu6500SetStandbyZGyroEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_PWR_MGMT_2, mpu6500_PWR2_STBY_ZG_BIT, enabled);
}
uint16_t mpu6500GetFIFOCount()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_FIFO_COUNTH, 2, buffer);
    return (((uint16_t) buffer[0]) << 8) | buffer[1];
}
uint8_t mpu6500GetFIFOByte()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_FIFO_R_W, buffer);
    return buffer[0];
}
void mpu6500GetFIFOBytes(uint8_t *data, uint8_t length)
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_FIFO_R_W, length, data);
}
void mpu6500SetFIFOByte(uint8_t data)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_FIFO_R_W, data);
}
uint8_t mpu6500GetDeviceID()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_WHO_AM_I, mpu6500_WHO_AM_I_BIT, mpu6500_WHO_AM_I_LENGTH,
                   buffer);
    return buffer[0];
}
void mpu6500SetDeviceID(uint8_t id)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_WHO_AM_I, mpu6500_WHO_AM_I_BIT, mpu6500_WHO_AM_I_LENGTH,
                    id);
}
uint8_t mpu6500GetOTPBankValid()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_XG_OFFS_TC, mpu6500_TC_OTP_BNK_VLD_BIT, buffer);
    return buffer[0];
}
void mpu6500SetOTPBankValid(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_XG_OFFS_TC, mpu6500_TC_OTP_BNK_VLD_BIT, enabled);
}
int8_t mpu6500GetXGyroOffset()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_XG_OFFS_TC, mpu6500_TC_OFFSET_BIT,
                   mpu6500_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetXGyroOffset(int8_t offset)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_XG_OFFS_TC, mpu6500_TC_OFFSET_BIT,
                    mpu6500_TC_OFFSET_LENGTH, offset);
}
int8_t mpu6500GetYGyroOffset()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_YG_OFFS_TC, mpu6500_TC_OFFSET_BIT,
                   mpu6500_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetYGyroOffset(int8_t offset)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_YG_OFFS_TC, mpu6500_TC_OFFSET_BIT,
                    mpu6500_TC_OFFSET_LENGTH, offset);
}
int8_t mpu6500GetZGyroOffset()
{
    i2cdevReadBits(I2Cx, devAddr, mpu6500_RA_ZG_OFFS_TC, mpu6500_TC_OFFSET_BIT,
                   mpu6500_TC_OFFSET_LENGTH, buffer);
    return buffer[0];
}
void mpu6500SetZGyroOffset(int8_t offset)
{
    i2cdevWriteBits(I2Cx, devAddr, mpu6500_RA_ZG_OFFS_TC, mpu6500_TC_OFFSET_BIT,
                    mpu6500_TC_OFFSET_LENGTH, offset);
}
int8_t mpu6500GetXFineGain()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_X_FINE_GAIN, buffer);
    return buffer[0];
}
void mpu6500SetXFineGain(int8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_X_FINE_GAIN, gain);
}
int8_t mpu6500GetYFineGain()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_Y_FINE_GAIN, buffer);
    return buffer[0];
}
void mpu6500SetYFineGain(int8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_Y_FINE_GAIN, gain);
}
int8_t mpu6500GetZFineGain()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_Z_FINE_GAIN, buffer);
    return buffer[0];
}
void mpu6500SetZFineGain(int8_t gain)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_Z_FINE_GAIN, gain);
}
int16_t mpu6500GetXAccelOffset()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_XA_OFFS_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500SetXAccelOffset(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_XA_OFFS_H, 2, (uint8_t *)&offset);
}
int16_t mpu6500GetYAccelOffset()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_YA_OFFS_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500SetYAccelOffset(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_YA_OFFS_H, 2, (uint8_t *)&offset);
}
int16_t mpu6500GetZAccelOffset()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ZA_OFFS_H, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500SetZAccelOffset(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_ZA_OFFS_H, 2, (uint8_t *)&offset);
}
int16_t mpu6500GetXGyroOffsetUser()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_XG_OFFS_USRH, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500SetXGyroOffsetUser(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_XG_OFFS_USRH, 2, (uint8_t *)&offset);
}
int16_t mpu6500GetYGyroOffsetUser()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_YG_OFFS_USRH, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500SetYGyroOffsetUser(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_YG_OFFS_USRH, 2, (uint8_t *)&offset);
}
int16_t mpu6500GetZGyroOffsetUser()
{
    i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_ZG_OFFS_USRH, 2, buffer);
    return (((int16_t) buffer[0]) << 8) | buffer[1];
}
void mpu6500SetZGyroOffsetUser(int16_t offset)
{
    i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_ZG_OFFS_USRH, 2, (uint8_t *)&offset);
}
bool mpu6500GetIntPLLReadyEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_PLL_RDY_INT_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntPLLReadyEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_PLL_RDY_INT_BIT, enabled);
}
bool mpu6500GetIntDMPEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_DMP_INT_BIT, buffer);
    return buffer[0];
}
void mpu6500SetIntDMPEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_INT_ENABLE, mpu6500_INTERRUPT_DMP_INT_BIT, enabled);
}
bool mpu6500GetDMPInt5Status()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_DMP_INT_STATUS, mpu6500_DMPINT_5_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetDMPInt4Status()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_DMP_INT_STATUS, mpu6500_DMPINT_4_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetDMPInt3Status()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_DMP_INT_STATUS, mpu6500_DMPINT_3_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetDMPInt2Status()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_DMP_INT_STATUS, mpu6500_DMPINT_2_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetDMPInt1Status()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_DMP_INT_STATUS, mpu6500_DMPINT_1_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetDMPInt0Status()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_DMP_INT_STATUS, mpu6500_DMPINT_0_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntPLLReadyStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_PLL_RDY_INT_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetIntDMPStatus()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_INT_STATUS, mpu6500_INTERRUPT_DMP_INT_BIT, buffer);
    return buffer[0];
}
bool mpu6500GetDMPEnabled()
{
    i2cdevReadBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_DMP_EN_BIT, buffer);
    return buffer[0];
}
void mpu6500SetDMPEnabled(bool enabled)
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_DMP_EN_BIT, enabled);
}
void mpu6500ResetDMP()
{
    i2cdevWriteBit(I2Cx, devAddr, mpu6500_RA_USER_CTRL, mpu6500_USERCTRL_DMP_RESET_BIT, 1);
}
void mpu6500SetMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank)
{
    bank &= 0x1F;
    if (userBank) {
        bank |= 0x20;
    }
    if (prefetchEnabled) {
        bank |= 0x40;
    }
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_BANK_SEL, bank);
}
void mpu6500SetMemoryStartAddress(uint8_t address)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_MEM_START_ADDR, address);
}
uint8_t mpu6500ReadMemoryByte()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_MEM_R_W, buffer);
    return buffer[0];
}
void mpu6500WriteMemoryByte(uint8_t data)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_MEM_R_W, data);
}
void mpu6500ReadMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address)
{
    mpu6500SetMemoryBank(bank, true, true);
    mpu6500SetMemoryStartAddress(address);
    uint8_t chunkSize;
    uint16_t i;
    for (i = 0; i < dataSize;) {
        chunkSize = mpu6500_DMP_MEMORY_CHUNK_SIZE;
        if (i + chunkSize > dataSize) {
            chunkSize = dataSize - i;
        }
        if (chunkSize > 256 - address) {
            chunkSize = 256 - address;
        }
        i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_MEM_R_W, chunkSize, data + i);
        i += chunkSize;
        address += chunkSize;
        if (i < dataSize) {
            if (address == 0) {
                bank++;
            }
            mpu6500SetMemoryBank(bank, true, true);
            mpu6500SetMemoryStartAddress(address);
        }
    }
}
bool mpu6500WriteMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address,
                             bool verify)
{
    static uint8_t verifyBuffer[mpu6500_DMP_MEMORY_CHUNK_SIZE];
    uint8_t chunkSize;
    uint8_t *progBuffer;
    uint16_t i;
    mpu6500SetMemoryBank(bank, true, true);
    mpu6500SetMemoryStartAddress(address);
    for (i = 0; i < dataSize;) {
        chunkSize = mpu6500_DMP_MEMORY_CHUNK_SIZE;
        if (i + chunkSize > dataSize) {
            chunkSize = dataSize - i;
        }
        if (chunkSize > 256 - address) {
            chunkSize = 256 - address;
        }
        progBuffer = (uint8_t *) data + i;
        i2cdevWriteReg8(I2Cx, devAddr, mpu6500_RA_MEM_R_W, chunkSize, progBuffer);
        if (verify) {
            uint32_t j;
            mpu6500SetMemoryBank(bank, true, true);
            mpu6500SetMemoryStartAddress(address);
            i2cdevReadReg8(I2Cx, devAddr, mpu6500_RA_MEM_R_W, chunkSize, verifyBuffer);
            for (j = 0; j < chunkSize; j++) {
                if (progBuffer[j] != verifyBuffer[j]) {
                    return false;
                }
            }
        }
        i += chunkSize;
        address += chunkSize;
        if (i < dataSize) {
            if (address == 0) {
                bank++;
            }
            mpu6500SetMemoryBank(bank, true, true);
            mpu6500SetMemoryStartAddress(address);
        }
    }
    return true;
}
bool mpu6500WriteProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank,
                                 uint8_t address, bool verify)
{
    return mpu6500WriteMemoryBlock(data, dataSize, bank, address, verify);
}
#define mpu6500_DMP_CONFIG_BLOCK_SIZE 6
bool mpu6500WriteDMPConfigurationSet(const uint8_t *data, uint16_t dataSize)
{
    uint8_t *progBuffer, success, special;
    uint16_t i;
    uint8_t bank = 0;
    uint8_t offset = 0;
    uint8_t length = 0;
    for (i = 0; i < dataSize;) {
        bank = data[i++];
        offset = data[i++];
        length = data[i++];
    }
    if (length > 0) {
        progBuffer = (uint8_t *) data + i;
        success = mpu6500WriteMemoryBlock(progBuffer, length, bank, offset, true);
        i += length;
    } else {
        special = data[i++];
        if (special == 0x01) {
            mpu6500SetIntZeroMotionEnabled(true);
            mpu6500SetIntFIFOBufferOverflowEnabled(true);
            mpu6500SetIntDMPEnabled(true);
            success = true;
        } else {
            success = false;
        }
    }
    if (!success) {
        return false; 
    }
    return true;
}
bool mpu6500WriteProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize)
{
    return mpu6500WriteDMPConfigurationSet(data, dataSize);
}
uint8_t mpu6500GetDMPConfig1()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_DMP_CFG_1, buffer);
    return buffer[0];
}
void mpu6500SetDMPConfig1(uint8_t config)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_DMP_CFG_1, config);
}
uint8_t mpu6500GetDMPConfig2()
{
    i2cdevReadByte(I2Cx, devAddr, mpu6500_RA_DMP_CFG_2, buffer);
    return buffer[0];
}
void mpu6500SetDMPConfig2(uint8_t config)
{
    i2cdevWriteByte(I2Cx, devAddr, mpu6500_RA_DMP_CFG_2, config);
}