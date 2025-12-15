#include "sensors_mpu9250_lps25h.h"
#include <math.h>
#include "config.h"
// Removed: #include "lps25h.h" - Barometer hardware not available
#include "mpu6500.h"
#include "ak8963.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "system.h"
#include "configblock.h"
#include "param.h"
#include "log.h"
#include "debug_cf.h"
#include "imu.h"
#include "ledseq.h"
// DÜZELTME D2: Buzzer kullanılmıyor - sound.h kaldırıldı
#include "filter.h"
#include "static_mem.h"
#include "rateSupervisor.h"
#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#endif
#define MAG_GAUSS_PER_LSB     666.7f
#define SENSORS_GYRO_FS_CFG       mpu6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   mpu6500_DEG_PER_LSB_2000
#define SENSORS_ACCEL_FS_CFG      mpu6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG     mpu6500_G_PER_LSB_16
#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000) 
#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f      
#define SENSORS_BIAS_SAMPLES       1000
#define SENSORS_ACC_SCALE_SAMPLES  200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN        8
// Removed: SENSORS_BARO_BUFF_LEN - Barometer hardware not available
#define SENSORS_BARO_BUFF_LEN       0
#define GYRO_NBR_OF_AXES            3
#define GYRO_MIN_BIAS_TIMEOUT_MS    M2T(1*1000)
#define SENSORS_NBR_OF_BIAS_SAMPLES     1024
#define GYRO_VARIANCE_BASE          5000
#define GYRO_VARIANCE_THRESHOLD_X   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y   (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z   (GYRO_VARIANCE_BASE)
typedef struct
{
  Axis3f     bias;
  Axis3f     variance;
  Axis3f     mean;
  bool       isBiasValueFound;
  bool       isBufferFilled;
  Axis3i16*  bufHead;
  Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;
static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
// Removed: barometerDataQueue - Barometer hardware not available
static xSemaphoreHandle sensorsDataReady;
static StaticSemaphore_t sensorsDataReadyBuffer;
static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;
static bool isInit = false;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;
static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
NO_DMA_CCM_SAFE_ZERO_INIT static BiasObj gyroBiasRunning;
static Axis3f  gyroBias;
#if defined(SENSORS_GYRO_BIAS_CALCULATE_STDDEV) && defined (GYRO_BIAS_LIGHT_WEIGHT)
static Axis3f  gyroBiasStdDev;
#endif
static bool    gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;
#define GYRO_LPF_CUTOFF_FREQ  100
#define ACCEL_LPF_CUTOFF_FREQ 50
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
// Removed: isBarometerPresent - Barometer hardware not available
static bool isMagnetometerPresent = false;
static rateSupervisor_t imuRateSupervisorContext;
static bool imuRateWarningDisplayed = false;
// AK8963 ASA factory adjustment values (for magnetometer sensitivity correction)
static uint8_t ak8963_asa[3] = {128, 128, 128}; // Default values (1.0 scale factor)
static bool ak8963_asa_valid = false;
// Timeout log rate limiting
static uint32_t lastTimeoutLogTime = 0;
#define TIMEOUT_LOG_INTERVAL_MS 1000 // Log timeout at most once per second
#ifndef CONFIG_MPU_PIN_INT
#define CONFIG_MPU_PIN_INT 4
#endif
#define CONFIG_MPU9250_INT_GPIO CONFIG_MPU_PIN_INT
#ifdef ESP_PLATFORM
void IRAM_ATTR mpu9250_isr_handler(void* arg) {
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}
#endif
static bool isMpu6500TestPassed = false;
static bool isAK8963TestPassed = false;
// Removed: isLPS25HTestPassed - Barometer hardware not available
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;
static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};
// Removed: processBarometerMeasurements - Barometer hardware not available
static void processAccGyroMeasurements(const uint8_t *buffer);
static void processMagnetometerMeasurements(const uint8_t *buffer);
static void sensorsSetupSlaveRead(void);
#ifdef GYRO_GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz,  Axis3f *gyroBiasOut);
#endif
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static void sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out);
STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);
bool sensorsMpu9250Lps25hReadGyro(Axis3f *gyro)
{
  return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
bool sensorsMpu9250Lps25hReadAcc(Axis3f *acc)
{
  return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
bool sensorsMpu9250Lps25hReadMag(Axis3f *mag)
{
  return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
// Removed: sensorsMpu9250Lps25hReadBaro - Barometer hardware not available
void sensorsMpu9250Lps25hAcquire(sensorData_t *sensors, const uint32_t tick)
{
  sensorsReadGyro(&sensors->gyro);
  sensorsReadAcc(&sensors->acc);
  sensorsReadMag(&sensors->mag);
  // Removed: sensorsReadBaro - Barometer hardware not available
  sensors->interruptTimestamp = sensorData.interruptTimestamp;
}
bool sensorsMpu9250Lps25hAreCalibrated() {
  return gyroBiasFound;
}
static void sensorsTask(void *param)
{
  systemWaitStart();
  sensorsSetupSlaveRead();
  uint32_t currentTimeMs = T2M(xTaskGetTickCount());
  rateSupervisorInit(&imuRateSupervisorContext, currentTimeMs, M2T(1000), 997, 1003, 1);
  imuRateWarningDisplayed = false;
  static uint32_t measurement_count = 0;
  static uint32_t i2c_error_count = 0;
  const uint32_t LOG_INTERVAL = 1000; 
  DEBUG_PRINTI("MPU9250 sensor task started, rate: 1000Hz\n");
  while (1)
  {
    if (pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY))
    {
      sensorData.interruptTimestamp = imuIntTimestamp;
      measurement_count++;
      uint32_t interruptTimeMs = (uint32_t)(imuIntTimestamp / 1000);
      if (!rateSupervisorValidate(&imuRateSupervisorContext, interruptTimeMs)) {
        if (!imuRateWarningDisplayed) {
          DEBUG_PRINT("WARNING: IMU interrupt rate is off (%lu Hz)\n", (unsigned long)rateSupervisorLatestCount(&imuRateSupervisorContext));
          imuRateWarningDisplayed = true;
        }
      } else {
        imuRateWarningDisplayed = false;
      }
      uint8_t dataLen = (uint8_t) (SENSORS_MPU6500_BUFF_LEN +
              (isMagnetometerPresent ? SENSORS_MAG_BUFF_LEN : 0)
              // Removed: barometer buffer length - Barometer hardware not available
              );
      uint8_t mpu_addr = CONFIG_MPU_I2C_ADDR;
      if (!i2cdevReadReg8(I2C0_DEV, mpu_addr, mpu6500_RA_ACCEL_XOUT_H, dataLen, buffer)) {
        i2c_error_count++;
        if (i2c_error_count % 100 == 0) { 
          DEBUG_PRINTW("MPU9250: I2C read failed (errors: %lu)\n", (unsigned long)i2c_error_count);
        }
        xSemaphoreGive(dataReady);
        continue;
      }
      processAccGyroMeasurements(&(buffer[0]));
      if (isMagnetometerPresent)
      {
          processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
      }
      // Removed: processBarometerMeasurements - Barometer hardware not available
      xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
      xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
      if (isMagnetometerPresent)
      {
        xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
      }
      // Removed: barometerDataQueue overwrite - Barometer hardware not available
      if (measurement_count % LOG_INTERVAL == 0) {
        DEBUG_PRINTI("MPU9250: gyro=[%.2f,%.2f,%.2f]deg/s acc=[%.3f,%.3f,%.3f]g calib=%d errors=%lu\n",
                     sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z,
                     sensorData.acc.x, sensorData.acc.y, sensorData.acc.z,
                     gyroBiasFound ? 1 : 0,
                     (unsigned long)i2c_error_count);
      }
      xSemaphoreGive(dataReady);
    }
  }
}
void sensorsMpu9250Lps25hWaitDataReady(void)
{
  #define SENSOR_DATA_TIMEOUT_MS 10  
  TickType_t timeout = pdMS_TO_TICKS(SENSOR_DATA_TIMEOUT_MS);
  if (xSemaphoreTake(dataReady, timeout) != pdTRUE) {
    // Rate-limit timeout logs to prevent spam (max once per second)
    uint32_t currentTime = xTaskGetTickCount();
    if (currentTime - lastTimeoutLogTime >= pdMS_TO_TICKS(TIMEOUT_LOG_INTERVAL_MS)) {
      DEBUG_PRINT("CRITICAL: Sensor data ready timeout in stabilizer!\n");
      lastTimeoutLogTime = currentTime;
    }
  }
}
// Removed: processBarometerMeasurements - Barometer hardware not available
void processMagnetometerMeasurements(const uint8_t *buffer)
{
  // Check ST1: Data Ready bit
  if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) {
    // Check ST2: Overflow (HOFL) bit - discard data if overflow occurred
    uint8_t st2 = buffer[7];
    if (st2 & (1 << AK8963_ST2_HOFL_BIT)) {
      // Overflow detected - discard this reading, keep previous values
      return;
    }
    
    int16_t headingx_raw = (((int16_t) buffer[2]) << 8) | buffer[1];
    int16_t headingy_raw = (((int16_t) buffer[4]) << 8) | buffer[3];
    int16_t headingz_raw = (((int16_t) buffer[6]) << 8) | buffer[5];
    
    // Apply ASA factory adjustment for sensitivity correction
    // Formula: adjusted = raw * ((ASA - 128) * 0.5 / 128 + 1)
    float asa_scale_x = 1.0f;
    float asa_scale_y = 1.0f;
    float asa_scale_z = 1.0f;
    
    if (ak8963_asa_valid) {
      asa_scale_x = ((float)(ak8963_asa[0] - 128) * 0.5f / 128.0f + 1.0f);
      asa_scale_y = ((float)(ak8963_asa[1] - 128) * 0.5f / 128.0f + 1.0f);
      asa_scale_z = ((float)(ak8963_asa[2] - 128) * 0.5f / 128.0f + 1.0f);
    }
    
    float magX = (float)headingx_raw * asa_scale_x / MAG_GAUSS_PER_LSB;
    float magY = (float)headingy_raw * asa_scale_y / MAG_GAUSS_PER_LSB;
    float magZ = (float)headingz_raw * asa_scale_z / MAG_GAUSS_PER_LSB;
    
    // Validate magnetometer values - reject NaN/Inf and extreme values
    if (isnan(magX) || isinf(magX) || fabsf(magX) > 10.0f) {
      // Keep previous value or set to 0
      return;
    }
    if (isnan(magY) || isinf(magY) || fabsf(magY) > 10.0f) {
      return;
    }
    if (isnan(magZ) || isinf(magZ) || fabsf(magZ) > 10.0f) {
      return;
    }
    
    sensorData.mag.x = magX;
    sensorData.mag.y = magY;
    sensorData.mag.z = magZ;
  }
}
void processAccGyroMeasurements(const uint8_t *buffer)
{
  Axis3f accScaled;
  accelRaw.y = (((int16_t) buffer[0]) << 8) | buffer[1];
  accelRaw.x = (((int16_t) buffer[2]) << 8) | buffer[3];
  accelRaw.z = (((int16_t) buffer[4]) << 8) | buffer[5];
  gyroRaw.y = (((int16_t) buffer[8]) << 8) | buffer[9];
  gyroRaw.x = (((int16_t) buffer[10]) << 8) | buffer[11];
  gyroRaw.z = (((int16_t) buffer[12]) << 8) | buffer[13];
#ifdef GYRO_BIAS_LIGHT_WEIGHT
  gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
  gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif
  if (gyroBiasFound)
  {
     processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
  }
  // Calculate gyro values with bias correction
  float gyroX = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
  float gyroY =  (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
  float gyroZ =  (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
  
  // Validate gyro values - reject NaN/Inf and extreme values
  if (isnan(gyroX) || isinf(gyroX) || fabsf(gyroX) > 2000.0f) gyroX = 0.0f;
  if (isnan(gyroY) || isinf(gyroY) || fabsf(gyroY) > 2000.0f) gyroY = 0.0f;
  if (isnan(gyroZ) || isinf(gyroZ) || fabsf(gyroZ) > 2000.0f) gyroZ = 0.0f;
  
  sensorData.gyro.x = gyroX;
  sensorData.gyro.y = gyroY;
  sensorData.gyro.z = gyroZ;
  applyAxis3fLpf((lpf2pData*)(&gyroLpf), &sensorData.gyro);
  
  // Validate accScale
  if (accScale < 0.5f || accScale > 2.0f || isnan(accScale) || isinf(accScale)) {
    accScale = 1.0f; 
  }
  
  accScaled.x = -(accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.y =  (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
  accScaled.z =  (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;
  
  // Validate accelerometer values - reject NaN/Inf and extreme values
  if (isnan(accScaled.x) || isinf(accScaled.x) || fabsf(accScaled.x) > 20.0f) accScaled.x = 0.0f;
  if (isnan(accScaled.y) || isinf(accScaled.y) || fabsf(accScaled.y) > 20.0f) accScaled.y = 0.0f;
  if (isnan(accScaled.z) || isinf(accScaled.z) || fabsf(accScaled.z) > 20.0f) accScaled.z = 0.0f;
  
  sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
  
  // Validate final accelerometer values
  if (isnan(sensorData.acc.x) || isinf(sensorData.acc.x)) sensorData.acc.x = 0.0f;
  if (isnan(sensorData.acc.y) || isinf(sensorData.acc.y)) sensorData.acc.y = 0.0f;
  if (isnan(sensorData.acc.z) || isinf(sensorData.acc.z)) sensorData.acc.z = -9.81f; // Default to gravity
  
  applyAxis3fLpf((lpf2pData*)(&accLpf), &sensorData.acc);
}
static void sensorsDeviceInit(void)
{
  isMagnetometerPresent = false;
// Removed: isBarometerPresent initialization - Barometer hardware not available
  while (xTaskGetTickCount() < 2000){
    vTaskDelay(M2T(50));
  };
  i2cdevInit(I2C0_DEV);
  mpu6500Init(I2C0_DEV);
  if (mpu6500TestConnection() == true)
  {
    DEBUG_PRINT("MPU9250 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MPU9250 I2C connection [FAIL].\n");
  }
  mpu6500Reset();
  vTaskDelay(M2T(50));
  mpu6500SetSleepEnabled(false);
  vTaskDelay(M2T(100));
  mpu6500SetClockSource(mpu6500_CLOCK_PLL_XGYRO);
  vTaskDelay(M2T(200));
  mpu6500SetTempSensorEnabled(true);
  mpu6500SetIntEnabled(false);
  mpu6500SetI2CBypassEnabled(true);
  mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);
  mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);
#if SENSORS_MPU6500_DLPF_256HZ
  mpu6500SetRate(7);
  mpu6500SetDLPFMode(mpu6500_DLPF_BW_256);
#else
  mpu6500SetRate(0);
  mpu6500SetDLPFMode(mpu6500_DLPF_BW_98);
  for (uint8_t i = 0; i < 3; i++)
  {
    lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
    lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
  }
#endif
#ifdef SENSORS_ENABLE_MAG_AK8963
  ak8963Init(I2C0_DEV);
  if (ak8963TestConnection() == true)
  {
    isMagnetometerPresent = true;
    // Read ASA factory adjustment values for sensitivity correction
    uint8_t devAddr = AK8963_ADDRESS_00;
    i2cdevWriteByte(I2C0_DEV, devAddr, AK8963_RA_CNTL, AK8963_MODE_FUSE_ROM);
    vTaskDelay(M2T(10));
    if (i2cdevReadReg8(I2C0_DEV, devAddr, AK8963_RA_ASAX, 3, ak8963_asa)) {
      ak8963_asa_valid = true;
      DEBUG_PRINT("AK8963 ASA values: X=%d Y=%d Z=%d\n", ak8963_asa[0], ak8963_asa[1], ak8963_asa[2]);
    } else {
      ak8963_asa_valid = false;
      DEBUG_PRINT("AK8963 ASA read failed, using default values\n");
    }
    i2cdevWriteByte(I2C0_DEV, devAddr, AK8963_RA_CNTL, AK8963_MODE_POWER_DOWN);
    vTaskDelay(M2T(10));
    ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); 
    DEBUG_PRINT("AK8963 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("AK8963 I2C connection [FAIL].\n");
  }
#endif
  // Removed: lps25hInit - Barometer hardware not available
  cosPitch = cosf(configblockGetCalibPitch() * (float) M_PI/180);
  sinPitch = sinf(configblockGetCalibPitch() * (float) M_PI/180);
  cosRoll = cosf(configblockGetCalibRoll() * (float) M_PI/180);
  sinRoll = sinf(configblockGetCalibRoll() * (float) M_PI/180);
}
static void sensorsSetupSlaveRead(void)
{
#ifdef SENSORS_MPU6500_DLPF_256HZ
  mpu6500SetSlave4MasterDelay(15); 
#else
  mpu6500SetSlave4MasterDelay(9); 
#endif
  mpu6500SetI2CBypassEnabled(false);
  mpu6500SetWaitForExternalSensorEnabled(true); 
  mpu6500SetInterruptMode(0); 
  mpu6500SetInterruptDrive(0); 
  mpu6500SetInterruptLatch(0); 
  mpu6500SetInterruptLatchClear(1); 
  mpu6500SetSlaveReadWriteTransitionEnabled(false); 
  mpu6500SetMasterClockSpeed(13); 
#ifdef SENSORS_ENABLE_MAG_AK8963
  if (isMagnetometerPresent)
  {
    mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); 
    mpu6500SetSlaveRegister(0, AK8963_RA_ST1); 
    mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); 
    mpu6500SetSlaveDelayEnabled(0, true);
    mpu6500SetSlaveEnabled(0, true);
  }
#endif
  // Removed: barometer slave setup - Barometer hardware not available
  mpu6500SetI2CMasterModeEnabled(true);
  mpu6500SetIntDataReadyEnabled(true);
}
static void sensorsTaskInit(void)
{
  accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
  gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
  magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
  // Removed: barometerDataQueue creation - Barometer hardware not available
  STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}
static void sensorsInterruptInit(void)
{
  sensorsDataReady = xSemaphoreCreateBinaryStatic(&sensorsDataReadyBuffer);
  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);
  #ifdef ESP_PLATFORM
  gpio_config_t io_conf = {
    .intr_type = GPIO_INTR_POSEDGE,
    .pin_bit_mask = (1ULL << CONFIG_MPU9250_INT_GPIO),
    .mode = GPIO_MODE_INPUT,
    .pull_down_en = 0,
    .pull_up_en = 1  // Changed: Use pull-up for MPU interrupt line (typical configuration)
  };
  gpio_config(&io_conf);
  static bool isr_service_installed = false;
  if (!isr_service_installed) {
    gpio_install_isr_service(0);
    isr_service_installed = true;
  }
  gpio_isr_handler_add(CONFIG_MPU9250_INT_GPIO, mpu9250_isr_handler, NULL);
  #endif 
}
void sensorsMpu9250Lps25hInit(void)
{
  if (isInit)
  {
    return;
  }
  sensorsBiasObjInit(&gyroBiasRunning);
  sensorsDeviceInit();
  sensorsInterruptInit();
  sensorsTaskInit();
  isInit = true;
}
bool sensorsMpu9250Lps25hTest(void)
{
  bool testStatus = true;
  if (!isInit)
  {
    DEBUG_PRINT("Error while initializing sensor task\r\n");
    testStatus = false;
  }
  for (int i = 0; i < 300; i++)
  {
    if(mpu6500SelfTest() == true)
    {
      isMpu6500TestPassed = true;
      break;
    }
    else
    {
      vTaskDelay(M2T(10));
    }
  }
  testStatus &= isMpu6500TestPassed;
#ifdef SENSORS_ENABLE_MAG_AK8963
  testStatus &= isMagnetometerPresent;
  if (testStatus)
  {
    isAK8963TestPassed = ak8963SelfTest();
    testStatus = isAK8963TestPassed;
  }
#endif
  // Removed: barometer test - Barometer hardware not available
  return testStatus;
}
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
  static bool accBiasFound = false;
  static uint32_t accScaleSumCount = 0;
  if (!accBiasFound)
  {
    accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
    accScaleSumCount++;
    if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
    {
      accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
      accBiasFound = true;
    }
  }
  return accBiasFound;
}
#ifdef GYRO_BIAS_LIGHT_WEIGHT
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  static uint32_t gyroBiasSampleCount = 0;
  static bool gyroBiasNoBuffFound = false;
  static Axis3i64 gyroBiasSampleSum;
  static Axis3i64 gyroBiasSampleSumSquares;
  if (!gyroBiasNoBuffFound)
  {
    gyroBiasSampleSum.x += gx;
    gyroBiasSampleSum.y += gy;
    gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
    gyroBiasSampleSumSquares.x += gx * gx;
    gyroBiasSampleSumSquares.y += gy * gy;
    gyroBiasSampleSumSquares.z += gz * gz;
#endif
    gyroBiasSampleCount += 1;
    if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES)
    {
      gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
      gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
      gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
      gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
      gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
      gyroBiasNoBuffFound = true;
    }
  }
  return gyroBiasNoBuffFound;
}
#else
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
  sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);
  if (!gyroBiasRunning.isBiasValueFound)
  {
    sensorsFindBiasValue(&gyroBiasRunning);
    if (gyroBiasRunning.isBiasValueFound)
    {
      // DÜZELTME D2: Buzzer kullanılmıyor - soundSetEffect(SND_CALIB) kaldırıldı
      ledseqRun(&seq_calibrated);
    }
  }
  gyroBiasOut->x = gyroBiasRunning.bias.x;
  gyroBiasOut->y = gyroBiasRunning.bias.y;
  gyroBiasOut->z = gyroBiasRunning.bias.z;
  return gyroBiasRunning.isBiasValueFound;
}
#endif
static void sensorsBiasObjInit(BiasObj* bias)
{
  bias->isBufferFilled = false;
  bias->bufHead = bias->buffer;
}
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
{
  uint32_t i;
  int64_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};
  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }
  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);
  meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
static void __attribute__((used)) sensorsCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};
  for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }
  meanOut->x = sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
{
  bias->bufHead->x = x;
  bias->bufHead->y = y;
  bias->bufHead->z = z;
  bias->bufHead++;
  if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = true;
  }
}
static bool sensorsFindBiasValue(BiasObj* bias)
{
  static int32_t varianceSampleTime;
  bool foundBias = false;
  if (bias->isBufferFilled)
  {
    sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);
    if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount()))
    {
      varianceSampleTime = xTaskGetTickCount();
      bias->bias.x = bias->mean.x;
      bias->bias.y = bias->mean.y;
      bias->bias.z = bias->mean.z;
      foundBias = true;
      bias->isBiasValueFound = true;
    }
  }
  return foundBias;
}
bool sensorsMpu9250Lps25hManufacturingTest(void)
{
  bool testStatus = false;
  Axis3i16 g;
  Axis3i16 a;
  Axis3f acc;  
  float pitch, roll;
  uint32_t startTick = xTaskGetTickCount();
  testStatus = mpu6500SelfTest();
  if (testStatus)
  {
    sensorsBiasObjInit(&gyroBiasRunning);
    while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT)
    {
      mpu6500GetMotion6(&a.y, &a.x, &a.z, &g.y, &g.x, &g.z);
      if (processGyroBias(g.x, g.y, g.z, &gyroBias))
      {
        gyroBiasFound = true;
        DEBUG_PRINT("Gyro variance test [OK]\n");
        break;
      }
    }
    if (gyroBiasFound)
    {
      acc.x = -(a.x) * SENSORS_G_PER_LSB_CFG;
      acc.y =  (a.y) * SENSORS_G_PER_LSB_CFG;
      acc.z =  (a.z) * SENSORS_G_PER_LSB_CFG;
      pitch = tanf(-acc.x/(sqrtf(acc.y*acc.y + acc.z*acc.z))) * 180/(float) M_PI;
      roll = tanf(acc.y/acc.z) * 180/(float) M_PI;
      if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX))
      {
        DEBUG_PRINT("Acc level test [OK]\n");
        testStatus = true;
      }
      else
      {
        DEBUG_PRINT("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", (double)roll, (double)pitch);
        testStatus = false;
      }
    }
    else
    {
      DEBUG_PRINT("Gyro variance test [FAIL]\n");
      testStatus = false;
    }
  }
  return testStatus;
}
void __attribute__((used)) EXTI13_Callback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  imuIntTimestamp = usecTimestamp();
  xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
static void sensorsAccAlignToGravity(Axis3f* in, Axis3f* out)
{
  Axis3f rx;
  Axis3f ry;
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;
  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}
void sensorsMpu9250Lps25hSetAccMode(accModes accMode)
{
  switch (accMode)
  {
    case ACC_MODE_PROPTEST:
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, 500);
      }
      break;
    case ACC_MODE_FLIGHT:
    default:
      for (uint8_t i = 0; i < 3; i++)
      {
        lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
      }
      break;
  }
}
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
  for (uint8_t i = 0; i < 3; i++) {
    in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
  }
}
#ifdef GYRO_ADD_RAW_AND_VARIANCE_LOG_VALUES
LOG_GROUP_START(gyro)
LOG_ADD(LOG_INT16, xRaw, &gyroRaw.x)
LOG_ADD(LOG_INT16, yRaw, &gyroRaw.y)
LOG_ADD(LOG_INT16, zRaw, &gyroRaw.z)
LOG_ADD(LOG_FLOAT, xVariance, &gyroBiasRunning.variance.x)
LOG_ADD(LOG_FLOAT, yVariance, &gyroBiasRunning.variance.y)
LOG_ADD(LOG_FLOAT, zVariance, &gyroBiasRunning.variance.z)
LOG_GROUP_STOP(gyro)
#endif
PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, AK8963, &isMagnetometerPresent)  // Fixed: Changed from HMC5883L to AK8963
// Removed: LPS25H parameter - Barometer hardware not available
PARAM_GROUP_STOP(imu_sensors)
PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, MPU6500, &isMpu6500TestPassed)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, AK8963, &isAK8963TestPassed)  // Fixed: Changed from HMC5883L to AK8963
// Removed: LPS25H test parameter - Barometer hardware not available
PARAM_GROUP_STOP(imu_tests)