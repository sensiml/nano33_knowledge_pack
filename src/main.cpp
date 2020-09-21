#include <Arduino.h>
#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU

#define dbgprintlev(level, ...)      \
    if (level <= KB_LOG_LEVEL) {     \
        Serial.println(__VA_ARGS__); \
    }

#include <kb.h>
#include <recognition_config.h>

static unsigned long currentMs, previousMs;
static unsigned long interval = 0;
int num_sensors = ((ENABLE_ACCEL * 3) + (ENABLE_GYRO * 3) + (ENABLE_MAG * 3));
int ret = 0;

static int16_t sensorRawData[((ENABLE_ACCEL * 3) + (ENABLE_GYRO * 3) + (ENABLE_MAG * 3))];
static SENSOR_DATA_T* data = (SENSOR_DATA_T*)&sensorRawData[0];
static int sensorRawIndex = 0;

static uint8_t features[MAX_VECTOR_SIZE];
static uint8_t fv_length;

DynamicJsonDocument classification_result(1024);

static int get_acc_gyro_odr()
{
    switch (ACCEL_GYRO_DEFAULT_ODR)
    {
        case ACCEL_GYRO_ODR_OFF:
            return 0;
        case ACCEL_GYRO_ODR_10HZ:
            return 10;
        case ACCEL_GYRO_ODR_50HZ:
            return 50;
        case ACCEL_GYRO_ODR_119HZ:
            return 119;
        case ACCEL_GYRO_ODR_238HZ:
            return 238;
        case ACCEL_GYRO_ODR_476HZ:
            return 476;
    }
}

static void setup_imu()
{
    if (!IMU.begin()) //Initialize IMU sensor
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    //Set units.
    IMU.accelUnit = METERPERSECOND2;
    IMU.gyroUnit = DEGREEPERSECOND;
    IMU.magnetUnit = MICROTESLA;

#if ENABLE_ACCEL && (ENABLE_GYRO == 0)
    IMU.setAccelODR(ACCEL_GYRO_DEFAULT_ODR);
    IMU.setGyroODR(ACCEL_GYRO_ODR_OFF);

#elif (ENABLE_ACCEL && ENABLE_GYRO)
    IMU.setAccelODR(ACCEL_GYRO_DEFAULT_ODR);
    IMU.setGyroODR(ACCEL_GYRO_DEFAULT_ODR);

#else //gyro only
    IMU.setAccelODR(ACCEL_GYRO_ODR_OFF);
    IMU.setGyroODR(ACCEL_GYRO_DEFAULT_ODR);

#endif

#if ENABLE_MAG
    IMU.setMagnetODR(mag_speed);
#endif
    IMU.setContinuousMode();
}

static void update_imu()
{
    int16_t x, y, z;
    //Accelerometer values IMU.accelerationAvailable() &&
    if (ENABLE_ACCEL)
    {
        IMU.readRawAccelInt16(x, y, z);
        sensorRawData[sensorRawIndex++] = x;
        sensorRawData[sensorRawIndex++] = y;
        sensorRawData[sensorRawIndex++] = z;
    }

    //Gyroscope values IMU.gyroscopeAvailable() &&
    if (ENABLE_GYRO)
    {
        IMU.readRawGyroInt16(x, y, z);
        sensorRawData[sensorRawIndex++] = x;
        sensorRawData[sensorRawIndex++] = y;
        sensorRawData[sensorRawIndex++] = z;
    }

    //Magnetometer values IMU.magneticFieldAvailable() &&
    if (ENABLE_MAG)
    {
        IMU.readRawMagnetInt16(x, y, z);
        sensorRawData[sensorRawIndex++] = x;
        sensorRawData[sensorRawIndex++] = y;
        sensorRawData[sensorRawIndex++] = z;
    }
}

static void sml_output_results(uint16_t model, uint16_t classification)
{
    classification_result["ModelNumber"] = model;
    classification_result["Classification"] = classification;

    kb_get_feature_vector(model, features, &fv_length);
    classification_result["FeatureLength"] = fv_length;

    JsonArray fv = classification_result.createNestedArray("FeatureVector");
    for (int i = 0; i < fv_length; i++)
    {
        fv.add(String(features[i]));
    }

    serializeJson(classification_result, Serial);
    Serial.println("");
    Serial.flush();
    classification_result.clear();
}

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(1000);
    Serial.println("Setting up...");

    setup_imu();
    delay(1000);
    kb_model_init();
    delay(1000);
    interval = (1000 / (long)get_acc_gyro_odr());
    delay(1000);
    memset(sensorRawData, 0, num_sensors * sizeof(int16_t));
}

void loop()
{

    sensorRawIndex = 0;
    currentMs = millis();
    if (currentMs - previousMs >= interval)
    {
        update_imu();
        data = sensorRawData;
        ret = kb_run_model((SENSOR_DATA_T*)data, num_sensors, KB_MODEL_jp1_rank_0_INDEX);
        if (ret >= 0) {
            sml_output_results(KB_MODEL_jp1_rank_0_INDEX, ret);
            kb_reset_model(0);
        };

        previousMs = currentMs;
    }
}
