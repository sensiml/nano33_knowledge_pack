#include <Arduino.h>
#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h> //Include the library for 9-axis IMU
#include <kb.h>
#include <sml_recognition_run.h>
#include <recognition_config.h>


static unsigned long currentMs, previousMs;
static long interval = 0;

static int16_t sensorRawData[(ENABLE_ACCEL * 3) + (ENABLE_GYRO * 3) + (ENABLE_MAG * 3)];
static int sensorRawIndex = 0;
static SensiML sml;
DynamicJsonDocument classification_result(1024);

static int get_acc_gyro_odr()
{
    switch (ACCEL_GYRO_DEFAULT_ODR) {
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
    //Accelerometer values IMU.accelerationAvailable() &&
    if (ENABLE_ACCEL) {
        IMU.readRawAccelInt16(sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++]);
    }

    //Gyroscope values IMU.gyroscopeAvailable() &&
    if (ENABLE_GYRO) {
        IMU.readRawGyroInt16(sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++]);
    }

    //Magnetometer values IMU.magneticFieldAvailable() &&
    if (ENABLE_MAG) {
        IMU.readRawMagnetInt16(sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++], sensorRawData[sensorRawIndex++]);
    }
}

static void sendKnowledgePackOutput(int classification, int model)
{

uint8_t features[MAX_VECTOR_SIZE];

uint8_t fv_length;
classification_result["ModelNumber"] = model;
classification_result["Classification"] = classification;
//kb_get_feature_vector(model,  features, &fv_length);
sml.GetFeatures(model, &fv_length, (uint8_t*)features);
int charIndex = 0;
classification_result["FeatureLength"] = fv_length;
// featureString[charIndex++] = '[';

JsonArray fv = classification_result.createNestedArray("FeatureVector");
for(int i=0; i < fv_length; i++)
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
    Serial.println("Setting up...");
    // put your setup code here, to run once:

    setup_imu();
    delay(1000);
    //kb_model_init();
    sml = SensiML();
    interval = (1000 / (long)get_acc_gyro_odr());
    delay(1000);
}

void loop()
{
    //We start with the initial model we want to run.
    //If there is a heirarchy of models, the final model number will be placed here.
    sensorRawIndex = 0;
    int final_model_number = KB_MODEL_jp1_rank_0_INDEX;
    int classification = -1;
    currentMs = millis();
    // put your main code here, to run repeatedly:
    if (currentMs - previousMs >= interval)
    {
        update_imu();
        classification = sml.RunModel(sensorRawData, &final_model_number);
        if(classification >= 0)
        {
            sendKnowledgePackOutput(classification, final_model_number);

            sml.ResetModel(KB_MODEL_jp1_rank_0_INDEX);
        }
        previousMs = currentMs;
    }
}
