#include <Arduino.h>
#include <ArduinoJson.h>
#include <Arduino_LSM9DS1.h>  //Include the library for 9-axis IMU

#include <kb.h>
#include <sensor_config.h>
#if USE_BLE
#include <ArduinoBLE.h>
const char* nameOfPeripheral            = "Nano33 SensiML KP";
const char* RecognitionServiceUuid      = "42421100-5A22-46DD-90F7-7AF26F723159";
const char* RecognitionClassOnlyUuid    = "42421101-5A22-46DD-90F7-7AF26F723159";
const char* RecognitionClassFeatureUuid = "42421102-5A22-46DD-90F7-7AF26F723159";
const int   WRITE_BUFFER_SIZE           = 128;
bool        WRITE_BUFFER_FIXED_LENGTH   = false;

BLEService        recognitionService = BLEService(RecognitionServiceUuid);
BLECharacteristic classOnlyChar
    = BLECharacteristic(RecognitionClassOnlyUuid, BLERead | BLENotify, 4, true);
BLECharacteristic classFeaturesChar = BLECharacteristic(
    RecognitionClassFeatureUuid, BLERead | BLENotify, WRITE_BUFFER_SIZE, WRITE_BUFFER_FIXED_LENGTH);
BLEDevice central;


static void connectedLight()
{
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, LOW);
}

static void disconnectedLight()
{
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
}

static void onBLEConnected(BLEDevice central)
{
    Serial.print("Connected event, central: ");
    Serial.println(central.address());
    connectedLight();
}
static void onBLEDisconnected(BLEDevice central)
{
    Serial.print("Disconnected event, central: ");
    Serial.println(central.address());
    disconnectedLight();
    BLE.setConnectable(true);
}

void PrintInfo()
{
    Serial.println("Peripheral advertising info: ");
    Serial.print("Name: ");
    Serial.println(nameOfPeripheral);
    Serial.print("MAC: ");
    Serial.println(BLE.address());
    Serial.print("Service UUID: ");
    Serial.println(recognitionService.uuid());
    Serial.print("classOnlyChar UUID: ");
    Serial.println(classOnlyChar.uuid());
    Serial.print("classFeaturesChar UUID: ");
    Serial.println(classFeaturesChar.uuid());
}

void setup_ble()
{
    if (!BLE.begin())
    {
        Serial.println("starting BLE failed!");
        while (1)
            ;
    }
    // Serial.println("BLE Started!");
    BLE.setLocalName(nameOfPeripheral);
    BLE.noDebug();


    recognitionService.addCharacteristic(classOnlyChar);
    recognitionService.addCharacteristic(classFeaturesChar);
    delay(1000);
    BLE.addService(recognitionService);
    BLE.setAdvertisedService(recognitionService);
    // Bluetooth LE connection handlers.
    BLE.setEventHandler(BLEConnected, onBLEConnected);
    BLE.setEventHandler(BLEDisconnected, onBLEDisconnected);

    Serial.println("BLE Init done!");
    PrintInfo();
}

void Send_Notification(uint16_t model_no,
                       uint16_t classification,
                       uint8_t* features,
                       uint16_t num_features)
{

    kp_output_t    base_output;
    kp_output_fv_t output_with_features;
    base_output.model          = model_no;
    base_output.classification = classification;

    output_with_features.model_out = base_output;
    if (features != NULL && classFeaturesChar.subscribed() && num_features > 0)
    {
        output_with_features.fv_len = num_features;
        memcpy(output_with_features.features, features, num_features);
        classFeaturesChar.writeValue((void*) &output_with_features, sizeof(kp_output_fv_t));
        Serial.println("Sending With Classification with Features");
    }
    if (classOnlyChar.subscribed())
    {
        Serial.println("Sending Classification");
        classOnlyChar.writeValue((void*) &base_output, sizeof(kp_output_t));
    }
}
#endif  // USE_BLE

static unsigned long currentMs, previousMs;
static unsigned long interval    = 0;
int                  num_sensors = ((ENABLE_ACCEL * 3) + (ENABLE_GYRO * 3) + (ENABLE_MAG * 3));
int                  ret         = 0;

static int16_t        sensorRawData[((ENABLE_ACCEL * 3) + (ENABLE_GYRO * 3) + (ENABLE_MAG * 3))];
static SENSOR_DATA_T* data           = (SENSOR_DATA_T*) &sensorRawData[0];
static int            sensorRawIndex = 0;

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
    if (!IMU.begin())  // Initialize IMU sensor
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    // Set units.
    IMU.accelUnit  = METERPERSECOND2;
    IMU.gyroUnit   = DEGREEPERSECOND;
    IMU.magnetUnit = MICROTESLA;

#if ENABLE_ACCEL && (ENABLE_GYRO == 0)
    IMU.setAccelODR(ACCEL_GYRO_DEFAULT_ODR);
    IMU.setGyroODR(ACCEL_GYRO_ODR_OFF);

#elif (ENABLE_ACCEL && ENABLE_GYRO)
    IMU.setAccelODR(ACCEL_GYRO_DEFAULT_ODR);
    IMU.setGyroODR(ACCEL_GYRO_DEFAULT_ODR);

#else  // gyro only
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
    // Accelerometer values IMU.accelerationAvailable() &&
    if (ENABLE_ACCEL)
    {
        IMU.readRawAccelInt16(x, y, z);
        sensorRawData[sensorRawIndex++] = x;
        sensorRawData[sensorRawIndex++] = y;
        sensorRawData[sensorRawIndex++] = z;
    }

    // Gyroscope values IMU.gyroscopeAvailable() &&
    if (ENABLE_GYRO)
    {
        IMU.readRawGyroInt16(x, y, z);
        sensorRawData[sensorRawIndex++] = x;
        sensorRawData[sensorRawIndex++] = y;
        sensorRawData[sensorRawIndex++] = z;
    }

    // Magnetometer values IMU.magneticFieldAvailable() &&
    if (ENABLE_MAG)
    {
        IMU.readRawMagnetInt16(x, y, z);
        sensorRawData[sensorRawIndex++] = x;
        sensorRawData[sensorRawIndex++] = y;
        sensorRawData[sensorRawIndex++] = z;
    }
}

void sml_output_results(uint16_t model, uint16_t classification)
{
    classification_result["ModelNumber"]    = model;
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
#if USE_BLE
if(central.connected())
    Send_Notification(model, classification, features, fv_length);

#endif  // USE_BLE
}

#if ENABLE_AUDIO
#include <PDM.h>
static void  onPDMdata();
volatile int samplesRead;
short        sampleBuffer[2048];

int setup_audio()
{
    PDM.onReceive(onPDMdata);
    if (!PDM.begin(1, 16000))
    {
        Serial.println("Failed to start PDM!");
        while (1)
            ;
    }

    return 0;
}

uint8_t* getSampleBuffer() { return (uint8_t*) sampleBuffer; }

static void onPDMdata()
{
    // query the number of bytes available
    int bytesAvailable = PDM.available();

    // read into the sample buffer
    PDM.read(sampleBuffer, bytesAvailable);

    // 16-bit, 2 bytes per sample
    samplesRead = bytesAvailable / 2;
}

#endif

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    delay(2000);
    Serial.println("Setting up...");
    kb_model_init();
    delay(1000);

#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
    setup_imu();
    interval = (1000 / (long) get_acc_gyro_odr());
#endif
#if ENABLE_AUDIO
    setup_audio();
    interval = 16;
    delay(1000);
#endif

#if USE_BLE
    // ble_reporter = SensiML_KP_BLE();
    Serial.println("Start ble setup");
    setup_ble();
    delay(1000);
    BLE.advertise();
    disconnectedLight();
    Serial.println("Advertising...");

#endif

    memset(sensorRawData, 0, num_sensors * sizeof(int16_t));
}

void loop()
{
#if USE_BLE
    central = BLE.central();
    if (central.connected())
    {
        connectedLight();
    }
#endif  // USE_BLE
    sensorRawIndex = 0;
    currentMs      = millis();
    if (currentMs - previousMs >= interval)
    {
#if ENABLE_ACCEL || ENABLE_GYRO || ENABLE_MAG
        update_imu();
        data = sensorRawData;
        sml_recognition_run(data, num_sensors);
#endif

#if ENABLE_AUDIO
    num_sensors = 1;
        if (samplesRead)
        {
            for (int i = 0; i < samplesRead; i++)
            {
                data = &sampleBuffer[i];

                sml_recognition_run(data, num_sensors);
            }
            samplesRead = 0;
        }
#endif
        previousMs = currentMs;
    }
}
