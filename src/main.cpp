/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include "Nicla_System.h"
#include "diabetic_ketone_tracker_inferencing.h"
#include "Arduino_BHY2.h" //Click here to get the library: http://librarymanager/All#Arduino_BHY2
#include <ArduinoBLE.h>
/** Struct to link sensor axis name to sensor value function */
typedef struct
{
  const char *name;
  float (*get_value)(void);

} eiSensors;

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f
#define BLE_SENSE_UUID(val) ("19b10000-" val "-537e-4f6c-d104768a1214")

/** Number sensor axes used */
#define NICLA_N_SENSORS 10

const int VERSION = 0x00000000;

BLEService service(BLE_SENSE_UUID("0000"));

BLEUnsignedIntCharacteristic versionCharacteristic(BLE_SENSE_UUID("1001"), BLERead);

BLEUnsignedIntCharacteristic patientStatusCharacteristic(BLE_SENSE_UUID("3101"), BLENotify);

BLECharacteristic accelerometerCharacteristic(BLE_SENSE_UUID("5001"), BLERead | BLENotify, 3 * sizeof(float)); // Array of 3x 2 Bytes, XY
BLECharacteristic gyroscopeCharacteristic(BLE_SENSE_UUID("6001"), BLERead | BLENotify, 3 * sizeof(float));     // Array of 3x 2 Bytes, XYZ

BLEFloatCharacteristic bsecCharacteristic(BLE_SENSE_UUID("9001"), BLERead);
BLEIntCharacteristic co2Characteristic(BLE_SENSE_UUID("9002"), BLERead);
BLEUnsignedIntCharacteristic gasCharacteristic(BLE_SENSE_UUID("9003"), BLERead);

/* Private variables ------------------------------------------------------- */
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

String name;

Sensor temperature(SENSOR_ID_TEMP);
Sensor humidity(SENSOR_ID_HUM);
Sensor pressure(SENSOR_ID_BARO);
Sensor gas(SENSOR_ID_GAS);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ accel(SENSOR_ID_ACC);
SensorQuaternion quaternion(SENSOR_ID_RV);
SensorBSEC bsec(SENSOR_ID_BSEC);

static bool ei_connect_fusion_list(const char *input_list);
static float get_accX(void) { return (accel.x() * 8.0 / 32768.0) * CONVERT_G_TO_MS2; }
static float get_accY(void) { return (accel.y() * 8.0 / 32768.0) * CONVERT_G_TO_MS2; }
static float get_accZ(void) { return (accel.z() * 8.0 / 32768.0) * CONVERT_G_TO_MS2; }
static float get_gyrX(void) { return (gyro.x() * 8.0 / 32768.0) * CONVERT_G_TO_MS2; }
static float get_gyrY(void) { return (gyro.y() * 8.0 / 32768.0) * CONVERT_G_TO_MS2; }
static float get_gyrZ(void) { return (gyro.z() * 8.0 / 32768.0) * CONVERT_G_TO_MS2; }
static float get_iaq(void) { return bsec.iaq() * 1.0; }
static float get_voc(void) { return bsec.b_voc_eq(); }
static float get_co2(void) { return bsec.co2_eq(); }
static float get_accuracy(void) { return bsec.accuracy(); }

static int8_t fusion_sensors[NICLA_N_SENSORS];
static int fusion_ix = 0;

/** Used sensors value function connected to label name */
eiSensors nicla_sensors[] =
    {
        "accel.x",
        &get_accX,
        "accel.y",
        &get_accY,
        "accel.z",
        &get_accZ,
        "gyro.x",
        &get_gyrX,
        "gyro.y",
        &get_gyrY,
        "gyro.z",
        &get_gyrZ,
        "iaq",
        &get_iaq,
        "voc",
        &get_voc,
        "co2",
        &get_co2,
        "accuracy",
        &get_accuracy,
};

void blePeripheralDisconnectHandler(BLEDevice central);

void onPressureCharacteristicRead(BLEDevice central, BLECharacteristic characteristic);
void onBsecCharacteristicRead(BLEDevice central, BLECharacteristic characteristic);
void onCo2CharacteristicRead(BLEDevice central, BLECharacteristic characteristic);
void onGasCharacteristicRead(BLEDevice central, BLECharacteristic characteristic);

/**
 * @brief      Arduino setup function
 */
void setup()
{
  /* Init serial */
  Serial.begin(115200);
  // comment out the below line to cancel the wait for USB connection (needed for native USB)
  nicla::begin();
  nicla::leds.begin();
  nicla::leds.setColor(green);

  Serial.println("Edge Impulse Sensor Fusion Inference\r\n");

  /* Connect used sensors */
  if (ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false)
  {
    ei_printf("ERR: Errors in sensor list detected\r\n");
    return;
  }

  /* Init & start sensors */
  BHY2.begin(NICLA_STANDALONE);
  temperature.begin();
  humidity.begin();
  pressure.begin();
  gyro.begin();
  accel.begin();
  quaternion.begin();
  bsec.begin();
  gas.begin();

  if (!BLE.begin())
  {
    Serial.println("Failled to initialized BLE!");

    while (1)
      ;
  }

  String address = BLE.address();

  Serial.print("address = ");
  Serial.println(address);

  address.toUpperCase();

  name = "NiclaSenseME-";
  name += address[address.length() - 5];
  name += address[address.length() - 4];
  name += address[address.length() - 2];
  name += address[address.length() - 1];

  Serial.print("name = ");
  Serial.println(name);

  BLE.setLocalName(name.c_str());
  BLE.setDeviceName(name.c_str());
  BLE.setAdvertisedService(service);

  // Add all the previously defined Characteristics

  service.addCharacteristic(patientStatusCharacteristic);

  service.addCharacteristic(versionCharacteristic);
  service.addCharacteristic(accelerometerCharacteristic);
  service.addCharacteristic(gyroscopeCharacteristic);

  service.addCharacteristic(bsecCharacteristic);
  service.addCharacteristic(co2Characteristic);
  service.addCharacteristic(gasCharacteristic);

  // Disconnect event handler
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // Sensors event handlers

  bsecCharacteristic.setEventHandler(BLERead, onBsecCharacteristicRead);
  co2Characteristic.setEventHandler(BLERead, onCo2CharacteristicRead);
  gasCharacteristic.setEventHandler(BLERead, onGasCharacteristicRead);

  versionCharacteristic.setValue(VERSION);

  BLE.addService(service);
  BLE.advertise();
}

/**
 * @brief      Get data and run inferencing
 */
void loop()
{

  while (BLE.connected())
  {
    BHY2.update();

    if (gyroscopeCharacteristic.subscribed())
    {
      float x, y, z;

      x = gyro.x();
      y = gyro.y();
      z = gyro.z();

      float gyroscopeValues[3] = {x, y, z};

      gyroscopeCharacteristic.writeValue(gyroscopeValues, sizeof(gyroscopeValues));
    }

    if (accelerometerCharacteristic.subscribed())
    {
      float x, y, z;
      x = accel.x();
      y = accel.y();
      z = accel.z();

      float accelerometerValues[] = {x, y, z};
      accelerometerCharacteristic.writeValue(accelerometerValues, sizeof(accelerometerValues));
    }

    if (patientStatusCharacteristic.subscribed())
    {
      // Allocate a buffer here for the values we'll read from the IMU
      float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

      for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)
      {
        // Determine the next tick (and then sleep later)
        

        // Update function should be continuously polled
        BHY2.update();

        for (int i = 0; i < fusion_ix; i++)
        {
          buffer[ix + i] = nicla_sensors[fusion_sensors[i]].get_value();
        }

        
      }

      // Turn the raw buffer in a signal which we can the classify
      signal_t signal;
      int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
      if (err != 0)
      {
        ei_printf("ERR:(%d)\r\n", err);
        return;
      }

      // Run the classifier
      ei_impulse_result_t result = {0};

      err = run_classifier(&signal, &result, debug_nn);
      if (err != EI_IMPULSE_OK)
      {
        ei_printf("ERR:(%d)\r\n", err);
        return;
      }
      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
      {

        if (result.classification[ix].value > 0.9)
        {
          patientStatusCharacteristic.writeValue(ix);
        }
        ei_printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);

      }
    }

    // print the predictions
    /*ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\r\n",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
    }*/
  }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\r\n", result.anomaly);
#endif
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif

/**
 * @brief Go through nicla sensor list to find matching axis name
 *
 * @param axis_name
 * @return int8_t index in nicla sensor list, -1 if axis name is not found
 */
static int8_t ei_find_axis(char *axis_name)
{
  int ix;
  for (ix = 0; ix < NICLA_N_SENSORS; ix++)
  {
    if (strstr(axis_name, nicla_sensors[ix].name))
    {
      return ix;
    }
  }
  return -1;
}

/**
 * @brief Check if requested input list is valid sensor fusion, create sensor buffer
 *
 * @param[in]  input_list      Axes list to sample (ie. "accX + gyrY + magZ")
 * @retval  false if invalid sensor_list
 */
static bool ei_connect_fusion_list(const char *input_list)
{
  char *buff;
  bool is_fusion = false;

  /* Copy const string in heap mem */
  char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
  if (input_string == NULL)
  {
    return false;
  }
  memset(input_string, 0, strlen(input_list) + 1);
  strncpy(input_string, input_list, strlen(input_list));

  /* Clear fusion sensor list */
  memset(fusion_sensors, 0, NICLA_N_SENSORS);
  fusion_ix = 0;

  buff = strtok(input_string, "+");

  while (buff != NULL)
  { /* Run through buffer */
    int8_t found_axis = 0;

    is_fusion = false;
    found_axis = ei_find_axis(buff);

    if (found_axis >= 0)
    {
      if (fusion_ix < NICLA_N_SENSORS)
      {
        fusion_sensors[fusion_ix++] = found_axis;
      }
      is_fusion = true;
    }

    buff = strtok(NULL, "+ ");
  }

  ei_free(input_string);

  return is_fusion;
}

void blePeripheralDisconnectHandler(BLEDevice central)
{
  nicla::leds.setColor(red);
}

void onBsecCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  float airQuality = float(bsec.iaq());
  bsecCharacteristic.writeValue(airQuality);
}

void onCo2CharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  uint32_t co2 = bsec.co2_eq();
  co2Characteristic.writeValue(co2);
}

void onGasCharacteristicRead(BLEDevice central, BLECharacteristic characteristic)
{
  unsigned int g = gas.value();
  gasCharacteristic.writeValue(g);
}
