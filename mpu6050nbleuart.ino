#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
// ##### Ble Ini
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string>

BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharacteristic2;
BLECharacteristic *pCharacteristic3;
BLECharacteristic *pCharacteristic4;

bool deviceConnected = false;
uint8_t light = 0;
int dhp11 = 0;
byte temperature = 0;
byte humidity = 0;
uint8_t temp = 0;
uint8_t humid= 0;
String led_state;
uint8_t cc =64;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "1a8f6007-a27e-4355-8557-db16a7c92fe0"
#define CHARACTERISTIC_UUID "e805c592-b0a7-405e-ac83-e2bf56efac07"

#define SERVICE2_UUID        "63a21f41-5c56-4677-b276-6d84e42b8fd7"
#define CHARACTERISTIC2_UUID "6ecc9e4c-9848-4e30-a1e1-c5cf99fba9a9"

#define CHARACTERISTIC3_UUID "e379c733-1a49-48c7-ac61-cb27976e4c09"
#define CHARACTERISTIC4_UUID "e379c733-1a49-48c7-ac62-cb27976e4c09"


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};



// ble
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define PIN_SDA 21 // GPIO_NUM_21 = 21,   /*!< GPIO21, input and output */
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050
#define portTICK_PERIOD_MS 1

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1 0x6B

/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x50 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */

static char tag[] = "mpu6050";

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                         \
	do                                             \
	{                                              \
		esp_err_t rc = (x);                        \
		if (rc != ESP_OK)                          \
		{                                          \
			ESP_LOGE("err", "esp_err_t = %d", rc); \
			assert(0 && #x);                       \
		}                                          \
	} while (0);
i2c_config_t conf;
i2c_cmd_handle_t cmd;
uint8_t data[14];

uint8_t accel_x;
uint8_t accel_y;
uint8_t accel_z;


void setup()
{
	Serial.begin(115200);


	ESP_LOGD(tag, ">> mpu6050");

	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = GPIO_NUM_21;
	conf.scl_io_num = GPIO_NUM_22;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	vTaskDelay(200 / portTICK_PERIOD_MS);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

// Create the BLE Device
  BLEDevice::init("VRTREN");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_INDICATE|
                      BLECharacteristic::PROPERTY_NOTIFY 
                    );
 
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  
  pCharacteristic->setValue(&cc, 1);
  
  BLEService *pService2 = pServer->createService(SERVICE2_UUID);
  pCharacteristic2 = pService2->createCharacteristic(
                    CHARACTERISTIC2_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_INDICATE
                  );
  pCharacteristic2->addDescriptor(new BLE2902());

  pCharacteristic3 = pService2->createCharacteristic(
                    CHARACTERISTIC3_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_INDICATE
                  );
  pCharacteristic3->addDescriptor(new BLE2902());

  pCharacteristic4 = pService2->createCharacteristic(
                    CHARACTERISTIC4_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY |
                    BLECharacteristic::PROPERTY_INDICATE
                  );
  pCharacteristic4->addDescriptor(new BLE2902());
  pService2->start();

  
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

}
void loop()
{
if (deviceConnected) {
 
   
// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + 1, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + 2, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + 3, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + 4, I2C_MASTER_ACK));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data + 5, I2C_MASTER_NACK));

		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		accel_x = (data[0] << 8) | data[1];
		accel_y = (data[2] << 8) | data[3];
		accel_z = (data[4] << 8) | data[5];
		ESP_LOGD(tag, "accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);
		Serial.printf("accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);
		Serial.println();
		
		


    
    pCharacteristic2->setValue(&accel_x, 1);
    pCharacteristic2->notify();
    
    pCharacteristic3->setValue(&accel_y, 1);
    pCharacteristic3->notify();

    pCharacteristic4->setValue(&accel_z, 1);
    pCharacteristic4->notify();
	 delay(10);
  }
  delay(80);
	
	
}
