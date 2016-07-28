
#include <BLE.h>

int heartRateMeasurement = 0;

BLE_Char heartRateChar =
{
  {0x37, 0x2A}, // UUID for heart rate measurement
  BLE_READABLE, // The BLE client is able to only read this characterisitic value
  "Heart rate measurement" // Description shown to the client
};

/* This syntax lets services have any number of characteristics. */
BLE_Char *heartRateChars[] = {&heartRateChar};

BLE_Service heartRateService =
{
  {0x0D, 0x18}, // UUID for heart rate service
  1, heartRateChars
};

void setup() {
  Serial.begin(115200);
  ble.setLogLevel(BLE_LOG_ALL);

  /* Start the BLE layer and connect to the network processor. */
  ble.begin();

  /* Add the heart rate service. */
  ble.addService(&heartRateService);
  ble.writeValue(&heartRateChar, heartRateMeasurement);

  /* Set the name that dislays when scanning for bluetooth devices. */
  ble.setAdvertName("HeartRateService");

  /* Start advertising to other devices. */
  ble.startAdvert();
}

void loop() {
  /* Handle any messages received from the communication layer. */
  ble.handleEvents();

  /* Update the measurement every second. */
  if (millis() % 1000 == 0)
  {
    heartRateMeasurement += 1; // A little unrealistic

    /* Register the new value with the BLE layer. */
    ble.writeValue(&heartRateChar, heartRateMeasurement);

    /* This line isn't necessary for displaying the characteristic value,
       but it demonstrates how to read from the BLE layer. */
    heartRateMeasurement = ble.readValue_int(&heartRateChar);

    /* Print the measurement. */
    Serial.print("heartRateMeasurement=");Serial.println(heartRateMeasurement);
  }
}
