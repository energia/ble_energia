
#include <BLE.h>

char char1Value = 0;
int char2Value = 0;
long char3Value = 0;
const char *char4Value = "Hello, world!";

BLE_Char char1 =
{
  {0xF1, 0xFF},
  BLE_READABLE | BLE_WRITABLE,
  "Characteristic 1"
};

BLE_Char char2 =
{
  {0xF2, 0xFF},
  BLE_READABLE,
  "Characteristic 2"
};

BLE_Char char3 =
{
  {0xF3, 0xFF},
  BLE_WRITABLE,
  "Characteristic 3"
};

BLE_Char char4 =
{
  {0xF4, 0xFF},
  BLE_NOTIFIABLE,
  "Characteristic 4"
};

BLE_Char *simpleServiceChars[] = {&char1, &char2, &char3, &char4};

BLE_Service simpleService =
{
  {0xF0, 0xFF},
  4, simpleServiceChars
};

int t1;
int t2;
int t3;

void setup() {
  Serial.begin(115200);
  ble.begin();
  ble.addService(&simpleService);
  ble.writeValue(&char1, char1Value);
  ble.writeValue(&char2, char2Value);
  ble.writeValue(&char3, char3Value);
  ble.writeValue(&char4, char4Value);
  ble.serial();
  ble.setAdvertName("Energia BLE");
  ble.startAdvert();

  t1 = millis();
  t2 = t1;
  t3 = t1;
}

/* +1 for null-terminator */
char serialData[BLE_SERIAL_BUFFER_SIZE + 1];

void loop() {
  ble.handleEvents();

  /* Forward Energia serial monitor to BLE serial. */
  int numBytes = Serial.available();
  if (numBytes)
  {
    Serial.readBytes(serialData, numBytes);
    serialData[numBytes] = '\0';
    ble.print(serialData);
  }

  /* Forward BLE serial to Energia serial monitor. */
  numBytes = ble.available();
  if (numBytes)
  {
    ble.readBytes(serialData, numBytes);
    serialData[numBytes] = '\0';
    Serial.println(serialData);
  }

  /* Increment char2 every second. */
  if (millis() - t1 >= 1000)
  {
    char2Value++;
    ble.writeValue(&char2, char2Value);
    t1 = millis();
  }

  /* Increment char4 by 100 every 5 seconds. */
  if (millis() - t2 >= 5000)
  {
    char3Value += 100;
    ble.writeValue(&char3, char3Value);
    t2 = millis();
  }

  /* Print all characteristic values every second. */
  if (millis() - t3 >= 1000)
  {
    char1Value = ble.readValue_char(&char1);
    Serial.print("char1Value=");
    Serial.println((int) char1Value);

    char2Value = ble.readValue_int(&char2);
    Serial.print("char2Value=");
    Serial.println(char2Value);

    char3Value = ble.readValue_long(&char3);
    Serial.print("char3Value=");
    Serial.println(char3Value);

    char4Value = ble.readValue_charArr(&char4);
    Serial.print("char4Value=");
    Serial.println(char4Value);

    Serial.print("\n\n\n");
    t3 = millis();

  }
}