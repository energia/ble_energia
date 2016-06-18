
boolean a = 1;
int b = 12;
float c = 123;
char* d = "1234";

void setup()
{
  Serial.begin(115200);
  Serial.println(readValue_boolean(0));
  Serial.println(readValue_int(1));
  Serial.println(readValue_float(2));
  Serial.println(readValue_string(3));
}

void* data[] = { &a, &b, &c, &d};

boolean readValue_boolean(int handle) {
  return *(boolean*) data[handle];
}

int readValue_int(int handle) {
  return *(int*) data[handle];
}

float readValue_float(int handle) {
  return *(float*) data[handle];
}

char* readValue_string(int handle) {
  return *(char**) data[handle];
}

void loop()
{
}

