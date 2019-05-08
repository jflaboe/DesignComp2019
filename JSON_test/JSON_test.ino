#include <ArduinoJson.h>

const int capacity = JSON_ARRAY_SIZE(2) + JSON_ARRAY_SIZE(6) + JSON_OBJECT_SIZE(4);
string json_out = "";

void setup() {
  
}

void loop() {
  StaticJsonDocument<capacity> doc;
  doc["key1"] = "val1";
  doc["key2"] = "val2";
  serializeJson(doc, json_out);
  SerialBT.println(json_out);
  json_out = "";
  delay(500);
}
