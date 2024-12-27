#include <SDISerial.h>
#include <Wire.h>

#define DATA_PIN 2
#define SLAVE_ADDRESS 0x33 

String receivedDataMeasurence = "";
String receivedDataModel = "";
String all = "";
String model = "";
String data1 = "";
String data2 = "";

uint8_t sendData[32];

SDISerial sdi(DATA_PIN);

void setup() {
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDRESS);
  sdi.begin();
  Wire.onRequest(RequestEvent); 
}

void loop() {
  receivedDataMeasurence = measure();
  // receivedDataModel = info();

}

void RequestEvent()
{

  all = "";

  // all = receivedDataModel.substring(11, 15) + receivedDataMeasurence.substring(1, 20);
  all = receivedDataMeasurence.substring(1, 20);

  Serial.println(all);

  Wire.write(all.c_str(), all.length());

}

String measure() {
  sdi.sdi_cmd("0M!");
  delay(1200);  // делей для sdi-12 12 мс можно попробовать уменьшить 1200 
  String data1 = sdi.sdi_query("0D0!", 2500); // делей для sdi-12 12 мс можно попробовать уменьшить 1200 

  if (data1 == NULL || data1.length() == 0) {
    data1 = "No data"; // Handle empty or null responses
  }
  return data1; 
}

String info() {
  // delay(1500);
  String data2 = sdi.sdi_query("0I!", 120); 

  if (data2 == NULL || data2.length() == 0 ) {
    data2 = "No model data"; // Handle empty or null responses
  }
  return data2;
}
