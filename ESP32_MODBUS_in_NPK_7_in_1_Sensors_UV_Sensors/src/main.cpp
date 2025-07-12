#include <Arduino.h>
#include <HardwareSerial.h>

#define RX_PIN    16
#define TX_PIN    17
#define REDE_PIN  2

HardwareSerial mySerial(2);

/*----------------------------------NPK-Sensor-7-in-1----------------------------*/

byte NPK_request[] = {0x01,0x03,0x00,0x00,0x00,0x07};  //Modbus frame ,0x04,0x08
byte request_frame[8];
const int NPK_response_Length = 19;
byte NPK_received[NPK_response_Length];  // Number of expected bytes in the response from sensor

float moisture = 0.0;
float temperature = 0.0;
int conductivity = 0;
float phvalue = 0.0;
int nitrogen = 0;   // N
int phosphorus = 0; // P
int potassium = 0;  // K

/*----------------------------------RS485-UV-Sensors----------------------------*/
byte UV_request[] = {0x02,0x03,0x00,0x00,0x00,0x01};
const int UV_response_Length = 7;
byte UV_received[UV_response_Length];  // Number of expected bytes in the response from sensor
float uv_intensity = 0.0; 


// Modbus CRC16 function 
uint16_t calculateModbusCRC(const byte *data, uint8_t length)
{
  uint16_t modbusCRC  = 0xFFFF;
  for(int i = 0; i < length; i++)
  {
    modbusCRC  ^= data[i];
    for(int j = 0; j < 8; j++)
    {
      if(modbusCRC  & 1)
      {
        modbusCRC  = (modbusCRC  >> 1) ^ 0xA001;
      }
      else
      {
        modbusCRC  >>= 1;
      }
    }
  }
  return modbusCRC;
}

// Convert two bytes to a 16-bit integer FF, 9B = 0xFF9B
uint16_t combineSignedBytes(byte highByte, byte lowByte)
{
  return (highByte << 8 ) | lowByte;
}

void CreateFrame(const byte *request, uint8_t length)
{
  // Serial.println("Recieved : ");
  // for (int i = 0; i < length; i++) {
  //   if (request[i] < 0x10) Serial.print("0");
  //   Serial.print(request[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  uint16_t crc = calculateModbusCRC(request, length);
  memcpy(request_frame,request,length);
  request_frame[6] = crc & 0xFF;        // CRC LOW
  request_frame[7] = (crc >> 8) & 0xFF; // CRC HIGH

   // Debug: Print sent bytes
  Serial.print("Sent: ");
  for (int i = 0; i < 8; i++) {
    if (request_frame[i] < 0x10) Serial.print("0");
    Serial.print(request_frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void SendModbusRequest()
{
  digitalWrite(REDE_PIN,HIGH); // Enable Transmit Driver
  delay(2);
 
  mySerial.write(request_frame, 8);
  mySerial.flush();
  delay(2);
  digitalWrite(REDE_PIN,LOW);  // Enable Receive Driver
}

void ReadModbusResponse(byte *received, int length)
{

  // Read Response 
  unsigned long startTime = millis();
  int index = 0;
  while((millis() - startTime < 1000) && index < length)
  {
    if(mySerial.available())
    {
      received[index++] = mySerial.read();
    }
  }

  Serial.print("Received : ");
  for(int i = 0; i < length; i++)
  {
    if(received[i] < 0x10) Serial.print("0");
    Serial.print(received[i], HEX);
    Serial.print(" ");
  }

  Serial.println();

  // Validate Header 

  if(received[0] == 0x01 && received[1] == 0x03 && received[2] == 0x0E)
  {
    moisture     = (int16_t)combineSignedBytes(received[3], received[4]) * 0.1;
    temperature  = (int16_t)combineSignedBytes(received[5], received[6]) * 0.1;
    conductivity = (int16_t)combineSignedBytes(received[7], received[8]);
    phvalue      = (int16_t)combineSignedBytes(received[9], received[10]) * 0.1;
    nitrogen     = (int16_t)combineSignedBytes(received[11], received[12]);
    phosphorus   = (int16_t)combineSignedBytes(received[13], received[14]);
    potassium    = (int16_t)combineSignedBytes(received[15], received[16]);

    Serial.println("-------------------------Soil-Parameters-------------------------");
    Serial.printf("Moisture     : %.2f %\n"   , moisture);
    Serial.printf("Temperature  : %.2f C\n"   ,temperature);
    Serial.printf("Conductivity : %d us/cm\n" ,conductivity);
    Serial.printf("PH Value     : %.2f \n"    ,phvalue);
    Serial.printf("Nitrogen     : %d mg/kg\n" ,nitrogen);
    Serial.printf("Phosphorus   : %d mg/kg\n" ,phosphorus);
    Serial.printf("Potassium    : %d mg/kg\n" ,potassium);
    Serial.printf("UV Intensity : %.2f %\n"   , moisture);
    Serial.println("----------------------------------------------------------------");

  }
  else  if(received[0] == 0x02 && received[1] == 0x03 && received[2] == 0x02)
  {
    uv_intensity = (int16_t)combineSignedBytes(received[3],received[4]);
    uv_intensity *= 0.01;
    Serial.println("--------------------------UV-Parameters-------------------------");
    Serial.printf("UV Intensity  : %.2f mW/cm2\n"   , uv_intensity);
    Serial.println("----------------------------------------------------------------");
  }

}

void setup()
{
  Serial.begin(115200);
  mySerial.begin(4800,SERIAL_8N1,RX_PIN,TX_PIN);
  pinMode(REDE_PIN,OUTPUT);
  digitalWrite(REDE_PIN,LOW);  // Start in receive mode
}

void loop()
{
  /*-------------NPK-Sensor-Request--&--Response-------------*/
  CreateFrame(NPK_request, sizeof(NPK_request));
  SendModbusRequest();
  ReadModbusResponse(NPK_received , NPK_response_Length);

  /*-------------UV-Sensor-Request--&--Response--------------*/
  CreateFrame(UV_request, sizeof(UV_request));
  SendModbusRequest();
  ReadModbusResponse(UV_received, UV_response_Length);

  delay(2000);

}