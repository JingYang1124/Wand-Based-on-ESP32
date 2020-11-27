#include <WiFi.h>
#include <Wire.h>
#include <WiFiMulti.h>
#include <string.h>

WiFiMulti WiFiMulti;

#define SDA 32
#define SCL 33
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t Old_AcX,Old_AcY,Old_AcZ,Old_Tmp,Old_GyX,Old_GyY,Old_GyZ;

const char* ssid = "TP-LINK_1059";
const char* password = "JzRzKa0419";

WiFiServer server(1024);//Server Port
int LED = 5;
void IMU_Measure(void)
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers

  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcX = AcX;
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcY = AcY;
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  AcZ = AcZ;
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyX = GyX;
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyY = GyY;
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  GyZ = GyZ;
}

void setup()
{ 
  pinMode(LED, OUTPUT);
  Wire.begin(SDA, SCL);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(100);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); // 
  Wire.write(0x10); // Full range is +-8g, and 4000 is 1g
  Wire.endTransmission(true);
  delay(100);
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); //
  Wire.write(0x10); // Full range is +-1000Degree/s, and 32.4 is 1Degree/s
  Wire.endTransmission(true);
  
  Serial.begin(115200);
   
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Wand Ready! Use '");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  server.begin();
}

int count = 0;
boolean   LedFlag = 0;
void loop()
{
  // Use WiFiClient class to create TCP connections
  WiFiClient client = server.available();
  if(client)
  {
    Serial.println("New Client.");
    while (client.connected())//Loop while the client's connected
    {
      if(client.available())
      {
        {
          Serial.println("Measure");
          IMU_Measure();
          Old_AcX = AcX;
          Old_AcY = AcY;
          Old_AcZ = AcZ;
          Old_GyX = GyX;
          Old_GyY = GyY;
          Old_GyZ = GyZ;
          while(1)
          {
            IMU_Measure();
            if(LedFlag == 0)
            {
              if(abs(Old_AcX - AcX)>=2000 or abs(Old_AcY - AcY)>=2000 or abs(Old_AcZ - AcZ)>=2000)
              {
                digitalWrite(LED, HIGH); 
                LedFlag = 1;
              }
              else
              {
                Old_AcX = AcX;
                Old_AcY = AcY;
                Old_AcZ = AcZ;
                Old_GyX = GyX;
                Old_GyY = GyY;
                Old_GyZ = GyZ;  
              }
            }

            client.print("AcX:");  client.print(AcX);  
            client.print(" AcY:"); client.print(AcY);
            client.print(" AcZ:"); client.print(AcZ);
            client.print(" GyX:"); client.print(GyX);
            client.print(" GyY:"); client.print(GyY);
            client.print(" GyZ:"); client.println(GyZ);

            delay(10);
            if(LedFlag == 1)
            {
              count ++;
              if(count >= 200)
              {
                count = 0;
                LedFlag = 0;
                digitalWrite(LED, LOW); 
                Old_AcX = AcX;
                Old_AcY = AcY;
                Old_AcZ = AcZ;
                Old_GyX = GyX;
                Old_GyY = GyY;
                Old_GyZ = GyZ; 
              }
            }

          }
        }
      }
    }
  }
  else
  {
    IMU_Measure();
    if(LedFlag == 0)
    {
      if(abs(Old_AcX - AcX)>=2000 or abs(Old_AcY - AcY)>=2000 or abs(Old_AcZ - AcZ)>=2000)
      {
        digitalWrite(LED, HIGH); 
        LedFlag = 1;
      }
      else
      {
        Old_AcX = AcX;
        Old_AcY = AcY;
        Old_AcZ = AcZ;
        Old_GyX = GyX;
        Old_GyY = GyY;
        Old_GyZ = GyZ;  
      }
    }
    else
    {
      count ++;
      if(count >= 400)
      {
        count = 0;
        LedFlag = 0;
        digitalWrite(LED, LOW); 
        Old_AcX = AcX;
        Old_AcY = AcY;
        Old_AcZ = AcZ;
        Old_GyX = GyX;
        Old_GyY = GyY;
        Old_GyZ = GyZ; 
      }
    }
    delay(10);
  }
}
