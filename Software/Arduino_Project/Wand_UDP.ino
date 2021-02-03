#include <WiFi.h>
#include <WiFiUdp.h> //引用以使用UDP
#include <Wire.h>
#include <string.h>
//const char *ssid = "Waxd"; //Connect to the hot point of my phone 
//const char *password = "22222222"; //Connect to the hot point of my phone 
const char *ssid = "TP-LINK_1059";  //Connect to the WIFI  *IP:192.168.1.101*
const char *password = "JzRzKa0419"; //Connect to the WIFI  *IP:192.168.1.101*


WiFiUDP Udp;                      //创建UDP对象
unsigned int localUdpPort = 2333; //本地端口号


#define SDA 32
#define SCL 33
#define Single_Measure    1
#define Multi_Measure     1200
const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;



void IMU_Measure(void)
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers

  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}



void setup()
{

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
  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());

  Udp.begin(localUdpPort); //启用UDP监听以接收数据
}
int oldtime,newtime,measure_times = 1;
void loop()
{
  int packetSize = Udp.parsePacket(); //获取当前队首数据包长度
  if (packetSize)                     //如果有数据可用
  {
    char buf[packetSize];
    Udp.read(buf, packetSize); //读取当前包数据
    
    Serial.println();
    Serial.print("Received: ");
    Serial.println(buf[0]);
    Serial.print("From IP: ");
    Serial.println(Udp.remoteIP());
    Serial.print("From Port: ");
    Serial.println(Udp.remotePort());
    oldtime = millis();
    if(buf[0] == 'C' || buf[0] == 'c')
      measure_times = Single_Measure;
    if(buf[0] == 'S' || buf[0] == 's')
      measure_times = Multi_Measure;
    /************************************************************************/
    /***************   当收到S("spell")时，连续测量1200次       ****************/
    /***************   当收到C("Control")时，测量1次           ****************/
    /************************************************************************/
    for(int i = 0;i<measure_times;i++){
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //准备发送数据
    IMU_Measure();
    Udp.print("AcX:");  Udp.print(AcX);  
    Udp.print(" AcY:"); Udp.print(AcY);
    Udp.print(" AcZ:"); Udp.print(AcZ);
    Udp.print(" GyX:"); Udp.print(GyX);
    Udp.print(" GyY:"); Udp.print(GyY);
    Udp.print(" GyZ:"); Udp.println(GyZ);
    Udp.endPacket();            //发送数据
    }
    /************************************************************************/
    newtime = millis();
    Serial.print("Measurement takes: ");
    Serial.print(newtime - oldtime);
    Serial.println("ms");
  }
}
