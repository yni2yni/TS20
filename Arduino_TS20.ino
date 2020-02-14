// ADS Touch Sensor Test Example Program (IC P/N:TS20, 28QFN)
// Code: 
// Date: 2020.02.14  Ver.: 0.0.2
// H/W Target: ARDUINO UNO R3, S/W: Arduino IDE  1.8.11
// Author: Park, Byoungbae (yni2yni@hanmail.net)
// Note: More information? Please ,send to e-mail.
// Uno R3, A4:SDA, A5: SCL, Leonardo 2:SDA,3:SCL

#include <Wire.h>

#define LF				0x0A //New Line
#define CR				0x0D //Carriage  return
#define SPC				0x20 //Spcae

#define Sensitivity_PWM1 	0x00 //ch2,ch1
#define Sensitivity_PWM2 	0x01 //ch4,ch3
#define Sensitivity_PWM3 	0x02 //ch6,ch5
#define Sensitivity_PWM4	0x03 //ch21,ch7
#define Sensitivity_PWM5 	0x04 //ch9,ch8
#define Sensitivity_PWM6 	0x05 //ch11,ch10
#define Sensitivity_PWM7 	0x06 //ch13,ch12
#define Sensitivity_PWM8 	0x07 //ch15,ch14
#define Sensitivity_PWM9 	0x08 //ch17,ch16
#define Sensitivity_PWM10	0x09 //ch19,ch18
#define Sensitivity_PWM11 	0x0A //ch20

#define GTRL1			0x0B  
#define GTRL2			0x0C

#define Cal_CTRL		0x0D

#define Port_CTRL1		0x0E
#define Port_CTRL2		0x0F
#define Port_CTRL3		0x10
#define Port_CTRL4		0x11
#define Port_CTRL5		0x12
#define Port_CTRL6		0x13

#define Cal_HOLD1		0x14
#define Cal_HOLD2		0x15
#define Cal_HOLD3		0x16

#define Err_CTRL		0x17

#define Output1			0x20
#define Output2			0x21
#define Output3			0x22

#define Ref_wr_H		0x23
#define Ref_wr_L		0x24
#define Ref_wr_CH1		0x25
#define Ref_wr_CH2		0x26
#define Ref_wr_CH3		0x27

#define Sen_RD_CTRL		0x28
#define Sensitivity_RD	0x29

#define Rd_CH1			0x30
#define Rd_CH2			0x31
#define Rd_CH3			0x32
#define Sen_H			0x33
#define Sen_L			0x34
#define Ref_H			0x35
#define Ref_L			0x36
#define Rd_CH1			0x37
#define Rd_CH2			0x38
#define Rd_CH3			0x39

#define TS20_SLAVE_GND  0x6A //0xD4>>1 //ADD Pin = GND (7bit address)
#define TS20_SLAVE_VDD 0x7A  //0xF4>>1 //ADD Pin = VDD (7bit address)

// #define TS20_SLAVE_GND 0xD4 // ADD Pin = GND (7bit address+1bit W/R)
// #define TS20_SLAVE_VDD 0xF4 // ADD Pin = VDD (7bit address+1bit W/R)

void  Init_TS20(void); //Initialize TS20

#define RESET_PIN 7 //Reset pin
#define EN_PIN    6 //I2C Enable Pin

void Register_Dump()
{
   byte read_data[1] = {0};

   for (int i = 0; i < 256; i += 16)
   {
      for (int j = 0; j <= 15; j++)
      {
         Wire.beginTransmission(TS20_SLAVE_GND); // sned ic slave address
         Wire.write((i + j));                 // sends register address
         Wire.endTransmission();              // stop transmitting
         Wire.requestFrom(TS20_SLAVE_GND, 1); // key data read (2 byte)
         read_data[0] = Wire.read();          //Key 1~8
         Wire.endTransmission();
         print2hex(read_data, 1); //
      }
      Serial.write(LF);
      Serial.write(CR);
   }
   delay(500);
}

void print2hex(byte *data, byte length) //Print Hex code
{
   Serial.print("0x");
   for (int i = 0; i < length; i++)
   {
      if (data[i] < 0x10)
      {
         Serial.print("0");
      }
      Serial.print(data[i], HEX);
      Serial.write(SPC);
   }
}

void setup(){
   Wire.begin();           // join i2c bus (address optional for master)
   Wire.setClock(2000000); // 200Khz
   Serial.begin(115200);   // start serial for output (Spped)
   //put your setup code here, to run once:

   pinMode(RESET_PIN, OUTPUT);
   pinMode(EN_PIN, OUTPUT);

   delay(100);  //wait for 100[msec]
   Init_TS20(); //Initialize TS20
   delay(100);  //wait for 100[msec]
  
}
void loop() {

	byte read_data[3]={0};
	delay(5);
	
	Wire.beginTransmission(TS20_SLAVE_GND); // sned ic slave address
	Wire.write(byte(Output1)); // sends register address
	Wire.endTransmission(); // stop transmitting
	Wire.requestFrom(TS20_SLAVE_GND,3); // key data read (3 byte)
	read_data[0]=Wire.read();
	read_data[1]=Wire.read();
	read_data[2]=Wire.read();
   Wire.endTransmission(); //

   Serial.write(10); 
	Serial.print(" Touch Sensor Output Data  ---- > ");// Test Code
	delay(10);

   print2hex(read_data, 1);
   print2hex((read_data + 1), 1);
   print2hex((read_data + 2), 1);
   //Serial.write(LF); 	
	Serial.write(CR);
   delay(50);
}

void  Init_TS20(void)
{
   //------------------ Software Reset Enable (Set)----------------------
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(GTRL2)); // sends register address
   Wire.write(byte(0x1A));  // sends register data RB_SEL=Noram, Sleep Mode=Disable
   // RB_SEL=Noram, Sleep Mode=Disable, S/W Reset=Enable, IMP_SEL=High Imp. S/M_Mode=Multi, VPM=0
   Wire.endTransmission(); // stop transmitting

   //----------------- Port Control ------------------------------------
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Port_CTRL1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Port_CTRL2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting    
   
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Port_CTRL3)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Port_CTRL4)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting  

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Port_CTRL5)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Port_CTRL6)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting

   //--------------- Sensitivty control,(threshold level)-----------
   // if SSC bit =1(Normal Step) , (Data Value x 0.2%)+0.15%
   // if SSC bit =0(Fine Stemp) , (Data Value x 0.1%)+0.05%
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM1)); // sends register address
   Wire.write(byte(0x55));
   // if SSC bit- 1 -> (0x55 = 0x5 X 0.2% + 0.15% = 1.15%)
   // if SSC bit- 0 -> (0x55 = 0x5 X 0.1% + 0.05% = 0.55%)
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM2)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting
  
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM3)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM4)); // sends register address
   Wire.write(byte(0xF5)); // sends register data
   Wire.endTransmission(); // stop transmitting
   
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM5)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   
  
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM6)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting  

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM7)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM8)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TS20_SLAVE_GND); /// send ic slave address
   Wire.write(byte(Sensitivity_PWM9)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Sensitivity_PWM10)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TS20_SLAVE_GND); // sned ic slave address
   Wire.write(byte(Sensitivity_PWM11)); // sends register address
   Wire.write(byte(0x55)); // sends register data
   Wire.endTransmission(); // stop transmitting    
   
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(GTRL1)); // sends register address
   Wire.write(byte(0x4B)); // sends register data
   //SSC= Normal Step, MS=Auto Mode, FTC=5sec, Respons Time Control(RTC)= 011b + 1 = 4 times
   Wire.endTransmission(); // stop transmitting   
   //------------------ Calibration On ---------------------------
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Cal_HOLD1)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting   
 
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Cal_HOLD2)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting  

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Cal_HOLD3)); // sends register address
   Wire.write(byte(0x00)); // sends register data
   Wire.endTransmission(); // stop transmitting 

   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Err_CTRL)); // sends register address
   Wire.write(byte(0x0D)); // sends register data
   Wire.endTransmission(); // stop transmitting   
   
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(Cal_CTRL)); // sends register address
   Wire.write(byte(0xFA)); // sends register data
   Wire.endTransmission(); // stop transmitting  
   
   //------------------ Software Reset Disable (Clear) -----------------------
   Wire.beginTransmission(TS20_SLAVE_GND); // send ic slave address
   Wire.write(byte(GTRL2)); // sends register address
   Wire.write(byte(0x12)); // sends register data
   // RB_SEL=Noram, Sleep Mode=Disable, S/W Reset=Disable, IMP_SEL=High Imp. S/M_Mode=Multi, VPM=0
   Wire.endTransmission(); // stop transmitting   
    
   }
// End 