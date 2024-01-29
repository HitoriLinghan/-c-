#include <Arduino.h>
#include  "Servo.h"
#define SER 2
#define RCK 3
#define SCK 4

unsigned char angle[8] =  {10,20,30,40,50,60,70,80 };
int wilth_us1 = map(angle[0], 0, 180, 1000, 2500);
int wilth_us2 = map(angle[1], 0, 180, 1000, 2500);
int wilth_us3 = map(angle[2], 0, 180, 1000, 2500);
int wilth_us4 = map(angle[3], 0, 180, 1000, 2500);
int wilth_us5 = map(angle[4], 0, 180, 1000, 2500);
int wilth_us6 = map(angle[5], 0, 180, 1000, 2500);
int wilth_us7 = map(angle[6], 0, 180, 1000, 2500);
int wilth_us8 = map(angle[7], 0, 180, 1000, 2500);

void setup()
{
	pinMode(2, OUTPUT);
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	//Serial.begin(9600);
	PORTD = 0xf0;
}

void loop()
{	
	for(int i=0;i<8;i++)
	Show_PWM(0x01<<i, 1500);
	
  
}
extern void Show_PWM(u8 Qx, uint32_t wilth_us)//Qx是对八位整体赋值   wilth_ms是位宽(单位毫秒）
{
  
	
	ShowByte(Qx);
	delayMicroseconds(wilth_us);
	ShowByte(~Qx);
	delayMicroseconds(20000 - wilth_us);


}
extern void ShowByte(unsigned char  Byte)//按高到低位显示输入的字节
{
	for (int j = 0; j < 8; j++)
	{
		digitalWrite(SER, Byte&(0x80 >> j));
		digitalWrite(SCK, 1);
		digitalWrite(SCK, 0);
	}
	digitalWrite(RCK, 0);
	digitalWrite(RCK, 1);
}