#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address

float AOAcmd, yawcmd, pitchcmd, rollcmd;//控制量需求值

int AOA, aoacanard;

int canardvalue, ruddervalue, LWvalue, RWvalue;//实际给舵机的角度值

byte PWM_PIN1 = 3;//副翼输入D3

byte PWM_PIN2 = 5;//升降输入D5

byte PWM_PIN4 = 6;//方向输入D6

int pwm_ali, pwm_ele, pwm_rud;//输入的pwm值

float volt;//攻角传感器的电压

float accx, accy, accz, wx, wy, wz;//IMU三轴加速度角速率

float accx0, accy0, accz0, AccX0, AccY0, AccZ0;//IMU零点校准用

float wx0, wy0, wz0, GyroX0, GyroY0, GyroZ0;//IMU零点校准用

int i;//IMU零点校准用

float x_n, y_n, x_n_1, y_n_1;//偏航阻尼器高通滤波器参数

float wx_old, wx_new, wy_old, wy_new, accy_old, accy_new;//一阶低通滤波相关参数定义

float alpha = 0.4; //alpha是滤波系数，越大响应越快，取值范围0~1

int ruddervalue_new, canardvalue_new, LWvalue_new, RWvalue_new;

int ruddervalue_old, canardvalue_old, LWvalue_old, RWvalue_old;

unsigned long timer;

float AOAerrorsum, AOAerror;



void setup()

{

	// Serial.begin(9600);

	//-------------------------//1.初始化//---------------------------------

	initialize_MPU6050();

	pinMode(2, OUTPUT);//设定舵机接口为输出接口 //左翼舵机D2

	pinMode(4, OUTPUT);//设定舵机接口为输出接口//右翼舵机D4 


	servopulse(90, 2, 90, 4, 90, 9, 90, 10);//舵机归中

	delay(500);


	//-------------------------//2.IMU校准//---------------------------------

	  //闪烁提示开始校准，校准时LED常亮

	pinMode(LED_BUILTIN, OUTPUT);

	digitalWrite(LED_BUILTIN, HIGH);
	// turn the LED on (HIGH is the voltage level)

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, LOW);
	// turn the LED off by making the voltage LOW

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, HIGH);
	// turn the LED on (HIGH is the voltage level)

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, LOW);
	// turn the LED off by making the voltage LOW

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, HIGH);
	// turn the LED on (HIGH is the voltage level)

	delay(200);
	// wait for a second

	//采集零点

	for (i = 1; i <= 50; i += 1)

	{

		// === Read acceleromter data === //

		Wire.beginTransmission(MPU);

		Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)

		Wire.endTransmission(false);

		Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

		//For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet

		AccX0 = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value

		AccY0 = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value

		AccZ0 = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value 

		Wire.beginTransmission(MPU);

		Wire.write(0x43); // Gyro data first register address 0x43

		Wire.endTransmission(false);

		Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

		GyroX0 = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet

		GyroY0 = (Wire.read() << 8 | Wire.read()) / 32.8;

		GyroZ0 = (Wire.read() << 8 | Wire.read()) / 32.8;

		accx0 = accx0 + AccX0;

		accy0 = accy0 + AccY0;

		accz0 = accz0 + AccZ0;

		wx0 = wx0 + GyroX0;

		wy0 = wy0 + GyroY0;

		wz0 = wz0 + GyroZ0;

		delay(50);

	}

	accx0 = accx0 / 50;

	accy0 = accy0 / 50;

	accz0 = accz0 / 50;

	wx0 = wx0 / 50;

	wy0 = wy0 / 50;

	wz0 = wz0 / 50;

	//闪烁提示校准完成，然后熄灭

	digitalWrite(LED_BUILTIN, HIGH);
	// turn the LED on (HIGH is the voltage level)

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, LOW);
	// turn the LED off by making the voltage LOW

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, HIGH);
	// turn the LED on (HIGH is the voltage level)

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, LOW);
	// turn the LED off by making the voltage LOW

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, HIGH);
	// turn the LED on (HIGH is the voltage level)

	delay(200);
	// wait for a second

	digitalWrite(LED_BUILTIN, LOW);
	// turn the LED off by making the voltage LOW

	delay(200);
	// wait for a second

}



void loop()

{

	//-------------------------//1.IMU数据处理//---------------------------------

	read_IMU();

	accy = accy - accy0;//读取扣除零点的数据

	wx = wx - wx0;

	wy = wy - wy0;

	wz = wz - wz0;



	//一阶低通滤波(wz因为是高通滤波，此处不处理)

	wx_new = wx;

	wx = wx_new * alpha + (1 - alpha) * wx_old;

	wx_old = wx;

	wy_new = wy;

	wy = wy_new * alpha + (1 - alpha) * wy_old;

	wy_old = wy;

	accy_new = accy;

	accy = accy_new * alpha + (1 - alpha) * accy_old;

	accy_old = accy;



	//添加死区防止IMU引起抖舵

	if (wy<1 && wy>-1)

		wy = 0;

	if (accy<0.2 && accy>-0.2)

		accy = 0;

	if (wx<1 && wx>-1)//dead zone

		wx = 0;

	if (wz<1 && wz>-1)//dead zone

		wz = 0;



	//-------------------------//2.接收机数据处理//---------------------------------

	  //读取3个通道PWM

	pwm_ali = pulseIn(PWM_PIN1, HIGH);//检测高电平 

	pwm_ele = pulseIn(PWM_PIN2, HIGH);//检测高电平

	pwm_rud = pulseIn(PWM_PIN4, HIGH);//检测高电平



	//pwm换算成指令

	yawcmd = map(pwm_rud, 1000, 2000, -35, 35);//方向舵只有舵量指令，最大±35°

	pitchcmd = map(pwm_ele, 1000, 2000, 40, -40); //俯仰攻角指令，杆量最大对应40°，拉杆是1000，推杆是2000

	rollcmd = map(pwm_ali, 1000, 2000, -400, 400);//滚转角速率指令，杆量最大对应400°/s



	//-------------------------//3.攻角传感器数据处理//---------------------------------



	float a0 = analogRead(A1);//攻角传感器接A1

	volt = a0 * 5 / 1023;//读取电压

	AOA = (volt - 2.5) * 180 / 2.5; //换算成攻角，传感器在右侧  

	if (AOA > 90)

	{
		AOA = 90;
	}

	if (AOA < -90)

	{
		AOA = -90;
	}//限制输出范围   



//-------------------------//4.鸭翼控制//---------------------------------

	canardvalue = pitchcmd + wy * 0.07 - 1.2 * AOA;//鸭翼负反馈，恒定配平，中立点自己调，1deg/s角速率对应0.07deg鸭翼; 

	aoacanard = AOA + canardvalue;

	if (aoacanard >= 13) //如果鸭翼自己的攻角大于13°，要限制鸭翼攻角

	{
		aoacanard = 13;
	}

	if (aoacanard <= -25) //如果鸭翼自己的攻角小于-25°，要限制鸭翼攻角

	{
		aoacanard = -25;
	}

	canardvalue = aoacanard - AOA;//反过来计算限制鸭翼攻角后的鸭翼舵量

	//鸭翼舵量限制 

	if (canardvalue <= -50)

	{
		canardvalue = -50;
	}//鸭翼下偏50°

	if (canardvalue >= 20)

	{
		canardvalue = 20;
	}//鸭翼上偏20°

	canardvalue = 90 - canardvalue;// 装机后发现鸭翼负是上偏,修改方向，叠加舵机中立点90°,得到舵机输出的位置。



	//-------------------------//5.方向舵控制//---------------------------------



	 //航向增稳和偏航阻尼器//

	x_n = wz;//单片机采一次角速率，赋值给xn

	y_n = x_n - x_n_1 + 0.9 * y_n_1;//系数由dsys求得，手动填入，从离散传递函数转差分方程的公式，见matlab帮助filter(B,A,xn)

	x_n_1 = x_n;

	y_n_1 = y_n;

	if (y_n<1 && y_n>-1)//dead zone

		y_n = 0;//dead zone



	if (AOA > 25)

	{
		ruddervalue = yawcmd * 0.5 - y_n * 0.15 - accy * 15 + rollcmd * 0.06 + 90;
	}

	//1deg/s角速率对应0.15deg方向舵，1g横向加速度对应15°方向舵，1deg/s滚转指令输入对应0.06°方向舵;版权所有 天穹（b站id 暖风新叶柳）

	if (AOA > 5 && AOA <= 25)

	{
		ruddervalue = yawcmd * 0.5 - y_n * 0.15 - accy * 15 * AOA / 25 + rollcmd * 0.06 * AOA / 25 + 90;
	}

	//AOA越大，则滚转指令输入对应的方向舵舵量越大

	if (AOA <= 5)

	{
		ruddervalue = yawcmd * 0.5 - y_n * 0.15 - accy * 3 + rollcmd * 0.012 + 90;
	}



	if (ruddervalue >= 135)//方向舵量限制

		ruddervalue = 135;

	if (ruddervalue <= 45)

		ruddervalue = 45;



	//-------------------------//6.升降副翼控制//---------------------------------





	rollcmd = rollcmd * 0.15 + wx * 0.04;//1deg/s滚转角速率误差对应0.04deg副翼,实测飞机滚转速率约每秒2圈。角速率阻尼D取0.04，不能调太大，否则振荡。





	//升降副翼PD控制，I暂时没用。

  //  AOAerror=pitchcmd-AOA;

  //  AOAerrorsum+=AOAerror/50.0;

  //  if(AOAerrorsum>=50)

  //  {AOAerrorsum=50;}

  //  if(AOAerrorsum<=-50)

  //  {AOAerrorsum=-50;}



	pitchcmd = pitchcmd * 0.7 + wy * 0.05 - AOA * 0.15;//+AOAerrorsum*0.2;//攻角误差小时，鸭翼可产生足够的俯仰稳定性，1deg/s俯仰角速率对应0.05deg副翼,1AOA=-0.15°舵面





	LWvalue = rollcmd + pitchcmd + 90 - 3;//-是下偏

	RWvalue = rollcmd - pitchcmd + 90 + 3;//左右副翼相反,+是下偏



	//-------------------------//7.偏离抑制器，大攻角时副翼反向控制偏航//---------------------------------

	if (AOA > 50)

	{

		LWvalue = LWvalue + wz * AOA / 200;//飞机自行左偏航时，副翼向左打，攻角越大增益越大

		RWvalue = RWvalue + wz * AOA / 200;

	}

	if (LWvalue >= 130)
		//升降副翼舵量限制

	{
		LWvalue = 130;
	}

	if (LWvalue <= 50)

	{
		LWvalue = 50;
	}

	if (RWvalue >= 130)

	{
		RWvalue = 130;
	}

	if (RWvalue <= 50)

	{
		RWvalue = 50;
	}



	//-------------------------//8.舵机指令//---------------------------------

	 //专治静态抖舵

	ruddervalue_new = ruddervalue;

	if ((ruddervalue_new - ruddervalue_old) <= 1 && (ruddervalue_new - ruddervalue_old) >= -1)

	{
		ruddervalue = ruddervalue_old;
	}//滤波，如果本次舵机输出值和上次的变化在正负1以内，那本次舵机就不变化，保留上次的值。

	ruddervalue_old = ruddervalue;



	canardvalue_new = canardvalue;

	if ((canardvalue_new - canardvalue_old) <= 1 && (canardvalue_new - canardvalue_old) >= -1)

	{
		canardvalue = canardvalue_old;
	}//滤波，如果本次舵机输出值和上次的变化在正负1以内，那本次舵机就不变化，保留上次的值。

	canardvalue_old = canardvalue;



	LWvalue_new = LWvalue;

	if ((LWvalue_new - LWvalue_old) <= 1 && (LWvalue_new - LWvalue_old) >= -1)

	{
		LWvalue = LWvalue_old;
	}//滤波，如果本次舵机输出值和上次的变化在正负1以内，那本次舵机就不变化，保留上次的值。

	LWvalue_old = LWvalue;



	RWvalue_new = RWvalue;

	if ((RWvalue_new - RWvalue_old) <= 1 && (RWvalue_new - RWvalue_old) >= -1)

	{
		RWvalue = RWvalue_old;
	}//滤波，如果本次舵机输出值和上次的变化在正负1以内，那本次舵机就不变化，保留上次的值。版权所有 天穹（b站id 暖风新叶柳）

	RWvalue_old = RWvalue;



	//发送PWM

	servopulse(ruddervalue, 10, canardvalue, 9, LWvalue, 2, RWvalue, 4);



	// Serial.print(LWvalue);Serial.print(","); Serial.print(RWvalue);Serial.print(",");

	// Serial.print(ruddervalue);Serial.print(",");Serial.println(canardvalue);

}



void initialize_MPU6050() {

	Wire.begin();
	// Initialize comunication

	Wire.beginTransmission(MPU);
	// Start communication with MPU6050 // MPU=0x68

	Wire.write(0x6B);
	// Talk to the register 6B

	Wire.write(0x00);
	// Make reset - place a 0 into the 6B register

	Wire.endTransmission(true);
	//end the transmission

	// Configure Accelerometer

	Wire.beginTransmission(MPU);

	Wire.write(0x1C);
	//Talk to the ACCEL_CONFIG register

	Wire.write(0x10);
	//Set the register bits as 00010000 (+/- 8g full scale range)

	Wire.endTransmission(true);

	// Configure Gyro

	Wire.beginTransmission(MPU);

	Wire.write(0x1B);
	// Talk to the GYRO_CONFIG register (1B hex)

	Wire.write(0x10);
	// Set the register bits as 00010000 (1000dps full scale)

	Wire.endTransmission(true);

}



void read_IMU() {

	// === Read acceleromter data === //

	Wire.beginTransmission(MPU);

	Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)

	Wire.endTransmission(false);

	Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

	//For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet

	accx = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value

	accy = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value

	accz = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value

	// === Read gyro data === //

	Wire.beginTransmission(MPU);

	Wire.write(0x43); // Gyro data first register address 0x43

	Wire.endTransmission(false);

	Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

	wx = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet

	wy = (Wire.read() << 8 | Wire.read()) / 32.8;

	wz = (Wire.read() << 8 | Wire.read()) / 32.8;

}



void servopulse(int angle1, int servopin1, int angle2, int servopin2, int angle3, int servopin3, int angle4, int servopin4)//定义一个脉冲函数

{

	if (micros() >= timer + 20000)

	{

		timer = micros(); //当前时间(ms)

		int pulsewidth1 = (angle1 * 11) + 500;
		//将角度转化为500-2480的脉宽值

		digitalWrite(servopin1, HIGH);
		//将舵机接口电平至高

		delayMicroseconds(pulsewidth1);
		//延时脉宽值的微秒数

		digitalWrite(servopin1, LOW);
		//将舵机接口电平至低



		int pulsewidth2 = (angle2 * 11) + 500;
		//将角度转化为500-2480的脉宽值

		digitalWrite(servopin2, HIGH);
		//将舵机接口电平至高

		delayMicroseconds(pulsewidth2);
		//延时脉宽值的微秒数

		digitalWrite(servopin2, LOW);
		//将舵机接口电平至低



		int pulsewidth3 = (angle3 * 11) + 500;
		//将角度转化为500-2480的脉宽值

		digitalWrite(servopin3, HIGH);
		//将舵机接口电平至高

		delayMicroseconds(pulsewidth3);
		//延时脉宽值的微秒数

		digitalWrite(servopin3, LOW);
		//将舵机接口电平至低



		int pulsewidth4 = (angle4 * 11) + 500;
		//将角度转化为500-2480的脉宽值

		digitalWrite(servopin4, HIGH);
		//将舵机接口电平至高

		delayMicroseconds(pulsewidth4);
		//延时脉宽值的微秒数

		digitalWrite(servopin4, LOW);
		//将舵机接口电平至低

	}



}
