# 1 "D:\\code\\c语言自学\\QAQ\\QAQ\\sketches\\QAQ.ino"




/**************************************************************

***  接线：

    * D5接En（闭环驱动板屏幕上的En选项选择L或Hold）

    * D6接Stp

    * D7接Dir

    * V+和Gnd接10~28V供电

    * 非工业套餐（不带光耦隔离）要把Arduino控制板的Gnd和闭环驱动板的Gnd接在一起（共地）



*** 注意事项：

    * Arduino控制板和闭环驱动板的两个Gnd要接在一起

    * 先接好线再通电，不要带电拔插！！！

    * 上电时，先通10~28V供电，再通Arduino控制板USB供电！！！避免一些效应造成损坏

    * 断电时，先断Arduino控制板USB供电，再断10~28V供电。

***************************************************************/
# 20 "D:\\code\\c语言自学\\QAQ\\QAQ\\sketches\\QAQ.ino"
long i = 0; bool cntDir = false;

void setup() {
 // put your setup code here, to run once:

 pinMode(5 /* the number of the En pin*/, 0x1); digitalWrite(5 /* the number of the En pin*/, 0x1); // initialize the En pin as an output
 pinMode(6 /* the number of the Stp pin*/, 0x1); digitalWrite(6 /* the number of the Stp pin*/, 0x1); // initialize the Stp pin as an output
 pinMode(7 /* the number of the Dir pin*/, 0x1); digitalWrite(7 /* the number of the Dir pin*/, 0x1); // initialize the Dir pin as an output
}

void loop() {
 // put your main code here, to run repeatedly:

 /**********************************************************

	***  高低电平的时间间隔，即脉冲时间的一半(控制电机转动速度)

	**********************************************************/
# 36 "D:\\code\\c语言自学\\QAQ\\QAQ\\sketches\\QAQ.ino"
 delayMicroseconds(800); //600us

   /**********************************************************

   ***  取反D6（Stp引脚）

   **********************************************************/
# 41 "D:\\code\\c语言自学\\QAQ\\QAQ\\sketches\\QAQ.ino"
 digitalWrite(6 /* the number of the Stp pin*/, !digitalRead(6 /* the number of the Stp pin*/));

 /**********************************************************

	***  记录IO取反次数（IO取反次数 = 2倍的脉冲数）

	**********************************************************/
# 46 "D:\\code\\c语言自学\\QAQ\\QAQ\\sketches\\QAQ.ino"
 if (cntDir) {--i; }
 else {++i; }

 /**********************************************************

	***  PA6（Stp引脚）取反了6400次，即发送了3200个脉冲

	*** 16细分下，发送3200个脉冲电机转动一圈（1.8度电机）

	**********************************************************/
# 53 "D:\\code\\c语言自学\\QAQ\\QAQ\\sketches\\QAQ.ino"
 if (i >= 6400)
 {
  digitalWrite(7 /* the number of the Dir pin*/, 0x0); cntDir = true;
   ; //切换方向转动
 }
 else if (i == 0)
 {
  digitalWrite(7 /* the number of the Dir pin*/, 0x1); cntDir = false;
  ; //切换方向转动
 }
}
