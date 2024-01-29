short bright;
short led = 10;
void setup()
{
    pinMode(A1, INPUT_PULLUP);
    pinMode(10, OUTPUT);
    Serial.begin(9600);
}
void loop()
{

    bright = analogRead(A1);
    //光传感器输入模信号
    int val = map(bright, 1024, 320, 0,255);
    //信号bright映射为val
 if (val >= 150)
    {
        analogWrite(led, 255);
    }
    //亮度过大直接熄灯
    if (val < 150)
    {
        analogWrite(led, 0);
    }
    //亮度过小直接亮灯
     else 
    {
        analogWrite(led, val);
    }
    //否则按照val来亮灯
    Serial.println(bright);
    //输出bright到串口监视器

}