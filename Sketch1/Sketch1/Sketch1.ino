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
    //�⴫��������ģ�ź�
    int val = map(bright, 1024, 320, 0,255);
    //�ź�brightӳ��Ϊval
 if (val >= 150)
    {
        analogWrite(led, 255);
    }
    //���ȹ���ֱ��Ϩ��
    if (val < 150)
    {
        analogWrite(led, 0);
    }
    //���ȹ�Сֱ������
     else 
    {
        analogWrite(led, val);
    }
    //������val������
    Serial.println(bright);
    //���bright�����ڼ�����

}