/*
 Name:		电机驱动.ino
 Created:	2023/10/9 23:37:14
 Author:	llh13
*/
int read_in;
int a, b, c;
// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(3, INPUT_PULLUP);
	Serial.begin(9600);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	read_in = analogRead(3);
	a = Serial.read();
	Serial.println("first is analog\n");
	Serial.println(read_in);
	Serial.println("second is RX:");
	Serial.println(a);
}
