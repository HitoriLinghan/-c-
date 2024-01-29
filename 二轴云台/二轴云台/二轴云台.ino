
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Servo.h>
#include<stdio.h>
Servo one, two;
int pos = 90;
int servo_one = 5;
int servo_two = 6;
MPU6050 mpu6050(Wire);
float angle_x0, angle_z0;
float angle_x1, angle_z1;


long timer = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu6050.update();
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    one.attach(servo_one);
    two.attach(servo_two);
    angle_x0 = mpu6050.getAngleY();
    angle_z0 = mpu6050.getAngleZ();

    
    one.write(pos);
    two.write(pos);
   

}

void loop() {
    mpu6050.update();
    
   

        angle_x1 = mpu6050.getAngleY();
           
        angle_z1 = mpu6050.getAngleZ();
        
        float diff_x = angle_x0 - angle_x1;
        float diff_z = angle_z0 - angle_z1;
       
        int to_x = pos + diff_x;
        int to_z = pos + diff_z;
        if (diff_x > 0.1 || diff_x < -0.1)
        {
            one.write(to_x);
        }
        if (diff_z > 1.5 || diff_z < -1.5)
        {
            two.write(to_z);
        }
            int angleX = to_x;
            int angleZ = to_z;


}
