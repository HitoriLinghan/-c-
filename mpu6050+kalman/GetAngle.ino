#include <Wire.h>

#define MPU6050_ADDR 0x68 // mpu6050地址
#define MPU6050_SMPLRT_DIV 0x19   //陀螺仪采样率分频寄存器
#define MPU6050_CONFIG 0x1a       //配置寄存器低通滤波
#define MPU6050_GYRO_CONFIG 0x1b  //陀螺仪配置寄存器
#define MPU6050_ACCEL_CONFIG 0x1c //加速度传感器配置寄存器
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42

class OneDimensionalKalmanFilter {
public:
  OneDimensionalKalmanFilter(float initial_state, float initial_error_estimate,
                             float process_noise, float measurement_noise,
                             float transition_matrix = 1.0f,
                             float measurement_matrix = 1.0f)
      : x_(initial_state), // 当前状态估计值
                           // 这是滤波器开始时对系统状态的初始猜测。在这个一维情况下，可以是物体的位置、速度或其他需要过滤的一维量的初值。
        p_(initial_error_estimate), // 初始状态误差协方差矩阵的对角线元素
                                    // 这是一个正数，它反映了我们对初始状态估计的信任度有多高。较大的值意味着更大的不确定性或更宽泛的误差范围。
        q_(process_noise),     // 过程噪声协方差
                               // 这个参数描述了系统内部噪声的影响程度。在卡尔曼滤波过程中，系统模型可能会因为未知因素而产生预测误差，此参数就是用来量化这种误差的标准偏差平方。
        r_(measurement_noise), // 测量噪声协方差
                               // 该参数衡量传感器读数中包含的噪声水平，也就是实际测量值与真实状态之间的随机误差。
        f_(transition_matrix), // 状态转移矩阵（在简单的一维情况下通常是1）
        h_(measurement_matrix) { // 测量矩阵（在简单的一维情况下通常是1）
  }

  void predict() {
    x_ = f_ * x_;           // 预测下一状态
    p_ = f_ * p_ * f_ + q_; // 更新状态误差协方差
  }

  void update(float measurement) {
    // 计算卡尔曼增益
    float kalman_gain = p_ * h_ / (h_ * p_ * h_ + r_);

    // 更新状态估计值
    x_ += kalman_gain * (measurement - h_ * x_);

    // 更新状态误差协方差
    p_ = (1.0f - kalman_gain * h_) * p_;
  }

  float getStateEstimate() const { return x_; }

private:
  float x_; // 当前状态估计值
  float p_; // 状态误差协方差矩阵
  float q_; // 过程噪声协方差
  float r_; // 测量噪声协方差
  float f_; // 状态转移矩阵
  float h_; // 测量矩阵
};

byte readMPU6050(byte reg);
void writeMPU6050(byte reg, byte data);
void MPU6050_update_rotation(); //得到角速度xyz
void shanwai_oscilloscope_send(uint8_t *data, uint8_t len);

OneDimensionalKalmanFilter kfx(0, 100, 0.1, 1); //
OneDimensionalKalmanFilter kfy(0, 100, 0.1, 1);
OneDimensionalKalmanFilter kfz(0, 100, 0.1, 1);

int16_t rx, ry, rz;
float x, y, z;
float angleX, angleY, angleZ;
float LSB = 65.5; // 500dps
float offsetX = 0, offsetY = 0, offsetZ = 0;
long past = 0;
void setup() {
  angleX = 0, angleY = 0, angleZ = 0;
  pinMode(13, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
  MPU6050_init();
  delay(100);
}

void loop() {

  MPU6050_update_rotation();

  kfx.predict();
  kfx.update(angleX);

  Serial.print(kfx.getStateEstimate());
  Serial.print(",");
  kfy.predict();
  kfy.update(angleY);
  Serial.print(kfy.getStateEstimate());
  Serial.print(",");
  kfz.predict();
  kfz.update(angleZ);
  Serial.println(kfz.getStateEstimate());

  // Serial.println((millis() - past));
}

void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

byte readMPU6050(byte reg) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  byte data = Wire.read();
  return data;
}
void MPU6050_init() {

  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  writeMPU6050(MPU6050_CONFIG, 0x36); //低通滤波拉满了
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08); // 500 dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  delay(100);
  int numReadings = 1000;
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < numReadings; i++) {
    MPU6050_update_rotation();
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(1); // 稍微延迟以获取新的读数
  }
  offsetX = sumX / numReadings;
  offsetY = sumY / numReadings;
  offsetZ = sumZ / numReadings;
}

void MPU6050_update_rotation() {
  long currentTime = millis(); // 先记录当前时间
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom((int)MPU6050_ADDR, 6);

  rx = Wire.read() << 8 | Wire.read();
  ry = Wire.read() << 8 | Wire.read();
  rz = Wire.read() << 8 | Wire.read();

  x = (rx / LSB) - offsetX;
  y = (ry / LSB) - offsetY;
  z = (rz / LSB) - offsetZ;

  // 使用旧的past值和新的currentTime进行积分计算
  angleX += x * ((currentTime - past) / 1000.0);
  angleY += y * ((currentTime - past) / 1000.0);
  angleZ += z * ((currentTime - past) / 1000.0);

  past = currentTime; // 更新'past'到当前时间以供下次计算
}
void shanwai_oscilloscope_send(uint8_t *data, uint8_t len) {
  const uint8_t cmdhead[2] = {0x03, 0xfc};
  const uint8_t cmdtail[2] = {0xfc, 0x03};
  Serial.write(cmdhead, sizeof(cmdhead));
  Serial.write((uint8_t *)data, sizeof(uint8_t) * len);
  Serial.write(cmdtail, sizeof(cmdtail));
}

/*在 `OneDimensionalKalmanFilter` 类中，构造函数的参数分别代表：

1.
`initial_state`（初始状态估计值）：这是滤波器开始时对系统状态的初始猜测。在这个一维情况下，可以是物体的位置、速度或其他需要过滤的一维量的初值。

   在代码示例中：
   ```cpp
   OneDimensionalKalmanFilter kfX(0, ...
   ```
   这里设置的是 `kfX` 对应的状态变量初始值为 0，假设是在处理 x
轴的角度数据，则表示初始角度估计为 0 度。

2.
`initial_error_estimate`（初始状态误差协方差矩阵的对角线元素）：这是一个正数，它反映了我们对初始状态估计的信任度有多高。较大的值意味着更大的不确定性或更宽泛的误差范围。

   示例中的：
   ```cpp
   ... 100, ...
   ```
   表示我们对 x
轴初始角度估计的误差协方差较大，即开始时我们对该角度的估计有较大的不确定性。

3.
`process_noise`（过程噪声协方差）：这个参数描述了系统内部噪声的影响程度。在卡尔曼滤波过程中，系统模型可能会因为未知因素而产生预测误差，此参数就是用来量化这种误差的标准偏差平方。

   示例中的：
   ```cpp
   ..., 0.1, ...
   ```
   表示我们认为 x 轴角度预测时受到的过程噪声标准偏差约为
0.1（单位取决于具体应用）。

4.
`measurement_noise`（测量噪声协方差）：该参数衡量传感器读数中包含的噪声水平，也就是实际测量值与真实状态之间的随机误差。

   示例中的：
   ```cpp
   ..., 5, ...
   ```
   表示我们认为从陀螺仪获取的 x 轴角度测量值的噪声标准偏差约为
5（同样，单位需根据实际情况定义）。

请注意，在实际应用中，这些参数的选择通常需要通过实验调试或者基于物理模型和传感器规格来确定。对于不同的应用场景，可能需要调整这些参数以获得最佳的滤波效果。*/
