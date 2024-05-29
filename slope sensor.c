#include "MPU9250.h"

MPU9250 mpu;
MPU9250Setting setting;

const int numReadings = 10;  
float readings[numReadings];  
int readIndex = 0;  
float total = 0;  
float average = 0;  

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    // MPU9250 설정
    setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {  // I2C 주소가 0x68인 경우
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    mpu.selectFilter(QuatFilterSel::MADGWICK);

    Serial.println("MPU9250 초기화 완료");

    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
    }
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 150) {  
            update_pitch();
            prev_ms = millis();
        }
    }
}

void update_pitch() {
    
    total = total - readings[readIndex];

    readings[readIndex] = mpu.getPitch();

    total = total + readings[readIndex];

    readIndex = readIndex + 1;

    if (readIndex >= numReadings) {
        readIndex = 0;
    }

    average = total / numReadings;

    int rounded_pitch = round(average / 5) * 5;

    Serial.print("경사도: ");
    Serial.println(rounded_pitch);
