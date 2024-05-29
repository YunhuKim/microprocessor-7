#include "MPU9250.h"

MPU9250 mpu;
MPU9250Setting setting;

const int numReadings = 10;  // 이동 평균에 사용할 샘플 수
float readings[numReadings];  // 샘플을 저장할 배열
int readIndex = 0;  // 현재 읽기 인덱스
float total = 0;  // 샘플의 총합
float average = 0;  // 이동 평균 값

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

    // MPU9250 초기화
    if (!mpu.setup(0x68, setting)) {  // I2C 주소가 0x68인 경우
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // Madgwick 필터 선택
    mpu.selectFilter(QuatFilterSel::MADGWICK);

    Serial.println("MPU9250 초기화 완료");

    // 이동 평균 초기화
    for (int i = 0; i < numReadings; i++) {
        readings[i] = 0;
    }
}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 150) {  // 출력 주기를 125ms로 설정
            update_pitch();
            prev_ms = millis();
        }
    }
}

void update_pitch() {
    // 이전 값을 총합에서 제거
    total = total - readings[readIndex];

    // 새로운 피치 값을 읽고 배열에 저장
    readings[readIndex] = mpu.getPitch();

    // 새로운 값을 총합에 추가
    total = total + readings[readIndex];

    // 읽기 인덱스를 다음으로 이동
    readIndex = readIndex + 1;

    // 인덱스가 배열 크기를 초과하면 처음으로 돌아감
    if (readIndex >= numReadings) {
        readIndex = 0;
    }

    // 평균 계산
    average = total / numReadings;

    // 5도 단위로 반올림
    int rounded_pitch = round(average / 5) * 5;

    // 결과 출력
    Serial.print("경사도: ");
    Serial.println(rounded_pitch);