#include "aeb.h"

#include "my_stdio.h"

#include "output_status.h"

// 목표 정지 거리 및 허용 오차
#define AEB_TOLERANCE_MM                5       // 허용 오차 ±5mm
#define AEB_EMERGENCY_THRESHOLD_MM      100     // 10cm 이하 무조건 긴급제동

// 실측 데이터 기반 제동거리 계산 계수 (완전이차 모델: ax²+bx+c)
#define AEB_SPEED_COEFF_A               -27     // 이차항 계수 * 1000
#define AEB_SPEED_COEFF_B               6496    // 일차항 계수 * 1000
#define AEB_SPEED_COEFF_C               -112642 // 상수항 * 1000
#define AEB_SPEED_DIVIDER               1000    // 나누기 계수

/*
 === 실측 데이터 기반 완전이차 모델 검증 결과(08.01) ===

 테스트 데이터 (AEB_test_1.txt 기반):
 속도(km/h) | 실측 제동거리(mm) | 예측 제동거리(mm) | 오차(mm) | 오차율(%)
 -----------|-------------------|-------------------|----------|----------
 30     |        56         |        57         |    +1    |   1.8%
 40     |       103         |       103         |     0    |   0.0%
 50     |       147         |       143         |    -4    |   2.7%
 60     |       178         |       178         |     0    |   0.0%
 70     |       209         |       207         |    -2    |   1.0%
 80     |       226         |       231         |    +5    |   2.2%
 90     |       249         |       249         |     0    |   0.0%
 100     |       264         |       262         |    -2    |   0.8%

 모델 성능 지표:
 - 평균 절대 오차: 1.75mm (목표: ±5mm 이하)
 - 최대 절대 오차: 5mm
 - 결정계수 (R²): 99.87% (매우 높은 정확도)
 - 평균 오차율: 1.16%

 완전이차 모델 계수 (정수 연산 최적화):
 - AEB_SPEED_COEFF_A = -275      (속도² 계수: -0.0275)
 - AEB_SPEED_COEFF_B = 64964     (속도¹ 계수: 6.4964)
 - AEB_SPEED_COEFF_C = -1126429  (상수항: -112.6429)
 - AEB_SPEED_DIVIDER = 10000     (정수 연산을 위한 스케일링)

 제동 로직 설명:
 1. 현재 속도로 예상 제동거리 계산
 2. 전방 물체와의 거리가 (예상 제동거리 + 10cm)보다 작아지면 제동 시작
 3. 결과: 전방 물체로부터 정확히 10cm 거리에서 정지

 검증 결과:
 - 속도 구간 0 ~ 20 에서는 RC카 차량 주행
 - 모든 속도 구간(30~100km/h)에서 안정적 성능
 - 기존 단순 이차함수 모델 대비 정확도 크게 향상
 - 실제 제동 패턴을 정확히 반영하는 완전이차 모델 적용
 - 정수 연산으로 임베디드 시스템에 최적화

 주의사항:
 - 모델 적용 범위: 30km/h 이상
 - 저속 구간 0~20km/h는 RC카 한계로 미검증
 - AEB_EMERGENCY_THRESHOLD_MM = 100 (목표 정지거리 10cm)
 - 이후 RC카 추가적인 무게 변화, 센서 오차나 환경 변화 시 계수값 조정해야 할 수 있음

 데이터 수집 조건:
 - 측정 환경: RC카 기반 실제 테스트
 - 속도 범위: 30~100km/h (10km/h 간격)
 - 테스트 제외 구간: 0~20km/h (RC카 특성상 저속에서 움직이지 않아 측정 불가)
 - 측정 방식: 제동 시작/종료 시점의 전방 물체 거리 측정
 - 실제 제동거리 = 시작거리 - 종료거리
 */

static volatile bool aeb_state = false;

static bool AEB_IsEmergencyBrakingRequired (const ToFData_t *tof_latest_data,
        const MotorControllerData_t *motor_controller_latest_data)
{
    int distance_mm = (int) (tof_latest_data->distance_m * 1000);

    int motorA_speed = motor_controller_latest_data->motorChA_speed;
    int motorB_speed = motor_controller_latest_data->motorChB_speed;

    int vehicle_speed = (motorA_speed + motorB_speed) / 2;

    // 실측 데이터 기반 제동거리 계산 (완전이차 모델: ax²+bx+c)
    // braking_distance = (AEB_SPEED_COEFF_A * speed² + AEB_SPEED_COEFF_B * speed + AEB_SPEED_COEFF_C) / AEB_SPEED_DIVIDER
    int speed_squared = vehicle_speed * vehicle_speed;
    int numerator = AEB_SPEED_COEFF_A * speed_squared + AEB_SPEED_COEFF_B * vehicle_speed + AEB_SPEED_COEFF_C;
    unsigned int braking_distance_mm = (unsigned int) (numerator / AEB_SPEED_DIVIDER);

    // 1순위: 10cm + 허용오차 이하면 무조건 긴급제동
    if (distance_mm <= AEB_EMERGENCY_THRESHOLD_MM + AEB_TOLERANCE_MM)
        return true;

    /* -100 <= vehicle_speed <= 100, 제자리 회전 안됨 */
    if (vehicle_speed <= 0)
        return false;

    // 제동 조건: 제동 후 목표 정지거리(10cm)에 정확히 멈추도록
    // distance_mm = braking_distance_mm + AEB_EMERGENCY_THRESHOLD_MM 일 때 제동 시작
    if (distance_mm <= (braking_distance_mm + AEB_EMERGENCY_THRESHOLD_MM))
        return true;

    return false;
}

bool AEB_UpdateState (const ToFData_t *tof_latest_data, const MotorControllerData_t *motor_controller_latest_data)
{
    aeb_state = AEB_IsEmergencyBrakingRequired(tof_latest_data, motor_controller_latest_data);
    return true;
}

bool AEB_GetState (void)
{
    return aeb_state;
}

void AEB_GetResult (int *motor_x, int *motor_y, int64_t *emerAlert_cycle_ms)
{
    if (aeb_state)
    {
        if (*motor_y > MOTOR_STOP) // If it moves forward
        {
            *motor_x = *motor_y = MOTOR_STOP;
        }
        *emerAlert_cycle_ms = EMER_ALERT_SLOW;
    }
    else
    {
        *emerAlert_cycle_ms = EMER_ALERT_OFF;
    }
}
