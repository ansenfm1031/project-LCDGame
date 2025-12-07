#include "buzzer.h"

// 특정 주파수(freq)의 소리를 duration_ms 밀리초 동안 PWM으로 발생시키는 함수
void tone(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t freq, uint32_t duration_ms) {
    uint32_t timer_clk;  // 해당 타이머가 사용하는 클럭 주파수 (Hz)

    // 고속 타이머(TIM1 또는 TIM8)는 APB2 버스에 연결되어 있음
#ifdef TIM8
    if (htim->Instance == TIM1 || htim->Instance == TIM8)
#else
    if (htim->Instance == TIM1)
#endif
    {
        timer_clk = HAL_RCC_GetPCLK2Freq();  // APB2 클럭 주파수를 가져옴
    } else {
        // 그 외 대부분의 타이머는 APB1 버스에 연결됨
        timer_clk = HAL_RCC_GetPCLK1Freq();  // APB1 클럭 주파수 가져옴

        // APB1 프리스케일러가 1이 아니면, 실제 타이머 클럭은 2배가 됨 (RM 문서 참고)
        if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
            timer_clk *= 2;
        }
    }

    // ARR 값을 고정하고 Prescaler를 계산하는 방식
    // -> ARR이 고정되어 있으므로 PWM의 duty 조절이 단순해짐
    uint32_t arr = 1000;  // Auto-Reload Register (PWM 주기 조절용) 값 고정
    uint32_t prescaler = (timer_clk / (arr * freq)) - 1;  // 프리스케일러 계산

    // 타이머 레지스터에 계산된 값을 직접 설정
    htim->Instance->PSC = prescaler;  // 프리스케일러 설정: 타이머 클럭 속도 줄임
    htim->Instance->ARR = arr;        // PWM 주기 설정: 하나의 PWM 사이클 길이

    // 각 채널에 해당하는 CCR(Compare/Capture Register)에 듀티 설정 (여기선 50%)
    switch (channel) {
        case TIM_CHANNEL_1: htim->Instance->CCR1 = arr / 2; break;  // duty 50%
        case TIM_CHANNEL_2: htim->Instance->CCR2 = arr / 2; break;
        case TIM_CHANNEL_3: htim->Instance->CCR3 = arr / 2; break;
        case TIM_CHANNEL_4: htim->Instance->CCR4 = arr / 2; break;
    }

    // PWM 출력 시작
    HAL_TIM_PWM_Start(htim, channel);

    // duration_ms가 0보다 크면 해당 시간만큼 딜레이 후 PWM을 멈춤
    if (duration_ms > 0) {
        HAL_Delay(duration_ms);               // 소리 지속 시간만큼 기다림
        HAL_TIM_PWM_Stop(htim, channel);      // PWM 멈춤 → 소리 꺼짐
    }
}

// 소리를 멈추는 함수 (PWM Stop)
void noTone(TIM_HandleTypeDef* htim, uint32_t channel) {
    HAL_TIM_PWM_Stop(htim, channel);  // PWM 정지 → 부저에서 소리 안남
}

void buzzer_init(Buzzer_HandleTypeDef* hbuzzer,
                 TIM_HandleTypeDef* htim,
                 uint32_t channel,
                 GPIO_TypeDef* gpio_port,
                 uint16_t gpio_pin) {
    hbuzzer->htim = htim;
    hbuzzer->channel = channel;
    hbuzzer->gpio_port = gpio_port;
    hbuzzer->gpio_pin = gpio_pin;
}

void buzzer_tone(Buzzer_HandleTypeDef* hbuzzer, uint32_t freq, uint32_t duration_ms) {
    tone(hbuzzer->htim, hbuzzer->channel, freq, duration_ms);
}

void buzzer_noTone(Buzzer_HandleTypeDef* hbuzzer) {
    noTone(hbuzzer->htim, hbuzzer->channel);
}



int tank_melody[] = {
    // [A] 도입
    NOTE_C4, NOTE_G3, NOTE_E4, 0,
    NOTE_C4, NOTE_G3, NOTE_E4, 0,

    // [B] 전개
    NOTE_G4, NOTE_A4, NOTE_G4, NOTE_E4,
    NOTE_G4, NOTE_A4, NOTE_B4, 0,

    // [C] 클라이막스
    NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4,
    NOTE_E5, NOTE_D5, NOTE_C5, 0,

    NOTE_C5, NOTE_B4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_E4, NOTE_D4, 0,

    // [D] 반복 및 마무리
    NOTE_E4, NOTE_C4, NOTE_G3, 0,
    NOTE_E4, NOTE_C4, NOTE_G3, 0,

    NOTE_C4, NOTE_D4, NOTE_E4, 0,
    NOTE_C4, NOTE_D4, NOTE_G4, 0
};

int tank_noteDurations[] = {
    // [A] 도입 - 느리고 낮은 음
    400, 400, 400, 200,
    400, 400, 400, 200,

    // [B] 전개 - 긴장감 고조
    200, 200, 200, 400,
    200, 200, 400, 200,

    // [C] 클라이막스 - 빠르고 웅장한 음
    200, 200, 200, 400,
    200, 200, 400, 200,

    200, 200, 200, 400,
    200, 200, 400, 200,

    // [D] 반복 및 페이드
    400, 400, 400, 200,
    400, 400, 400, 200,

    300, 300, 300, 200,
    300, 300, 400, 400
};

int arcade_melody[] = {
    // A파트 (Main loop)
    NOTE_C5, NOTE_E5, NOTE_G5, 0,
    NOTE_C5, NOTE_E5, NOTE_A5, 0,
    NOTE_B4, NOTE_D5, NOTE_G5, 0,
    NOTE_C5, NOTE_E5, NOTE_C6, 0,

    // 반복
    NOTE_C5, NOTE_E5, NOTE_G5, 0,
    NOTE_C5, NOTE_E5, NOTE_A5, 0,
    NOTE_B4, NOTE_D5, NOTE_G5, 0,
    NOTE_E5, NOTE_D5, NOTE_C5, 0,

    // B파트 (변화/브릿지)
    NOTE_F5, NOTE_A5, NOTE_C6, 0,
    NOTE_F5, NOTE_A5, NOTE_D6, 0,
    NOTE_E5, NOTE_G5, NOTE_C6, 0,
    NOTE_B5, NOTE_A5, NOTE_G5, 0
};

int arcade_noteDurations[] = {
    // A파트
    150, 150, 300, 100,
    150, 150, 300, 100,
    150, 150, 300, 100,
    150, 150, 300, 100,

    // 반복
    150, 150, 300, 100,
    150, 150, 300, 100,
    150, 150, 300, 100,
    150, 150, 300, 100,

    // B파트
    150, 150, 300, 100,
    150, 150, 300, 100,
    150, 150, 300, 100,
    150, 150, 300, 100
};

int main_melody[] = {
    NOTE_A4, NOTE_E4, NOTE_A4, 0,
    NOTE_C5, NOTE_E4, NOTE_G4, 0,
    NOTE_A4, NOTE_E5, NOTE_A4, 0,
    NOTE_D5, NOTE_F4, NOTE_A4, 0,

    NOTE_E4, NOTE_B3, NOTE_G4, 0,
    NOTE_A4, NOTE_E5, NOTE_A4, 0,
    NOTE_C5, NOTE_D5, NOTE_E5, 0,
    NOTE_A4, 0, NOTE_A4, 0
};

int main_noteDurations[] = {
    300, 300, 600, 200,
    300, 300, 600, 200,
    300, 300, 600, 200,
    300, 300, 600, 200,

    300, 300, 600, 200,
    300, 300, 600, 200,
    300, 300, 600, 200,
    600, 200, 600, 400
};


void playTankBGM(Buzzer_HandleTypeDef* hbuzzer, uint8_t delay) {
    int length = sizeof(tank_melody) / sizeof(tank_melody[0]);

    for (int i = 0; i < length; i++) {
        if (tank_melody[i] == 0) {
            buzzer_noTone(hbuzzer);
        } else {
            buzzer_tone(hbuzzer, tank_melody[i], tank_noteDurations[i]);
        }
        HAL_Delay(delay);
    }
}

void playArcadeBGM(Buzzer_HandleTypeDef* hbuzzer, uint8_t delay) {
    int length = sizeof(arcade_melody) / sizeof(arcade_melody[0]);

    for (int i = 0; i < length; i++) {
        if (arcade_melody[i] == 0) {
            buzzer_noTone(hbuzzer);
        } else {
            buzzer_tone(hbuzzer, arcade_melody[i], arcade_noteDurations[i]);
        }
        HAL_Delay(delay);  // 리듬 살리기용 딜레이
    }
}

void playMainTheme(Buzzer_HandleTypeDef* hbuzzer, uint8_t delay) {
    int length = sizeof(main_melody) / sizeof(main_melody[0]);

    for (int i = 0; i < length; i++) {
        if (main_melody[i] == 0) {
            buzzer_noTone(hbuzzer);
        } else {
            buzzer_tone(hbuzzer, main_melody[i], main_noteDurations[i]);
        }
        HAL_Delay(delay);  // 약간의 여백
    }
}
