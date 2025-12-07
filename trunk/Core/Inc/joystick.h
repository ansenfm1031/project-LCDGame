#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "stm32f4xx_hal.h"  // HAL 구조체 사용을 위해 포함

//#define ADC_REGULAR_RANK_1             ((uint32_t)0x00000001U)
//조이스틱 관련 설정과 상태를 저장하는 구조체
typedef struct {
	ADC_HandleTypeDef* hadc_x;         // X축 조이스틱이 연결된 ADC 핸들
	uint32_t adc_channel_x;            // X축 ADC 채널 번호

	ADC_HandleTypeDef* hadc_y;         // Y축 조이스틱이 연결된 ADC 핸들
	uint32_t adc_channel_y;            // Y축 ADC 채널 번호

	GPIO_TypeDef* button_gpio_port;    // 버튼이 연결된 GPIO 포트 (예: GPIOC)
	uint16_t button_gpio_pin;          // 버튼 핀 번호 (예: GPIO_PIN_13)

	uint16_t x_value;                  // 최근 읽은 X축 ADC 값 (0~4095)
	uint16_t y_value;                  // 최근 읽은 Y축 ADC 값 (0~4095)
	GPIO_PinState button_state;        // 최근 읽은 버튼 상태 (SET or RESET)

	uint32_t last_button_tick;         // 디바운싱을 위한 마지막 버튼 상태 변화 시간 (HAL_GetTick 기준)
}Joystick_HandleTypeDef;

//조이스틱의 현재 읽은 raw값들을 저장하는 구조체
typedef struct {
	uint16_t x;                        // X축 ADC 값
	uint16_t y;                        // Y축 ADC 값
	GPIO_PinState button;             // 버튼 상태
}Joystick_Data_t;

//조이스틱의 이동 이력 및 좌표를 추적하는 구조체
typedef struct {
	// X축 이동 히스토리 (bit 시프트 기반)
	uint8_t x_plus_history;          
	uint8_t x_minus_history;
	// Y축 이동 히스토리 
	uint8_t y_plus_history;          
	uint8_t y_minus_history;
	
	int8_t x_pos;                    // X축 좌표 (움직임 누적 결과)
	int8_t y_pos;                    // Y축 좌표
}Joystick_Tracker_t;

// 조이스틱 초기화 함수
void Joystick_Init(Joystick_HandleTypeDef* hjs,
                   ADC_HandleTypeDef* hadc_x, uint32_t ch_x,
                   ADC_HandleTypeDef* hadc_y, uint32_t ch_y,
                   GPIO_TypeDef* btn_port, uint16_t btn_pin);

// 조이스틱에서 ADC 및 버튼 상태를 polling 방식으로 읽어 반환
Joystick_Data_t Joystick_Read(Joystick_HandleTypeDef* hjs, uint16_t adc_timeout);

// 디바운싱을 고려하여 버튼 상태를 읽는 함수
GPIO_PinState Joystick_GetButton(Joystick_HandleTypeDef* hjs, uint16_t debounce_ms);

// X, Y 이동 이력을 기반으로 좌표를 갱신하는 추적 함수
void Joystick_Track(Joystick_Tracker_t* tracker, Joystick_Data_t* data,
                    uint16_t high_threshold, uint16_t low_threshold);


#endif