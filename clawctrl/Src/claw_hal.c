#include "claw_hal.h"

#include "main.h"

#include "stdint.h"
#include "string.h"

uint8_t read_btn1() {
    uint8_t st1 = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET;
    uint8_t st2 = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET;
    uint8_t st3 = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET;
    uint8_t st4 = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET;
    uint8_t st5 = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) == GPIO_PIN_RESET;
    return st1 && st2 && st3 && st4 && st5;
}

uint8_t read_btn2() {
    uint8_t st1 = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET;
    uint8_t st2 = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET;
    uint8_t st3 = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET;
    uint8_t st4 = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET;
    uint8_t st5 = HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) == GPIO_PIN_RESET;
    return st1 && st2 && st3 && st4 && st5;
}

uint8_t read_up_ls() {
    uint8_t st1 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
    uint8_t st2 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
    uint8_t st3 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
    uint8_t st4 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
    uint8_t st5 = HAL_GPIO_ReadPin(LS5_GPIO_Port, LS5_Pin) == GPIO_PIN_RESET;
    return st1 && st2 && st3 && st4 && st5;
}

uint8_t read_down_ls() {
    uint8_t st1 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin) == GPIO_PIN_RESET;
    uint8_t st2 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin) == GPIO_PIN_RESET;
    uint8_t st3 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin) == GPIO_PIN_RESET;
    uint8_t st4 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin) == GPIO_PIN_RESET;
    uint8_t st5 = HAL_GPIO_ReadPin(LS3_GPIO_Port, LS3_Pin) == GPIO_PIN_RESET;
    return st1 && st2 && st3 && st4 && st5;
}

void set_btn1_led(uint8_t on) {
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void set_btn2_led(uint8_t on) {
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void set_user_leds(uint8_t on1, uint8_t on2, uint8_t on3) {
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, on1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, on3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void beep(uint32_t times, uint32_t period) {
    for (int i = 0; i < times; i++) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_Delay(period);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        HAL_Delay(period);
    }
}
