#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "spi.h"
#include "math.h"

#include "UI.h"

#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "stdio.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

typedef struct {
    uint8_t input_scale_x; // top 6 bits: exponent
    uint8_t input_scale_y; // bottom 2 bits: 00 - 1, 01 - 2, 10 - 4, 11 - 8
    int32_t input_offset_x;
    int32_t input_offset_y;
} UI_state_t;

typedef struct {
    uint8_t *count[2];
    uint8_t mode;
    uint8_t prev;
    GPIO_TypeDef *COM_port;
    uint16_t COM_pin;
    GPIO_TypeDef *PUSH_port;
    uint16_t PUSH_pin;
    GPIO_TypeDef *A_port;
    uint16_t A_pin;
    GPIO_TypeDef *B_port;
    uint16_t B_pin;
} UI_rotary_t;

typedef struct {
    int32_t *offset[2];
    uint8_t *scale[2];
    uint32_t last_tick;
    uint32_t first_tick;
    uint8_t prev;
    GPIO_TypeDef *COM_port;
    uint16_t COM_pin;
    GPIO_TypeDef *UP_port;
    uint16_t UP_pin;
    GPIO_TypeDef *DOWN_port;
    uint16_t DOWN_pin;
    GPIO_TypeDef *LEFT_port;
    uint16_t LEFT_pin;
    GPIO_TypeDef *RIGHT_port;
    uint16_t RIGHT_pin;
    GPIO_TypeDef *CENTER_port;
    uint16_t CENTER_pin;
} UI_nav_t;

UI_state_t UI_state;

UI_rotary_t UI_input_rotary = {
    .count[0] = &UI_state.input_scale_y,
    .count[1] = &UI_state.input_scale_x,
    .prev = 0,
    .mode = 3,
    .COM_port =     INPUT_COM_GPIO_Port,
    .COM_pin =      INPUT_COM_Pin,
    .PUSH_port =    SCALE_PUSH_GPIO_Port,
    .PUSH_pin =     SCALE_PUSH_Pin,
    .A_port =       SCALE_A_GPIO_Port,
    .A_pin =        SCALE_A_Pin,
    .B_port =       SCALE_B_GPIO_Port,
    .B_pin =        SCALE_B_Pin,
};

UI_nav_t UI_input_nav = {
    .offset[0] = &UI_state.input_offset_x,
    .offset[1] = &UI_state.input_offset_y,
    .scale[0] = &UI_state.input_scale_y,
    .scale[1] = &UI_state.input_scale_x,
    .COM_port =     INPUT_COM_GPIO_Port,
    .COM_pin =      INPUT_COM_Pin,
    .UP_port =      OFFSET_UP_GPIO_Port,
    .UP_pin =       OFFSET_UP_Pin,
    .DOWN_port =    OFFSET_DOWN_GPIO_Port,
    .DOWN_pin =     OFFSET_DOWN_Pin,
    .LEFT_port =    OFFSET_LEFT_GPIO_Port,
    .LEFT_pin =     OFFSET_LEFT_Pin,
    .RIGHT_port =   OFFSET_RIGHT_GPIO_Port,
    .RIGHT_pin =    OFFSET_RIGHT_Pin,
    .CENTER_port =  OFFSET_CENTER_GPIO_Port,
    .CENTER_pin =   OFFSET_CENTER_Pin,
};

char buffer[50];

void UI_rotary_increment(UI_rotary_t *state)
{
    for (uint8_t i = 0; i < 2; i ++){
        if ((state->mode & (1 << i)) && (*(state->count[i]) < 255)) {
            (*(state->count[i])) ++;
        }
    }
}

void UI_rotary_decrement(UI_rotary_t *state)
{
    for (uint8_t i = 0; i < 2; i ++){
        if ((state->mode & (1 << i)) && (*(state->count[i]) > 0)) {
            (*(state->count[i])) --;
        }
    }
}

void UI_rotary_poll(UI_rotary_t *state)
{
    uint8_t tmp, encoder_combined; 

    HAL_GPIO_WritePin(state->COM_port, state->COM_pin, GPIO_PIN_RESET);

    tmp = HAL_GPIO_ReadPin(state->PUSH_port, state->PUSH_pin);
    if (tmp == GPIO_PIN_RESET) {
        if (!(state->prev & (1 << 2))) {
            state->mode --;
            if (state->mode == 0)
                state->mode = 3;
            state->prev |= (1 << 2);
        }
    } else {
        state->prev &= ~(1 << 2);
    }

    tmp =  (HAL_GPIO_ReadPin(state->A_port, state->A_pin) << 1) |
            HAL_GPIO_ReadPin(state->B_port, state->B_pin);

    encoder_combined = ((state->prev & 0b00000011) << 2) | tmp;

    state->prev = (state->prev & ~0b11) | tmp;

    switch (encoder_combined) {
        case 0b1000:
            UI_rotary_decrement(state);
            break;
        case 0b0100:
            UI_rotary_increment(state);
            break;
        default:
            break;
    }

    HAL_GPIO_WritePin(state->COM_port, state->COM_pin, GPIO_PIN_SET);
}

void UI_nav_poll(UI_nav_t *state)
{
    uint32_t tick = HAL_GetTick();

    HAL_GPIO_WritePin(state->COM_port, state->COM_pin, GPIO_PIN_RESET);

    if (state->prev == 0
        || ((tick - state->first_tick > 750) 
            && (tick - state->last_tick > 250) 
            && state->prev == 1)
        ) {

        if (state->prev == 0)
            state->first_tick = tick;
        state->last_tick = tick;
        
        if (!HAL_GPIO_ReadPin(state->LEFT_port, state->LEFT_pin)) {
            (*(state->offset[0])) -= (1 << (*(state->scale[1]) & 0b00000011)) * (int32_t)pow(10, (*(state->scale[1]) >> 2));
            state->prev = 1;
        } else if (!HAL_GPIO_ReadPin(state->RIGHT_port, state->RIGHT_pin)) {
            (*(state->offset[0])) += (1 << (*(state->scale[1]) & 0b00000011)) * (int32_t)pow(10, (*(state->scale[1]) >> 2));
            state->prev = 1;
        } else if (!HAL_GPIO_ReadPin(state->UP_port, state->UP_pin)) {
            (*(state->offset[1])) += (1 << (*(state->scale[0]) & 0b00000011)) * (int32_t)pow(10, (*(state->scale[0]) >> 2));
            state->prev = 1;
        } else if (!HAL_GPIO_ReadPin(state->DOWN_port, state->DOWN_pin)) {
            (*(state->offset[1])) -= (1 << (*(state->scale[0]) & 0b00000011)) * (int32_t)pow(10, (*(state->scale[0]) >> 2));
            state->prev = 1;
        } else {
            state->prev = 0;
        }
    }    

    if (!HAL_GPIO_ReadPin(state->CENTER_port, state->CENTER_pin)) {
        (*(state->offset[0])) = 0;
        (*(state->offset[1])) = 0;
    }

    HAL_GPIO_WritePin(state->COM_port, state->COM_pin, GPIO_PIN_SET);
}

void OLED_Write(uint8_t input_scale_x, uint8_t input_scale_y)
{   
    ssd1306_Fill(White);
    ssd1306_SetCursor(6, 21);
    sprintf(buffer, "Scale X: %li/div", (1 << (UI_state.input_scale_x & 0b00000011)) * (int32_t)pow(10, (UI_state.input_scale_x >> 2)));
    ssd1306_WriteString(buffer, Font_6x8, Black);
    portYIELD();
    ssd1306_SetCursor(6, 35);
    sprintf(buffer, "Scale Y: %li/div", (1 << (UI_state.input_scale_y & 0b00000011)) * (int32_t)pow(10, (UI_state.input_scale_y >> 2)));
    ssd1306_WriteString(buffer, Font_6x8, Black);
    portYIELD();
    ssd1306_UpdateScreen();
    portYIELD();
}

void UI(void *pvParameters)
{

    static uint8_t state; 
    static uint8_t encoder_current;
    static uint8_t encoder_combined;

    uint32_t prev_tick, tick;

    HAL_GPIO_WritePin(GPIOC, OFFSET_COM_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOF, INPUT_COM_Pin, GPIO_PIN_SET);

    while (1) {

        UI_rotary_poll(&UI_input_rotary);

        UI_nav_poll(&UI_input_nav);

        uint8_t led = ~((UI_input_rotary.mode << 6) & 0b11000000);
        HAL_SPI_Transmit(&hspi3, &led, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOC, UI_RCLK_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, UI_RCLK_Pin, GPIO_PIN_RESET);

        OLED_Write(UI_state.input_scale_x, UI_state.input_scale_y);

        vTaskDelay(10/portTICK_PERIOD_MS);


        /*
        dpad2_up = HAL_GPIO_ReadPin(GPIOC, OFFSET_UP_Pin);
        dpad2_down = HAL_GPIO_ReadPin(GPIOB, OFFSET_DOWN_Pin);
        dpad2_left = HAL_GPIO_ReadPin(GPIOB, OFFSET_LEFT_Pin);
        dpad2_right = HAL_GPIO_ReadPin(GPIOC, OFFSET_RIGHT_Pin);
        dpad2_button = HAL_GPIO_ReadPin(GPIOC, OFFSET_CENTER_Pin);

        HAL_GPIO_WritePin(GPIOC, OFFSET_COM_Pin, GPIO_PIN_SET);
        osDelay(1);

        
        HAL_GPIO_WritePin(GPIOF, INPUT_COM_Pin, GPIO_PIN_RESET);
        a2 = HAL_GPIO_ReadPin(GPIOC, SCALE_A_Pin);
        b2 = HAL_GPIO_ReadPin(GPIOC, SCALE_B_Pin);
        encoder2_button_state = HAL_GPIO_ReadPin(GPIOC, SCALE_PUSH_Pin);

        if(a2 != last_a_encoder2) {
            encoder2_count += (b2 != a2) ? 1 : -1;
        }
        last_a_encoder2 = a2;

        
        dpad1_up = HAL_GPIO_ReadPin(GPIOC, OFFSET_UP_Pin);
        dpad1_down = HAL_GPIO_ReadPin(GPIOB, OFFSET_DOWN_Pin);
        dpad1_left = HAL_GPIO_ReadPin(GPIOB, OFFSET_LEFT_Pin);
        dpad1_right = HAL_GPIO_ReadPin(GPIOC, OFFSET_RIGHT_Pin);
        dpad1_button = HAL_GPIO_ReadPin(GPIOC, OFFSET_CENTER_Pin);

        HAL_GPIO_WritePin(GPIOF, INPUT_COM_Pin, GPIO_PIN_SET);

        
        button1_state = HAL_GPIO_ReadPin(GPIOA, PUSH_1_Pin);
        button2_state = HAL_GPIO_ReadPin(GPIOB, PUSH_2_Pin);
        button3_state = HAL_GPIO_ReadPin(GPIOB, PUSH_3_Pin);
        */
    }

    vTaskDelete(NULL);
}

