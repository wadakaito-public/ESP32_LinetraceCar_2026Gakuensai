#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

//出力ピン指定
//TB6612出力側指定
//MotorA(左)
#define PWM_A 27
#define MotorA_IN1 12
#define MotorA_IN2 14
//MotorB(右)
#define MotorB_IN1 26
#define MotorB_IN2 25
#define PWM_B 33
#define STBY 13 //両モータ共通

#define LED 2 //esp32 onboard LED

//入力ピン指定
#define SW0 0 //esp32 onboard SW
//センサ入力設定
//フォトリフレクタ(反射光センサ)
#define sensor_Photoreflector_left_outer 16
#define sensor_Photoreflector_left_inner 17
#define sensor_Photoreflector_center 5
#define sensor_Photoreflector_right_inner 18
#define sensor_Photoreflector_right_outer 19

//ロータリーエンコーダ
#define sensor_encoder_A_C1 36
#define sensor_encoder_A_C2 39
#define sensor_encoder_B_C1 34
#define sensor_encoder_B_C2 35


//bit列に記憶
#define GPIO_OUTPUT_PIN ((1ULL << MotorA_IN1)|(1ULL << MotorA_IN2)|(1ULL << MotorB_IN1)|(1ULL << MotorB_IN2)|(1ULL << STBY)|(1ULL << LED))
#define GPIO_INPUT_PIN ((1ULL << SW0)|(1ULL << sensor_encoder_A_C1)|(1ULL << sensor_encoder_A_C2)|(1ULL << sensor_encoder_B_C1)|(1ULL << sensor_encoder_B_C2)|(1ULL << sensor_Photoreflector_center)||(1ULL << sensor_Photoreflector_left_inner)|(1ULL << sensor_Photoreflector_left_outer)|(1ULL << sensor_Photoreflector_right_inner)|(1ULL << sensor_Photoreflector_right_outer))

#define PWM_FREQ_HZ 20000 //PWM周波数
#define PERIOD_TICKS 1000 // mcpwm_timer_config.resolution_hz/PERIOD_TICKSでPWM1周期の時間→1000カウント中、duty比に応じた時間ON/OFFを実行→PWM信号の分解能は1000段階

//モータの状態を定義する列挙型の定義
typedef enum {
    MOTOR_STOP,
    MOTOR_BRAKE,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} motor_state;

void set_motorA_state(motor_state state){
    switch (state)
    {
    case MOTOR_FORWARD:
        gpio_set_level(MotorA_IN1,1);
        gpio_set_level(MotorA_IN2,0);
        break;
    case MOTOR_BACKWARD:
        gpio_set_level(MotorA_IN1,0);
        gpio_set_level(MotorA_IN2,1);
        break;
    case MOTOR_BRAKE:
        gpio_set_level(MotorA_IN1,1);
        gpio_set_level(MotorA_IN2,1);
        break;
    case MOTOR_STOP:
        gpio_set_level(MotorA_IN1,0);
        gpio_set_level(MotorA_IN2,0);
    }
}
void set_motorB_state(motor_state state){
    switch (state)
    {
    case MOTOR_FORWARD:
        gpio_set_level(MotorB_IN1,1);
        gpio_set_level(MotorB_IN2,0);
        break;
    case MOTOR_BACKWARD:
        gpio_set_level(MotorB_IN1,0);
        gpio_set_level(MotorB_IN2,1);
        break;
    case MOTOR_BRAKE:
        gpio_set_level(MotorB_IN1,1);
        gpio_set_level(MotorB_IN2,1);
        break;
    case MOTOR_STOP:
        gpio_set_level(MotorB_IN1,0);
        gpio_set_level(MotorB_IN2,0);
    }
}

void initialize_gpio_pins(void){
    //出力ピンの反映
    //gpio_config_t gpio_configの構造体
    gpio_config_t gpio_conf = {
        .pin_bit_mask = GPIO_OUTPUT_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&gpio_conf); //書き込む
    
    //入力ピンの反映
    gpio_conf.pin_bit_mask = GPIO_INPUT_PIN;
    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&gpio_conf);
}
void app_main() {
    //初期化処理
    ESP_LOGI(__func__, "initialize GPIO");
    initialize_gpio_pins();
    set_motorA_state(MOTOR_STOP);
    set_motorB_state(MOTOR_STOP);
    gpio_set_level((STBY),1);
    
    //MCPWM関連設定、生成
    //1,MCPWM タイマの作成
    ESP_LOGI(__func__, "create timer");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .resolution_hz = PWM_FREQ_HZ * PERIOD_TICKS,//モータ周波数*分解能=制御周波数
        .period_ticks = PERIOD_TICKS
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config,&timer));
    ESP_LOGI(__func__, "timer created");
    
    //2,MCPWM オペレータの設定
    ESP_LOGI(__func__, "create operator");
    mcpwm_oper_handle_t operA = NULL;
    mcpwm_operator_config_t operatorA_config = { .group_id = 0 };
    mcpwm_new_operator(&operatorA_config, &operA);
    ESP_LOGI(__func__, "connect operator");
    mcpwm_operator_connect_timer(operA, timer);

    mcpwm_oper_handle_t operB = NULL;
    mcpwm_operator_config_t operatorB_config = { .group_id = 0 };
    mcpwm_new_operator(&operatorB_config, &operB);
    ESP_LOGI(__func__, "connect operator");
    mcpwm_operator_connect_timer(operB, timer);

    //3,MCPWM コンパレータの設定
    ESP_LOGI(__func__, "create comparator");
    mcpwm_cmpr_handle_t cmprA = NULL;
    mcpwm_comparator_config_t cmprA_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operA,&cmprA_config, &cmprA);

    mcpwm_cmpr_handle_t cmprB = NULL;
    mcpwm_comparator_config_t cmprB_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operB,&cmprB_config, &cmprB);

    //4,MCPWM ジェネレータの設定
    ESP_LOGI(__func__, "create generator");
    mcpwm_gen_handle_t genA = NULL;
    mcpwm_generator_config_t genA_config = { .gen_gpio_num = PWM_A};
    mcpwm_new_generator(operA,&genA_config,&genA);

    mcpwm_gen_handle_t genB = NULL;
    mcpwm_generator_config_t genB_config = { .gen_gpio_num = PWM_B};
    mcpwm_new_generator(operB,&genB_config,&genB);

    //5,波形生成ルールの設定
    
    //5-1,タイマイベント設定(ジェネレータA,タイマイベント(カウントアップ中,カウンタ値0の瞬間,ジェネレータは出力信号をHIGHに));
    ESP_LOGI(__func__, "configure timer event");
    mcpwm_generator_set_action_on_timer_event(genA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_timer_event(genB,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH));
    //5-2,コンパレータ一致時イベント設定
    ESP_LOGI(__func__, "configure compare event");
    mcpwm_generator_set_action_on_compare_event(genA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmprA,MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_compare_event(genB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmprB,MCPWM_GEN_ACTION_LOW));
    //6,タイマスタート
    ESP_LOGI(__func__, "enable timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
     ESP_LOGI(__func__, "start timer");
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
    
    ESP_LOGI(__func__, "application started");
    while (1){
        int state_sensor_rf_r_out = gpio_get_level(sensor_Photoreflector_right_outer);
        if(state_sensor_rf_r_out == 1){
            ESP_LOGI(__func__, "sensor_HIGH");
        }else{
            ESP_LOGI(__func__, "sensor LOW");
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        /*
        set_motorA_state(MOTOR_FORWARD);
        set_motorB_state(MOTOR_FORWARD);
        ESP_LOGI(__func__, "motor start!!");
        for(uint32_t duty = 500; duty < PERIOD_TICKS; duty++){
            mcpwm_comparator_set_compare_value(cmprA,duty);
            mcpwm_comparator_set_compare_value(cmprB,duty);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(__func__, "motor stop!!");
        set_motorA_state(MOTOR_BRAKE);
        set_motorB_state(MOTOR_BRAKE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    */
   }
}