#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_attr.h"

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
#define LED_STRIP_GPIO 16 //freenove ESP32 wroom onboard fullcolorLED
#define RMT_CHANNEL 0

//入力ピン指定
#define SW0 0 //esp32 onboard SW
//センサ入力設定
//フォトリフレクタ(反射光センサ)
#define sensor_Photoreflector_left_outer 4
#define sensor_Photoreflector_left_inner 17
#define sensor_Photoreflector_center 5
#define sensor_Photoreflector_right_inner 18
#define sensor_Photoreflector_right_outer 19

//ロータリーエンコーダ
#define sensor_encoder_A_C1 21
#define sensor_encoder_A_C2 22
#define sensor_encoder_B_C1 23
#define sensor_encoder_B_C2 32


//bit列に記憶
#define GPIO_OUTPUT_PIN ((1ULL << MotorA_IN1)|(1ULL << MotorA_IN2)|(1ULL << MotorB_IN1)|(1ULL << MotorB_IN2)|(1ULL << STBY)|(1ULL << LED))
#define GPIO_INPUT_PIN ((1ULL << SW0)|(1ULL << sensor_encoder_A_C1)|(1ULL << sensor_encoder_A_C2)|(1ULL << sensor_encoder_B_C1)|(1ULL << sensor_encoder_B_C2)|(1ULL << sensor_Photoreflector_center)|(1ULL << sensor_Photoreflector_left_inner)|(1ULL << sensor_Photoreflector_left_outer)|(1ULL << sensor_Photoreflector_right_inner)|(1ULL << sensor_Photoreflector_right_outer))

#define PWM_FREQ_HZ 20000 //PWM周波数
#define PERIOD_TICKS 1000 // mcpwm_timer_config.resolution_hz/PERIOD_TICKSでPWM1周期の時間→1000カウント中、duty比に応じた時間ON/OFFを実行→PWM信号の分解能は1000段階
//ボタン長押しの時間
#define LONG_PRESS_MS 800

#define STALL_TIMEOUT_MS 300
#define RESTART_PAUSE_MS 100

static uint32_t previous_left_pulse = 0;
static uint32_t previous_right_pulse = 0;

static int64_t last_left_rotation_ms = 0;
static int64_t last_right_rotation_ms = 0;

static bool left_motor_stalled = false;
static bool right_motor_stalled = false;
static bool encoder_monitor_started = false;

void update_encoder_stall_monitor(bool left_should_rotate,bool right_should_rotate);
void reset_encoder_stall_monitor(void);

static bool restart_waiting = false;
static int64_t restart_started_ms = 0;
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

    gpio_config_t sw0_conf = {
    .pin_bit_mask = (1ULL << SW0),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sw0_conf);
    
    gpio_config_t encoder_conf = {
    .pin_bit_mask =
        (1ULL << sensor_encoder_A_C1) |
        (1ULL << sensor_encoder_A_C2) |
        (1ULL << sensor_encoder_B_C1) |
        (1ULL << sensor_encoder_B_C2),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&encoder_conf);
}
static volatile uint32_t left_pulse_count = 0;
static volatile uint32_t right_pulse_count = 0;

static void IRAM_ATTR left_encoder_isr(void *arg)
{
    left_pulse_count++;
}

static void IRAM_ATTR right_encoder_isr(void *arg)
{
    right_pulse_count++;
}
// センサが黒線を検出したときの値がLOWなら 0、HIGHなら 1。
#define LINE_DETECTED_LEVEL 1

static led_strip_handle_t led_strip;

static const char *TAG = "main";

typedef enum {
    CAR_STOPPED, CAR_MODE_SELECT, CAR_RUNNING
} car_state_t;
static car_state_t car_state = CAR_STOPPED;
static int selected_mode = 1;
bool sw0_is_pressed(void)
{
    return gpio_get_level(SW0) == 0;
}
void blink_mode_purple(led_strip_handle_t strip, int count);
void update_mode_indicator(void);
#define MODE_INDICATOR_INTERVAL_MS 2000
static int64_t last_mode_activity_ms = 0;
//スイッチ押下判定
static bool was_pressed = false;
//押下された時間
static int64_t pressed_at_ms = 0;
void handle_sw0(void)
{
    bool pressed = sw0_is_pressed();
    int64_t now_ms = esp_timer_get_time() / 1000;

    // 押し始め
    if (pressed && !was_pressed) {
        pressed_at_ms = now_ms;
    }

    // 離した瞬間に、短押し／長押しを確定する
    if (!pressed && was_pressed) {
        int64_t released_ms = now_ms - pressed_at_ms;
        bool is_long_press = released_ms >= LONG_PRESS_MS;

        switch (car_state) {
        case CAR_STOPPED:
            if (is_long_press) {
                car_state = CAR_MODE_SELECT;
                last_mode_activity_ms = now_ms;
                // LEDを「モード選択中」の色へ
                // led_strip_set_pixel(ハンドル, LEDの番号, 赤, 緑, 青) ※各0〜255
                led_strip_set_pixel(led_strip, 0, 0, 0, 100);
                led_strip_refresh(led_strip);
                ESP_LOGI(TAG, "STATE MODE_SELECT");
            } else {
                car_state = CAR_RUNNING;
                // LEDを「走行中」の色へ
                // led_strip_set_pixel(ハンドル, LEDの番号, 赤, 緑, 青) ※各0〜255
                led_strip_set_pixel(led_strip, 0, 0, 100, 0);
                led_strip_refresh(led_strip);
                ESP_LOGI(TAG, "STATE RUNNING");
            }
            
            break;

        case CAR_MODE_SELECT:
            if (is_long_press) {
                // 現在の selected_mode を確定
                car_state = CAR_STOPPED;
                // LEDを「停止」の色へ
                led_strip_set_pixel(led_strip, 0, 100, 0, 0);
                led_strip_refresh(led_strip);
                ESP_LOGI(TAG, "STATE STOPPED");
            } else {
                // 短押しごとにモードを切替
                selected_mode++;
                if (selected_mode >= 5) {
                    selected_mode = 1;
                }
                last_mode_activity_ms = now_ms;
                ESP_LOGI(TAG, "MODE: %d",selected_mode);
                blink_mode_purple(led_strip, selected_mode);
            }
            break;

        case CAR_RUNNING:
            if (!is_long_press) {
                car_state = CAR_STOPPED;
                // 安全のためブレーキまたは停止
                set_motorA_state(MOTOR_STOP);
                set_motorB_state(MOTOR_STOP);
                // LEDを「停止」の色へ
                led_strip_set_pixel(led_strip, 0, 100, 0, 0);
                led_strip_refresh(led_strip);
                ESP_LOGI(TAG, "STATE STOPPED");
            }
            break;
        }
    }

    was_pressed = pressed;
}
void set_motor_speed(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB,
    uint32_t left_duty,
    uint32_t right_duty);
static bool last_turn_was_left = true;


void run_mode1(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB);
void run_mode2(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB);

static int applied_left_duty = 0;
static int applied_right_duty = 0;

void app_main() {
    //初期化処理
    ESP_LOGI(TAG, "initialize GPIO");
    initialize_gpio_pins();
    gpio_set_intr_type(sensor_encoder_A_C1, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(sensor_encoder_B_C1, GPIO_INTR_POSEDGE);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        sensor_encoder_A_C1, left_encoder_isr, NULL));

    ESP_ERROR_CHECK(gpio_isr_handler_add(
        sensor_encoder_B_C1, right_encoder_isr, NULL));
    set_motorA_state(MOTOR_STOP);
    set_motorB_state(MOTOR_STOP);
    gpio_set_level((STBY),1);
    
    //MCPWM関連設定、生成
    //1,MCPWM タイマの作成
    ESP_LOGI(TAG, "create timer");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .resolution_hz = PWM_FREQ_HZ * PERIOD_TICKS,//モータ周波数*分解能=制御周波数
        .period_ticks = PERIOD_TICKS
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config,&timer));
    ESP_LOGI(TAG, "timer created");
    
    //2,MCPWM オペレータの設定
    ESP_LOGI(TAG, "create operator");
    mcpwm_oper_handle_t operA = NULL;
    mcpwm_operator_config_t operatorA_config = { .group_id = 0 };
    mcpwm_new_operator(&operatorA_config, &operA);
    ESP_LOGI(TAG, "connect operator");
    mcpwm_operator_connect_timer(operA, timer);

    mcpwm_oper_handle_t operB = NULL;
    mcpwm_operator_config_t operatorB_config = { .group_id = 0 };
    mcpwm_new_operator(&operatorB_config, &operB);
    ESP_LOGI(TAG, "connect operator");
    mcpwm_operator_connect_timer(operB, timer);

    //3,MCPWM コンパレータの設定
    ESP_LOGI(TAG, "create comparator");
    mcpwm_cmpr_handle_t cmprA = NULL;
    mcpwm_comparator_config_t cmprA_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operA,&cmprA_config, &cmprA);

    mcpwm_cmpr_handle_t cmprB = NULL;
    mcpwm_comparator_config_t cmprB_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operB,&cmprB_config, &cmprB);

    //4,MCPWM ジェネレータの設定
    ESP_LOGI(TAG, "create generator");
    mcpwm_gen_handle_t genA = NULL;
    mcpwm_generator_config_t genA_config = { .gen_gpio_num = PWM_A};
    mcpwm_new_generator(operA,&genA_config,&genA);

    mcpwm_gen_handle_t genB = NULL;
    mcpwm_generator_config_t genB_config = { .gen_gpio_num = PWM_B};
    mcpwm_new_generator(operB,&genB_config,&genB);

    //5,波形生成ルールの設定
    
    //5-1,タイマイベント設定(ジェネレータA,タイマイベント(カウントアップ中,カウンタ値0の瞬間,ジェネレータは出力信号をHIGHに));
    ESP_LOGI(TAG, "configure timer event");
    mcpwm_generator_set_action_on_timer_event(genA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_timer_event(genB,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH));
    //5-2,コンパレータ一致時イベント設定
    ESP_LOGI(TAG, "configure compare event");
    mcpwm_generator_set_action_on_compare_event(genA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmprA,MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_compare_event(genB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmprB,MCPWM_GEN_ACTION_LOW));
    //6,タイマスタート
    ESP_LOGI(TAG, "enable timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
     ESP_LOGI(TAG, "start timer");
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
    
    //オンボードフルカラーLED(WS2812)各種設定
    ESP_LOGI(TAG,"Onboard LED initializing");
    //1.LEDストリップの設定
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO,
        .max_leds = 1,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .led_model = LED_MODEL_WS2812
    };
    //2.RMT(周辺機能)の設定
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10*1000*1000,
    };
    //3.LEDデバイスの作成
    esp_err_t ret = led_strip_new_rmt_device(&strip_config,&rmt_config,&led_strip);
    if(ret != ESP_OK){
        ESP_LOGE(TAG,"LED strip make failed");
        return;
    }
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "application started");
    led_strip_set_pixel(led_strip,0,100,0,0);
    led_strip_refresh(led_strip);
    while (1){
        handle_sw0();
        update_mode_indicator();
        if (left_motor_stalled) {
            ESP_LOGW(TAG, "LEFT MOTOR STALLED");
}

if (right_motor_stalled) {
    ESP_LOGW(TAG, "RIGHT MOTOR STALLED");
}
        if(car_state == CAR_RUNNING){
            switch (selected_mode) {
            case 1:
                run_mode1(cmprA, cmprB);
                break;
            
            case 2:
                run_mode2(cmprA,cmprB);
                break;
            case 3:
                set_motorA_state(MOTOR_FORWARD);
                set_motorB_state(MOTOR_BACKWARD);
                set_motor_speed(cmprA, cmprB, 900, 900);
                break;
            default:
                set_motorA_state(MOTOR_STOP);
                set_motorB_state(MOTOR_STOP);
                break;
            }
        } else {
            set_motorA_state(MOTOR_STOP);
            set_motorB_state(MOTOR_STOP);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        /*
        int state_sensor_c = gpio_get_level(sensor_Photoreflector_center);
        if(state_sensor_c == 1){
            ESP_LOGI(TAG, "sensor_HIGH");
            }else{
            ESP_LOGI(TAG, "sensor LOW");
        }
            
            vTaskDelay(pdMS_TO_TICKS(100));
        */
        /*
         // --- 赤色に点灯 ---
        ESP_LOGI(TAG, "LED: RED");
        // led_strip_set_pixel(ハンドル, LEDの番号, 赤, 緑, 青) ※各0〜255
        led_strip_set_pixel(led_strip, 0, 100, 0, 0); 
        led_strip_refresh(led_strip);                 // 実際にLEDに反映
        vTaskDelay(pdMS_TO_TICKS(1000));              // 1秒待つ

        // --- 緑色に点灯 ---
        ESP_LOGI(TAG, "LED: GREEN");
        led_strip_set_pixel(led_strip, 0, 0, 100, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // --- 青色に点灯 ---
        ESP_LOGI(TAG, "LED: BLUE");
        led_strip_set_pixel(led_strip, 0, 0, 0, 100);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        set_motorA_state(MOTOR_FORWARD);
        set_motorB_state(MOTOR_FORWARD);
        ESP_LOGI(TAG, "motor start!!");
        for(uint32_t duty = 500; duty < PERIOD_TICKS; duty++){
            mcpwm_comparator_set_compare_value(cmprA,duty);
            mcpwm_comparator_set_compare_value(cmprB,duty);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        ESP_LOGI(TAG, "motor stop!!");
        set_motorA_state(MOTOR_BRAKE);
        set_motorB_state(MOTOR_BRAKE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    */
   }
}
void set_motor_speed(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB,
    uint32_t left_duty,
    uint32_t right_duty)
{
    mcpwm_comparator_set_compare_value(cmprA, left_duty);
    mcpwm_comparator_set_compare_value(cmprB, right_duty);
}

void blink_mode_purple(led_strip_handle_t strip, int count)
{
    for (int i = 0; i < count; i++) {
        // 紫を点灯
        led_strip_set_pixel(strip, 0, 80, 0, 80);
        led_strip_refresh(strip);
        vTaskDelay(pdMS_TO_TICKS(200));

        // 消灯
        led_strip_clear(strip);
        led_strip_refresh(strip);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // 点滅後は、モード選択中を示す青に戻す
    led_strip_set_pixel(strip, 0, 0, 0, 100);
    led_strip_refresh(strip);
}
void update_mode_indicator(void)
{
    if (car_state != CAR_MODE_SELECT) {
        return;
    }

    int64_t now_ms = esp_timer_get_time() / 1000;

    if (now_ms - last_mode_activity_ms >= MODE_INDICATOR_INTERVAL_MS) {
        // 現在のモード番号ぶん紫点滅
        blink_mode_purple(led_strip, selected_mode );

        // 次の点滅までの待ち時間を数え直す
        last_mode_activity_ms = esp_timer_get_time() / 1000;
    }
}

#define MODE1_BASE_DUTY 800
#define MODE1_KP 350
#define MODE1_MIN_DUTY 500
#define MODE1_RAMP_UP_STEP 30     // 10 msごとに増やすPWM値
#define MODE1_RAMP_DOWN_STEP 60   // 減速時は少し速く下げる

static bool searching_line = false;
static int64_t search_started_ms = 0;


static int ramp_duty(int current, int target)
{
    if (current < target) {
        current += MODE1_RAMP_UP_STEP;

        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= MODE1_RAMP_DOWN_STEP;

        if (current < target) {
            current = target;
        }
    }

    return current;
}
void run_mode1(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB){
    int right_duty;
    int left_duty;
    static float last_line_error = 0.0f;
    // 黒線を検出したセンサーを読む
    bool left_detected =
        gpio_get_level(sensor_Photoreflector_left_inner)
        == LINE_DETECTED_LEVEL;
    bool left_outer_detected = 
        gpio_get_level(sensor_Photoreflector_left_outer)
        == LINE_DETECTED_LEVEL;
    bool center_detected =
        gpio_get_level(sensor_Photoreflector_center)
        == LINE_DETECTED_LEVEL;

    bool right_detected =
        gpio_get_level(sensor_Photoreflector_right_inner)
        == LINE_DETECTED_LEVEL;
    bool right_outer_detected =
        gpio_get_level(sensor_Photoreflector_right_outer)
        == LINE_DETECTED_LEVEL;

    //黒検知の数
    int detected_count = left_detected + center_detected + right_detected+ left_outer_detected+ right_outer_detected;
    //中央からのずれ
    float error;
    if (detected_count == 0) {
        //線を検知していない
        if (last_line_error >= 0.0f) {
        // 右へ旋回
        set_motorA_state(MOTOR_FORWARD);
        set_motorB_state(MOTOR_BACKWARD);
        set_motor_speed(cmprA, cmprB, 950, 400);
        } else {
            // 左へ旋回
            set_motorA_state(MOTOR_BACKWARD);
            set_motorB_state(MOTOR_FORWARD);
            set_motor_speed(cmprA, cmprB, 400, 950);
        }

        
        return;
        /*
        int64_t now_ms = esp_timer_get_time() / 1000;
        if (!searching_line) {
            // 線を見失った瞬間だけ初期化
            reset_encoder_stall_monitor();
            searching_line = true;
            search_started_ms = now_ms;
            if(last_line_error >= 0.0f){
                //最後に線を検知したのは右or初期状態
                set_motorA_state(MOTOR_FORWARD);
                set_motorB_state(MOTOR_BACKWARD);
                set_motor_speed(cmprA, cmprB, 800, 0);
            } else{
                //最後に線を検知したのが左
                set_motorA_state(MOTOR_BACKWARD);
                set_motorB_state(MOTOR_FORWARD);
                set_motor_speed(cmprA, cmprB, 0,800);
            }
            return;
        }
        int64_t elapsed_ms = now_ms - search_started_ms;

        // 80 ms後に両輪を700で回し始める
        if (elapsed_ms < 80) {                
            return;
        }

        // 先に回していた外側の車輪は、800を維持して少しずつ上げる
        int outer_duty = 800 + (int)((elapsed_ms - 80) * 1.0f);
        if (outer_duty > 950) {
            outer_duty = 950;
        }
        // 内側の車輪だけを、0からゆっくり始動させる
        int inner_duty = (int)((elapsed_ms - 80) * 8.0f);
        if (inner_duty > 850) {
            inner_duty = 850;
        }
        if (last_line_error >= 0.0f) {
            // 右探索：左が外側、右が内側
            set_motorA_state(MOTOR_FORWARD);
            set_motorB_state(MOTOR_BACKWARD);
            set_motor_speed(cmprA, cmprB, outer_duty, inner_duty);

            update_encoder_stall_monitor(
                outer_duty >= MODE1_MIN_DUTY,
                inner_duty >= MODE1_MIN_DUTY
            );
        }else {
        // 左探索：右が外側、左が内側
        set_motorA_state(MOTOR_BACKWARD);
        set_motorB_state(MOTOR_FORWARD);
        set_motor_speed(cmprA, cmprB, inner_duty, outer_duty);
        
        update_encoder_stall_monitor(
            inner_duty >= MODE1_MIN_DUTY,
            outer_duty >= MODE1_MIN_DUTY
        );
        }
        return;
        */
    }else if(detected_count > 0){
        searching_line = false;
        //線を検知したのでP制御
        //左だけ検出 -> -1.0
        //中央 -> 0.0
        //右だけ -> 1.0
        // 左+中央検出    -> -0.5
        // 中央+右検出    -> +0.5

        error = (
            (left_detected ? -1.0f : 0.0f)
            + (right_detected ? 1.0f : 0.0f)
            + (left_outer_detected ? -1.5f : 0.0f)
            + (right_outer_detected ? 1.5f : 0.0f)
        ) / detected_count;
        if (error != 0.0f) {
            last_line_error = error;
        }
        //P制御の補正値
        int correction = (int)(MODE1_KP * error);
        //補正値を追加
        left_duty = MODE1_BASE_DUTY + correction;
        right_duty = MODE1_BASE_DUTY - correction;

        if (left_duty < MODE1_MIN_DUTY)left_duty = MODE1_MIN_DUTY;
        if(right_duty < MODE1_MIN_DUTY)right_duty = MODE1_MIN_DUTY; 
        if (left_duty > PERIOD_TICKS) left_duty = PERIOD_TICKS;
        if (right_duty > PERIOD_TICKS) right_duty = PERIOD_TICKS; 

        int64_t now_ms = esp_timer_get_time() / 1000;
        // 片輪でも止まったら、一度PWMを0にして再始動準備
        if ((left_motor_stalled || right_motor_stalled) && !restart_waiting) {
            applied_left_duty = 0;
            applied_right_duty = 0;
            restart_waiting = true;
            restart_started_ms = now_ms;

            reset_encoder_stall_monitor();

            set_motorA_state(MOTOR_FORWARD);
            set_motorB_state(MOTOR_FORWARD);
            set_motor_speed(cmprA, cmprB, 0, 0);
            return;
        }
        // 100 msだけ無通電にしてから、通常の段階加速へ戻る
        if (restart_waiting) {
            if (now_ms - restart_started_ms < RESTART_PAUSE_MS) {
                return;
              }
            restart_waiting = false;
        }
        // 目標PWMへ段階的に近づける
        applied_left_duty = ramp_duty(applied_left_duty, left_duty);
        applied_right_duty = ramp_duty(applied_right_duty, right_duty);
        set_motorA_state(MOTOR_FORWARD);
        set_motorB_state(MOTOR_FORWARD);
        set_motor_speed(cmprA, cmprB, applied_left_duty, applied_right_duty);
        update_encoder_stall_monitor(applied_left_duty >= MODE1_MIN_DUTY,applied_right_duty >= MODE1_MIN_DUTY);
    }

}

void run_mode2(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB)
{
    int center = gpio_get_level(sensor_Photoreflector_center);
    bool line_detected = (center == LINE_DETECTED_LEVEL);

    set_motorA_state(MOTOR_FORWARD);
    set_motorB_state(MOTOR_FORWARD);

    if (line_detected) {
        // 線を検出中：前進
        set_motor_speed(cmprA, cmprB, 700, 700);

    } else if (last_turn_was_left) {
        // 線を見失った：右に曲がって探索
        set_motor_speed(cmprA, cmprB, 900, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        last_turn_was_left = false;

    } else {
        // 次は左に曲がって探索
        set_motor_speed(cmprA, cmprB, 0, 800);
        vTaskDelay(pdMS_TO_TICKS(100));
        last_turn_was_left = true;
    }
}

void update_encoder_stall_monitor(
    bool left_should_rotate,
    bool right_should_rotate)
{
    int64_t now_ms = esp_timer_get_time() / 1000;

    // 初回は基準値だけ保存する
    if (!encoder_monitor_started) {
        previous_left_pulse = left_pulse_count;
        previous_right_pulse = right_pulse_count;
        last_left_rotation_ms = now_ms;
        last_right_rotation_ms = now_ms;
        encoder_monitor_started = true;
        return;
    }

    // 左のパルスが増えた = 左車輪は回転中
    if (left_pulse_count != previous_left_pulse) {
        previous_left_pulse = left_pulse_count;
        last_left_rotation_ms = now_ms;
    }

    // 右のパルスが増えた = 右車輪は回転中
    if (right_pulse_count != previous_right_pulse) {
        previous_right_pulse = right_pulse_count;
        last_right_rotation_ms = now_ms;
    }

    left_motor_stalled =
        left_should_rotate &&
        (now_ms - last_left_rotation_ms > STALL_TIMEOUT_MS);

    right_motor_stalled =
        right_should_rotate &&
        (now_ms - last_right_rotation_ms > STALL_TIMEOUT_MS);
}
void reset_encoder_stall_monitor(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;

    previous_left_pulse = left_pulse_count;
    previous_right_pulse = right_pulse_count;

    last_left_rotation_ms = now_ms;
    last_right_rotation_ms = now_ms;

    left_motor_stalled = false;
    right_motor_stalled = false;
    encoder_monitor_started = true;
}