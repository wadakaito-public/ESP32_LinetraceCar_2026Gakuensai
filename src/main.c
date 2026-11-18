#include <stdio.h>
#include <stdlib.h>
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
#define PWM_A 27//実装基板では33
#define MotorA_IN1 12//実装基板では26
#define MotorA_IN2 14//実装基板では25
//MotorB(右)
#define MotorB_IN1 26//実装基板では14
#define MotorB_IN2 25//実装基板では12
#define PWM_B 33//実装基板では13
#define STBY 13 //両モータ共通//実装基板では27

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
#define RESTART_PAUSE_MS 0

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
// 検出したい線の色の値が白なら 0、黒なら 1。
#define LINE_DETECTED_LEVEL 0

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



void run_mode1(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB);
void run_mode2(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB);

void bt_log_start(void);
void bt_log(const char *format, ...);

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

    bt_log_start();

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
            restart_waiting = false;
            restart_started_ms = 0;
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


//基礎デューティ比
#define MODE1_BASE_DUTY 800
//比例ゲイン
#define MODE1_KP 300
//最低デューティ比
#define MODE1_MIN_DUTY 400
//探索用パラメータ
#define MODE1_SEARCH_TURN_MS 1000
#define MODE1_SEARCH_DUTY 900
#define MODE1_SEARCH_INNER_DUTY 500
//内輪を止め始める偏差
#define MODE1_INNER_STOP_ERROR 1.75f
//内輪を逆転し始める偏差
#define MODE1_INNER_REVERSE_ERROR 2.5f
//逆転時のpwm
#define MODE1_INNER_REVERSE_DUTY 800
//逆転時の1周期当たりPWM増加量
#define MODE1_REVERSE_RAMP_STEP 250
#define MODE1_RAMP_UP_STEP 30     // 10 msごとに増やすPWM値
#define MODE1_RAMP_DOWN_STEP 60   // 減速時は少し速く下げる
//再始動用の定数
#define RESTART_KICK_MS 300
#define RESTART_KICK_DUTY 1000
static int applied_left_duty = 0;
static int applied_right_duty = 0;
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
//モータ駆動方向＆量設定
static void apply_mode1_drive(
    mcpwm_cmpr_handle_t cmprA, mcpwm_cmpr_handle_t cmprB,
    int left, int right)
{
    if ((applied_left_duty > 0 && left < 0) ||
        (applied_left_duty < 0 && left > 0)) {
        left = 0;
    }
    if ((applied_right_duty > 0 && right < 0) ||
        (applied_right_duty < 0 && right > 0)) {
        right = 0;
    }

    set_motorA_state(left > 0 ? MOTOR_FORWARD : left < 0 ? MOTOR_BACKWARD : MOTOR_STOP);
    set_motorB_state(right > 0 ? MOTOR_FORWARD : right < 0 ? MOTOR_BACKWARD : MOTOR_STOP);
    set_motor_speed(cmprA, cmprB, abs(left), abs(right));

    applied_left_duty = left;
    applied_right_duty = right;
}
//符号あり
static int ramp_signed_duty(int current, int target)
{
    if (target == 0) {
        return 0;
    }
    if (target > 0) {
        return ramp_duty(current > 0 ? current : 0, target);
    }

    int magnitude = current < 0 ? -current : 0;//今の逆転の大きさ
    int target_magnitude = -target;//目標の大きさ

    if (magnitude < target_magnitude) {
        magnitude += MODE1_REVERSE_RAMP_STEP;
        return magnitude < target_magnitude ? -magnitude : target;
    }

    return -ramp_duty(magnitude, target_magnitude);
}
static bool searching_state = false;
static int64_t search_started_ms = 0;

static bool recovery_state = false;
static int64_t recovery_started_ms = 0;

void run_mode1(mcpwm_cmpr_handle_t cmprA,mcpwm_cmpr_handle_t cmprB){
    int right_duty;
    int left_duty;
    
    static float last_line_error = 0.0f;
    static bool restart_drive_sent = false;
    static bool restart_left_wheel = false;
    static int64_t restart_kick_started_ms = 0;
    static uint32_t restart_outer_pulse_start = 0;
    bool restart_finished = false;

    if (restart_waiting) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        int64_t kick_elapsed_ms = now_ms - restart_kick_started_ms;
        bool center_on_line =
            gpio_get_level(sensor_Photoreflector_center) == LINE_DETECTED_LEVEL;

        // 最初の100 msは駆動を維持する。
        // その後は中央でラインを検出するか、最大時間に達したら終了。
        if (kick_elapsed_ms < 100 ||
            (kick_elapsed_ms < RESTART_KICK_MS && !center_on_line)) {
            return;
        }

        if (!restart_drive_sent) {
            uint32_t left_start = left_pulse_count;
            uint32_t right_start = right_pulse_count;
            restart_outer_pulse_start =
                restart_left_wheel ? left_start : right_start;

            apply_mode1_drive(
                cmprA, cmprB,
                restart_left_wheel ? RESTART_KICK_DUTY : 500,
                restart_left_wheel ? 500 : RESTART_KICK_DUTY);

            restart_kick_started_ms = esp_timer_get_time() / 1000;
            restart_drive_sent = true;
            reset_encoder_stall_monitor();

            bt_log("BLUE t=%lld pwm L=%d R=%d enc L=%lu R=%lu\r\n",
                   (long long)restart_kick_started_ms,
                   applied_left_duty, applied_right_duty,
                   (unsigned long)left_start, (unsigned long)right_start);
            led_strip_set_pixel(led_strip, 0, 0, 0, 100);
            led_strip_refresh(led_strip);
            return;
        }

        if (now_ms - restart_kick_started_ms < RESTART_KICK_MS) {
            return;
        }

        uint32_t outer_pulse_now =
            restart_left_wheel ? left_pulse_count : right_pulse_count;

        bt_log("KICK_END t=%lld wheel=%c delta=%lu\r\n",
               (long long)now_ms, restart_left_wheel ? 'L' : 'R',
               (unsigned long)(outer_pulse_now - restart_outer_pulse_start));

        restart_waiting = false;
        restart_finished = true;
        restart_started_ms = 0;
    }
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
    static int64_t last_sensor_log_ms = 0;
    int64_t sensor_log_ms = esp_timer_get_time() / 1000;

    if (sensor_log_ms - last_sensor_log_ms >= 200) {
        last_sensor_log_ms = sensor_log_ms;
        ESP_LOGI(TAG, "count=%d detected=%d %d %d %d %d",
             detected_count, left_outer_detected, left_detected,
             center_detected, right_detected, right_outer_detected);
    }
    //検出の連続を数える
    static int consecutive_detections = 0;
    if(searching_state && detected_count > 0){
        consecutive_detections++;
    }else {
        consecutive_detections = 0;
    }
    bool line_confirmed = detected_count > 0 && (!searching_state||consecutive_detections >= 3);

    //未検出の連続を数える
    static int consecutive_misses = 0;
    if (!searching_state && !restart_waiting && !restart_finished && detected_count == 0 &&
        (applied_left_duty != 0 || applied_right_duty != 0) &&
        consecutive_misses < 10) {
        consecutive_misses++;
        return;  // 直前のモータ指令を維持する
    }
    consecutive_misses = 0;

    int64_t led_now_ms = esp_timer_get_time() / 1000;

    if (restart_started_ms > 0 && led_now_ms - restart_started_ms < 1000) {
        if(restart_drive_sent){
            led_strip_set_pixel(led_strip, 0, 0, 0, 100);  // 青：タイヤ再始動
        } else {
            led_strip_set_pixel(led_strip, 0, 100, 0, 0);  // 赤：ストールでPWMを0にした
        }
    } else if (!line_confirmed && recovery_state &&
               led_now_ms - recovery_started_ms < 500) {
        led_strip_set_pixel(led_strip, 0, 100, 100, 0 );//オレンジ：後退
    } else if (line_confirmed) {
        led_strip_set_pixel(led_strip, 0, 50, 50, 50);  // 白：P制御
    } else {
        led_strip_set_pixel(led_strip, 0, 80, 0, 80);   // 紫：探索
    }
    led_strip_refresh(led_strip);
    
    if (!line_confirmed) {
        //線を検知していない
        int64_t now_ms = esp_timer_get_time() / 1000;
        // 線を見失った瞬間の時刻を記録
        if (!searching_state) {
            //未検出かつ探索状態ではない＝これから探索を始める
            searching_state = true;
            search_started_ms = now_ms;
            reset_encoder_stall_monitor();
        }
        if ((left_motor_stalled || right_motor_stalled) && !recovery_state) {
            recovery_state = true;
            recovery_started_ms = now_ms;
        }

        if (recovery_state) {
            int64_t recovery_elapsed_ms = now_ms - recovery_started_ms;

            // 最初の500 msは後退
            if (recovery_elapsed_ms < 500) {
                apply_mode1_drive(
                    cmprA, cmprB,
                    -MODE1_SEARCH_DUTY, -MODE1_SEARCH_DUTY);

                update_encoder_stall_monitor(true, true);
                return;
            }

            // 後退後の200 msは両輪前進で旋回し、探索を立ち上げる
            if (recovery_elapsed_ms < 700) {
                bool turn_right = last_line_error >= 0.0f;

                apply_mode1_drive(
                    cmprA, cmprB,
                    turn_right ? 1000 : 00,
                    turn_right ? 0 : 1000);

                return;
            }

            // 通常探索へ戻る時点からストール監視を再開
            recovery_state = false;
            reset_encoder_stall_monitor();
        }

        // 最後にラインを検出した側から探索を始める
        bool initial_search_right = (last_line_error >= 0.0f);
        /*
        // 600 msごとに探索方向を反転する
        int64_t elapsed_ms = now_ms - search_started_ms;
        int64_t turn_index = elapsed_ms / MODE1_SEARCH_TURN_MS;
        bool search_right = (turn_index % 2 == 0)
            ? initial_search_right
            : !initial_search_right;
        */
        
        if (initial_search_right) {
            apply_mode1_drive(cmprA, cmprB, MODE1_SEARCH_DUTY, -MODE1_SEARCH_INNER_DUTY);
        } else {
            apply_mode1_drive(cmprA, cmprB, -MODE1_SEARCH_INNER_DUTY, MODE1_SEARCH_DUTY);
        }
        update_encoder_stall_monitor(initial_search_right, !initial_search_right);
        return;
    
    } else {
        //線を検出した
        if (searching_state) {
            //直前が探索状態だったなら回転監視をリセット
            reset_encoder_stall_monitor();
        }
        //状態フラグリセット
        recovery_state = false;
        searching_state = false;
        
        //偏差を計算する
        error = (
            (left_detected ? -1.0f : 0.0f)
            + (right_detected ? 1.0f : 0.0f)
            + (left_outer_detected ? -2.5f : 0.0f)
            + (right_outer_detected ? 2.5f : 0.0f)
        ) / detected_count;
        if (error != 0.0f) {
            last_line_error = error;
        }
        //P制御の補正値
        int correction = (int)(MODE1_KP * error);
        //補正値を追加
        left_duty = MODE1_BASE_DUTY + correction;
        right_duty = MODE1_BASE_DUTY - correction;
        //上下限適用
        if (left_duty < MODE1_MIN_DUTY)left_duty = MODE1_MIN_DUTY;
        if(right_duty < MODE1_MIN_DUTY)right_duty = MODE1_MIN_DUTY; 
        if (left_duty > PERIOD_TICKS) left_duty = PERIOD_TICKS;
        if (right_duty > PERIOD_TICKS) right_duty = PERIOD_TICKS;
        //停止、逆転適用
        float abs_error = error < 0.0f ? -error : error;//絶対値を求める
        if (abs_error >=MODE1_INNER_STOP_ERROR) {
            int inner_duty = 0;
            if (abs_error >= MODE1_INNER_REVERSE_ERROR) {
                inner_duty = -MODE1_INNER_REVERSE_DUTY;
            }
            if(error > 0.0f) {
                right_duty = inner_duty;
            } else {
                left_duty = inner_duty;
            }
        }

        int64_t now_ms = esp_timer_get_time() / 1000;
        // 片輪でも止まったら、一度PWMを0にして再始動準備
        if (left_motor_stalled || right_motor_stalled) {
            restart_waiting = true;
            restart_started_ms = now_ms;
            restart_drive_sent = false;
            restart_left_wheel = last_line_error > 0.0f;

            bt_log("RED t=%lld stall L=%d R=%d pwm L=%d R=%d enc L=%lu R=%lu\r\n",
                   (long long)now_ms, left_motor_stalled, right_motor_stalled,
                   applied_left_duty, applied_right_duty,
                   (unsigned long)left_pulse_count,
                   (unsigned long)right_pulse_count);

            apply_mode1_drive(cmprA, cmprB, 0, 0);
            reset_encoder_stall_monitor();
            led_strip_set_pixel(led_strip, 0, 100, 0, 0);
            led_strip_refresh(led_strip);
            return;
        }

        int next_left = ramp_signed_duty(applied_left_duty, left_duty);
        int next_right = ramp_signed_duty(applied_right_duty, right_duty);
        apply_mode1_drive(cmprA, cmprB, next_left, next_right);

        update_encoder_stall_monitor(
            abs(applied_left_duty) >= MODE1_MIN_DUTY,
            abs(applied_right_duty) >= MODE1_MIN_DUTY);
    }
}

static bool last_line_was_left = true;
static bool mode2_searching = false;
static int64_t mode2_search_started_ms = 0;
#define MODE2_SEARCH_INNER_MS       100
#define MODE2_SEARCH_REVERSE_MS     50
#define MODE2_SEARCH_OUTER_DUTY     900
#define MODE2_SEARCH_INNER_DUTY     1000
void run_mode2(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB)
{
    int center = gpio_get_level(sensor_Photoreflector_center);
    int left_inner = gpio_get_level(sensor_Photoreflector_left_inner);
    int right_inner = gpio_get_level(sensor_Photoreflector_right_inner);

    bool line_detected =
        center == LINE_DETECTED_LEVEL;
    bool line_detected_left =
        left_inner == LINE_DETECTED_LEVEL;
    bool line_detected_right =
        right_inner == LINE_DETECTED_LEVEL;
    bool any_line_detected =
        line_detected || line_detected_left || line_detected_right;

    int64_t now_ms = esp_timer_get_time() / 1000;

    // 見失ったら探索を開始。探索時間中はセンサ反応を無視する
    if (mode2_searching || !any_line_detected) {
        if (!mode2_searching) {
            mode2_searching = true;
            mode2_search_started_ms = now_ms;
        }

        int64_t elapsed_ms = now_ms - mode2_search_started_ms;

        if (elapsed_ms < MODE2_SEARCH_INNER_MS + MODE2_SEARCH_REVERSE_MS) {
            bool drive_outer = elapsed_ms >= MODE2_SEARCH_INNER_MS;
            motor_state outer_state = drive_outer ? MOTOR_FORWARD : MOTOR_STOP;
            uint32_t outer_duty = drive_outer ? MODE2_SEARCH_OUTER_DUTY : 0;

            if (last_line_was_left) {
                set_motorA_state(MOTOR_BACKWARD);
                set_motorB_state(outer_state);
                set_motor_speed(
                    cmprA, cmprB, MODE2_SEARCH_INNER_DUTY, outer_duty);
            } else {
                set_motorA_state(outer_state);
                set_motorB_state(MOTOR_BACKWARD);
                set_motor_speed(
                    cmprA, cmprB, outer_duty, MODE2_SEARCH_INNER_DUTY);
            }

            return;
        }

        // 指定時間が終わったら、次の行からセンサ判定に戻る
        mode2_searching = false;
    }

    set_motorA_state(MOTOR_FORWARD);
    set_motorB_state(MOTOR_FORWARD);

    if (line_detected) {
        set_motor_speed(cmprA, cmprB, 600, 600);

    } else if (line_detected_left) {
        // 左センサで検出：左へ曲がる
        set_motorA_state(MOTOR_STOP);
        set_motor_speed(cmprA, cmprB, 0, 950);
        last_line_was_left = true;

    } else if (line_detected_right) {
        // 右センサで検出：右へ曲がる
        set_motorB_state(MOTOR_STOP);
        set_motor_speed(cmprA, cmprB, 950, 0);
        last_line_was_left = false;
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
    if (!left_should_rotate || left_pulse_count != previous_left_pulse) {
        previous_left_pulse = left_pulse_count;
        last_left_rotation_ms = now_ms;
    }

    // 右のパルスが増えた = 右車輪は回転中
    if (!right_should_rotate || right_pulse_count != previous_right_pulse) {
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
