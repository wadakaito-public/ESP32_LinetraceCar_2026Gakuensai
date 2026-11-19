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

/* 基板の配線と PWM 設定 */

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
// 検出したい線の色の値が白なら 0、黒なら 1。
#define LINE_DETECTED_LEVEL 0

//bit列に記憶
#define GPIO_OUTPUT_PIN ((1ULL << MotorA_IN1)|(1ULL << MotorA_IN2)|(1ULL << MotorB_IN1)|(1ULL << MotorB_IN2)|(1ULL << STBY)|(1ULL << LED))
#define GPIO_INPUT_PIN ((1ULL << SW0)|(1ULL << sensor_encoder_A_C1)|(1ULL << sensor_encoder_A_C2)|(1ULL << sensor_encoder_B_C1)|(1ULL << sensor_encoder_B_C2)|(1ULL << sensor_Photoreflector_center)|(1ULL << sensor_Photoreflector_left_inner)|(1ULL << sensor_Photoreflector_left_outer)|(1ULL << sensor_Photoreflector_right_inner)|(1ULL << sensor_Photoreflector_right_outer))

#define PWM_FREQ_HZ 20000 //PWM周波数
#define PERIOD_TICKS 1000 // mcpwm_timer_config.resolution_hz/PERIOD_TICKSでPWM1周期の時間→1000カウント中、duty比に応じた時間ON/OFFを実行→PWM信号の分解能は1000段階

/* 車体の状態・表示・共通ログ */
//モータの状態を定義する列挙型の定義
typedef enum {
    MOTOR_STOP,
    MOTOR_BRAKE,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} motor_state;
static led_strip_handle_t led_strip;

static const char *TAG = "main";

typedef enum {
    CAR_STOPPED, CAR_MODE_SELECT, CAR_RUNNING
} car_state_t;
static car_state_t car_state = CAR_STOPPED;
static int selected_mode = 1;

void bt_log(const char *format, ...);
#define FW_VERSION "v3.6.1"
// USB と Bluetooth に同じ内容を出す。Bluetooth の1行は改行込みで119バイトまで。
#define RUN_LOG(fmt, ...) do { \
    ESP_LOGI(TAG, fmt, ##__VA_ARGS__); \
    bt_log(fmt "\r\n", ##__VA_ARGS__); \
} while (0)
void bt_log_start(void);

/* モータ出力・GPIO 初期化・エンコーダ割り込み */
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
// C1 の立ち上がりを左右別に積算する。各監視処理は前回値との差で回転を判定する。
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
// 左右の PWM 比較値を更新する。回転方向は set_motorA_state / set_motorB_state で指定する。
void set_motor_speed(
    mcpwm_cmpr_handle_t cmprA,
    mcpwm_cmpr_handle_t cmprB,
    uint32_t left_duty,
    uint32_t right_duty)
{
    mcpwm_comparator_set_compare_value(cmprA, left_duty);
    mcpwm_comparator_set_compare_value(cmprB, right_duty);
}

/* モード1のストール監視と再始動状態 */
#define STALL_TIMEOUT_MS 300
#define RESTART_PAUSE_MS 0

// 前回のパルス数と最後に変化した時刻を左右別に保持する。
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

/* 試行の計測と停止判定（モード1・3共通） */
// 起動してからの試行番号。終了ログ RUN・TIME・STALL の3行を対応付ける。
static unsigned long run_id;
// 自動停止時に押していたボタンを離しても、次の走行を開始しないためのフラグ。
static bool ignore_button_release;
static bool sw0_is_pressed(void);
#define TRIAL_NO_WHEEL_MOTION_MS 5000

static struct {
    bool active;       // モード1または3で試行を計測中
    bool searching;    // 開始済みで、まだ累計に加えていない探索区間がある
    int mode;
    int64_t started_ms;
    int64_t search_started_ms;
    int64_t search_ms;
    int64_t last_wheel_motion_ms;
    uint32_t last_left_pulse;
    uint32_t last_right_pulse;
    uint32_t search_count;
    uint32_t stall_backoff;  // 探索中のストールから後退に入った回数
    uint32_t stall_restart;  // 追従中のストールから再始動に入った回数
} trial;

static void reset_trial_controller(int mode);

// 走行開始時に制御状態と計測値を初期化し、現在のパルス数を監視の基準にする。
static void trial_begin(int mode, int64_t now_ms)
{
    trial.active = mode == 1 || mode == 3;
    if (!trial.active) return;

    reset_trial_controller(mode);
    ++run_id;
    trial.mode = mode;
    trial.started_ms = now_ms;
    trial.searching = false;
    trial.search_ms = 0;
    trial.search_count = 0;
    trial.stall_backoff = 0;
    trial.stall_restart = 0;
    trial.last_wheel_motion_ms = now_ms;
    trial.last_left_pulse = left_pulse_count;
    trial.last_right_pulse = right_pulse_count;
}

// 探索に入った周期に1回だけ呼ぶ。探索中の後退も同じ探索区間に含める。
static void trial_search_begin(int64_t now_ms)
{
    trial.searching = true;
    trial.search_started_ms = now_ms;
    trial.search_count++;
}

// 追従復帰または走行終了の時点で、開いている探索区間の経過時間を加算する。
static void trial_search_end(int64_t now_ms)
{
    if (!trial.searching) return;
    trial.search_ms += now_ms - trial.search_started_ms;
    trial.searching = false;
}

// モータを停止し、計測対象の試行なら探索時間を確定して USB と Bluetooth に結果を出す。
static void stop_car(const char *reason, int64_t now_ms)
{
    car_state = CAR_STOPPED;
    set_motorA_state(MOTOR_STOP);
    set_motorB_state(MOTOR_STOP);
    led_strip_set_pixel(led_strip, 0, 100, 0, 0);
    led_strip_refresh(led_strip);
    if (!trial.active) {
        RUN_LOG("STATE STOPPED mode=%d reason=%s", selected_mode, reason);
        return;
    }
    trial_search_end(now_ms);
    trial.active = false;
    RUN_LOG("RUN id=%lu fw=" FW_VERSION " mode=%d control=%s stop_reason=%s",
            run_id, trial.mode, trial.mode == 1 ? "v3.6.0" : "v2.0.0", reason);
    RUN_LOG("TIME id=%lu run_ms=%lld search_count=%lu search_ms=%lld",
            run_id, (long long)(now_ms - trial.started_ms),
            (unsigned long)trial.search_count, (long long)trial.search_ms);
    RUN_LOG("STALL id=%lu stall_count=%lu stall_backoff=%lu stall_restart=%lu",
            run_id, (unsigned long)(trial.stall_backoff + trial.stall_restart),
            (unsigned long)trial.stall_backoff, (unsigned long)trial.stall_restart);
}

// 左右いずれかのパルスが変化すると無回転時間を数え直す。両輪が5秒間変化しないと停止する。
static void trial_check_wheel_motion(void)
{
    if (!trial.active || car_state != CAR_RUNNING) return;

    const int64_t now_ms = esp_timer_get_time() / 1000;
    const uint32_t left = left_pulse_count;
    const uint32_t right = right_pulse_count;
    if (left != trial.last_left_pulse || right != trial.last_right_pulse) {
        trial.last_left_pulse = left;
        trial.last_right_pulse = right;
        trial.last_wheel_motion_ms = now_ms;
    } else if (now_ms - trial.last_wheel_motion_ms >= TRIAL_NO_WHEEL_MOTION_MS) {
        ignore_button_release = sw0_is_pressed();
        stop_car("no_wheel_motion_5s", now_ms);
    }
}

/* 操作ボタンとモード表示 */
//ボタン長押しの時間
#define LONG_PRESS_MS 800
void blink_mode_purple(led_strip_handle_t strip, int count);
void update_mode_indicator(void);
#define MODE_INDICATOR_INTERVAL_MS 2000
static int64_t last_mode_activity_ms = 0;
//スイッチ押下判定
static bool was_pressed = false;
//押下された時間
static int64_t pressed_at_ms = 0;
static bool sw0_is_pressed(void)
{
    return gpio_get_level(SW0) == 0;
}
// ボタンを離した周期で押下時間を確定し、現在の車体状態に応じて短押し・長押しを処理する。
void handle_sw0(void)
{
    bool pressed = sw0_is_pressed();
    int64_t now_ms = esp_timer_get_time() / 1000;

    // 押し始め
    if (pressed && !was_pressed) {
        ignore_button_release = false;
        pressed_at_ms = now_ms;
    }

    // 離した瞬間に、短押し／長押しを確定する
    if (!pressed && was_pressed) {
        if (ignore_button_release) {
            was_pressed = false;
            ignore_button_release = false;
            return;
        }
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
                RUN_LOG("STATE MODE_SELECT mode=%d", selected_mode);
            } else {
                car_state = CAR_RUNNING;
                trial_begin(selected_mode, now_ms);
                // LEDを「走行中」の色へ
                // led_strip_set_pixel(ハンドル, LEDの番号, 赤, 緑, 青) ※各0〜255
                led_strip_set_pixel(led_strip, 0, 0, 100, 0);
                led_strip_refresh(led_strip);
                RUN_LOG("STATE RUNNING mode=%d", selected_mode);
            }
            
            break;

        case CAR_MODE_SELECT:
            if (is_long_press) {
                // 現在の selected_mode を確定
                car_state = CAR_STOPPED;
                // LEDを「停止」の色へ
                led_strip_set_pixel(led_strip, 0, 100, 0, 0);
                led_strip_refresh(led_strip);
                RUN_LOG("STATE STOPPED mode=%d", selected_mode);
            } else {
                // 短押しごとにモードを切替
                selected_mode++;
                if (selected_mode >= 5) {
                    selected_mode = 1;
                }
                last_mode_activity_ms = now_ms;
                RUN_LOG("MODE selected=%d", selected_mode);
                blink_mode_purple(led_strip, selected_mode);
            }
            break;

        case CAR_RUNNING:
            if (!is_long_press) {
                stop_car("button", now_ms);
            }
            break;
        }
    }

    was_pressed = pressed;
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

/* モード1　完成版の P 制御（v3.6.0） */
//基礎デューティ比
#define MODE1_BASE_DUTY 800
//比例ゲイン
#define MODE1_KP 300
//最低デューティ比
#define MODE1_MIN_DUTY 400
//探索用パラメータ
#define MODE1_SEARCH_TURN_MS 700
#define MODE1_SEARCH_DUTY 1000
#define MODE1_SEARCH_INNER_DUTY 1000
#define MODE1_SEARCH_BACK_DUTY 900
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
// ラインを再確認するまで探索を継続する。recovery_state は探索中の後退区間を表す。
static bool searching_state = false;
static int64_t search_started_ms = 0;

static bool recovery_state = false;
static int64_t recovery_started_ms = 0;

static float last_line_error = 0.0f;
// 再始動中に KICK 指令を送ったかを記録する。開始時刻と対象輪のパルス数は送信時に保存する。
static bool restart_drive_sent = false;
static bool restart_left_wheel = false;
static int64_t restart_kick_started_ms = 0;
static uint32_t restart_outer_pulse_start = 0;
// 探索中の連続検出回数。3回で追従に戻る。
static int consecutive_detections = 0;
// 追従中に未検出で直前の駆動指令を保持した回数。
static int consecutive_misses = 0;
static int64_t last_sensor_log_ms = 0;
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
void run_mode1(mcpwm_cmpr_handle_t cmprA,mcpwm_cmpr_handle_t cmprB){
    int right_duty;
    int left_duty;
    
    bool restart_finished = false;

    if (restart_waiting) {
        int64_t now_ms = esp_timer_get_time() / 1000;

        int64_t kick_elapsed_ms = now_ms - restart_kick_started_ms;
        bool center_on_line =
            gpio_get_level(sensor_Photoreflector_center) == LINE_DETECTED_LEVEL;

        // 最初の150 msは駆動を維持する。
        // その後は中央でラインを検出するか、最大時間に達したら終了。
        if (kick_elapsed_ms <150 ||
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

            RUN_LOG("BLUE t=%lld pwm L=%d R=%d enc L=%lu R=%lu",
                   (long long)restart_kick_started_ms,
                   applied_left_duty, applied_right_duty,
                   (unsigned long)left_start, (unsigned long)right_start);
            led_strip_set_pixel(led_strip, 0, 0, 0, 100);
            led_strip_refresh(led_strip);
            return;
        }

        uint32_t outer_pulse_now =
            restart_left_wheel ? left_pulse_count : right_pulse_count;

        RUN_LOG("KICK_END t=%lld wheel=%c delta=%lu",
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
    int64_t sensor_log_ms = esp_timer_get_time() / 1000;

    if (sensor_log_ms - last_sensor_log_ms >= 200) {
        last_sensor_log_ms = sensor_log_ms;
        ESP_LOGD(TAG, "count=%d detected=%d %d %d %d %d",
             detected_count, left_outer_detected, left_detected,
             center_detected, right_detected, right_outer_detected);
    }
    //検出の連続を数える
    if(searching_state && detected_count > 0){
        consecutive_detections++;
    }else {
        consecutive_detections = 0;
    }
    bool line_confirmed = detected_count > 0 && (!searching_state||consecutive_detections >= 3);

    //未検出の連続を数える
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
            trial_search_begin(now_ms);
            search_started_ms = now_ms;
            reset_encoder_stall_monitor();
        }
        if ((left_motor_stalled || right_motor_stalled) && !recovery_state) {
            recovery_state = true;
            trial.stall_backoff++;
            recovery_started_ms = now_ms;
        }

        //最低でも探索を続ける時間
        if(searching_state && now_ms - search_started_ms > MODE1_SEARCH_TURN_MS){
            return;
        }

        if (recovery_state) {
            int64_t recovery_elapsed_ms = now_ms - recovery_started_ms;

            // 最初の800 msは後退
            if (recovery_elapsed_ms < 800) {
                apply_mode1_drive(
                    cmprA, cmprB,
                    -MODE1_SEARCH_BACK_DUTY, -MODE1_SEARCH_BACK_DUTY);

                update_encoder_stall_monitor(true, true);
                return;
            }
            /*
            // 後退後の200 msは両輪前進で旋回し、探索を立ち上げる
            if (recovery_elapsed_ms < 700) {
                bool turn_right = last_line_error >= 0.0f;

                apply_mode1_drive(
                    cmprA, cmprB,
                    turn_right ? 1000 : 00,
                    turn_right ? 0 : 1000);

                return;
            }
            */
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
        trial_search_end(esp_timer_get_time() / 1000);
        
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
            trial.stall_restart++;
            restart_started_ms = now_ms;
            restart_drive_sent = false;
            restart_left_wheel = last_line_error > 0.0f;

            RUN_LOG("RED t=%lld stall L=%d R=%d pwm L=%d R=%d enc L=%lu R=%lu",
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

/* モード2　ON/OFF 制御 */
#define MODE2_SEARCH_INNER_MS       100
#define MODE2_SEARCH_REVERSE_MS     50
#define MODE2_SEARCH_OUTER_DUTY     900
#define MODE2_SEARCH_INNER_DUTY     1000
// 最後にラインを捉えた側を記憶し、未検出時の探索方向に使う。
static bool last_line_was_left = true;
static bool mode2_searching = false;
static int64_t mode2_search_started_ms = 0;
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

/* モード3　第1回改良版の P 制御（v2.0.0） */
#define LEGACY_STALL_TIMEOUT_MS 500
#define LEGACY_RESTART_PAUSE_MS 100
#define MODE3_RAMP_UP_STEP 30     // 10 msごとに増やすPWM値
#define MODE3_RAMP_DOWN_STEP 60   // 減速時は少し速く下げる

#define MODE3_BASE_DUTY 700
//比例ゲイン
#define MODE3_KP 300
//最低デューティ比
#define MODE3_MIN_DUTY 500
//探索用パラメータ
#define MODE3_SEARCH_TURN_MS 600
#define MODE3_SEARCH_DUTY 900
#define MODE3_SEARCH_INNER_DUTY 500
// legacy_ の変数はモード3専用。旧版の監視しきい値と再始動待ち時間で動作する。
static bool legacy_restart_waiting;
static int64_t legacy_restart_started_ms;
static uint32_t legacy_previous_left_pulse = 0;
static uint32_t legacy_previous_right_pulse = 0;

static int64_t legacy_last_left_rotation_ms = 0;
static int64_t legacy_last_right_rotation_ms = 0;

static bool legacy_left_motor_stalled = false;
static bool legacy_right_motor_stalled = false;
static bool legacy_encoder_monitor_started = false;
static int legacy_applied_left_duty;
static int legacy_applied_right_duty;
static bool legacy_searching_state = false;
static int64_t legacy_search_started_ms = 0;

static bool legacy_recovery_state = false;
static int64_t legacy_recovery_started_ms = 0;

static float legacy_last_line_error = 0.0f;
static int legacy_consecutive_detections = 0;
static int64_t legacy_last_sensor_log_ms = 0;
static void legacy_update_encoder_stall_monitor(
    bool left_should_rotate,
    bool right_should_rotate)
{
    int64_t now_ms = esp_timer_get_time() / 1000;

    // 初回は基準値だけ保存する
    if (!legacy_encoder_monitor_started) {
        legacy_previous_left_pulse = left_pulse_count;
        legacy_previous_right_pulse = right_pulse_count;
        legacy_last_left_rotation_ms = now_ms;
        legacy_last_right_rotation_ms = now_ms;
        legacy_encoder_monitor_started = true;
        return;
    }

    // 左のパルスが増えた = 左車輪は回転中
    if (left_pulse_count != legacy_previous_left_pulse) {
        legacy_previous_left_pulse = left_pulse_count;
        legacy_last_left_rotation_ms = now_ms;
    }

    // 右のパルスが増えた = 右車輪は回転中
    if (right_pulse_count != legacy_previous_right_pulse) {
        legacy_previous_right_pulse = right_pulse_count;
        legacy_last_right_rotation_ms = now_ms;
    }

    legacy_left_motor_stalled =
        left_should_rotate &&
        (now_ms - legacy_last_left_rotation_ms > LEGACY_STALL_TIMEOUT_MS);

    legacy_right_motor_stalled =
        right_should_rotate &&
        (now_ms - legacy_last_right_rotation_ms > LEGACY_STALL_TIMEOUT_MS);
}
static void legacy_reset_encoder_stall_monitor(void)
{
    int64_t now_ms = esp_timer_get_time() / 1000;

    legacy_previous_left_pulse = left_pulse_count;
    legacy_previous_right_pulse = right_pulse_count;

    legacy_last_left_rotation_ms = now_ms;
    legacy_last_right_rotation_ms = now_ms;

    legacy_left_motor_stalled = false;
    legacy_right_motor_stalled = false;
    legacy_encoder_monitor_started = true;
}
static int legacy_ramp_duty(int current, int target)
{
    if (current < target) {
        current += MODE3_RAMP_UP_STEP;

        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= MODE3_RAMP_DOWN_STEP;

        if (current < target) {
            current = target;
        }
    }

    return current;
}
void run_mode3(mcpwm_cmpr_handle_t cmprA,mcpwm_cmpr_handle_t cmprB){
    int right_duty;
    int left_duty;
    
    
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
    int64_t sensor_log_ms = esp_timer_get_time() / 1000;

    if (sensor_log_ms - legacy_last_sensor_log_ms >= 200) {
        legacy_last_sensor_log_ms = sensor_log_ms;
        ESP_LOGD(TAG, "count=%d detected=%d %d %d %d %d",
             detected_count, left_outer_detected, left_detected,
             center_detected, right_detected, right_outer_detected);
    }
    //検出の連続を数える
    if(legacy_searching_state && detected_count > 0){
        legacy_consecutive_detections++;
    }else {
        legacy_consecutive_detections = 0;
    }
    bool line_confirmed = detected_count > 0 && (!legacy_searching_state||legacy_consecutive_detections >= 3);
    if (!line_confirmed) {
        led_strip_set_pixel(led_strip, 0, 80, 0, 80);
        led_strip_refresh(led_strip);
        //線を検知していない
        int64_t now_ms = esp_timer_get_time() / 1000;
        // 線を見失った瞬間の時刻を記録
        if (!legacy_searching_state) {
            //未検出かつ探索状態ではない＝これから探索を始める
            legacy_searching_state = true;
            trial_search_begin(now_ms);
            legacy_search_started_ms = now_ms;
            legacy_reset_encoder_stall_monitor();
        }
        if ((legacy_left_motor_stalled || legacy_right_motor_stalled) && !legacy_recovery_state){
            //どちらかのモータがストール(止まっている)
            legacy_recovery_state = true;
            trial.stall_backoff++;
            legacy_recovery_started_ms = now_ms;
        }
        if (legacy_recovery_state){ 
            if(now_ms - legacy_recovery_started_ms < 1200){
                //ストール、800㎳後退
                set_motorA_state(MOTOR_BACKWARD);
                set_motorB_state(MOTOR_BACKWARD);
                set_motor_speed(cmprA, cmprB, MODE3_SEARCH_DUTY, MODE3_SEARCH_DUTY);
                legacy_update_encoder_stall_monitor(true,true);
                return;
            }
        //上のifを抜け、この行に到達(returnしていない)→800ms経過
        legacy_recovery_state = false;
        legacy_reset_encoder_stall_monitor();
        }
        
        // 最後にラインを検出した側から探索を始める
        bool initial_search_right = (legacy_last_line_error >= 0.0f);
        /*
        // 600 msごとに探索方向を反転する
        int64_t elapsed_ms = now_ms - legacy_search_started_ms;
        int64_t turn_index = elapsed_ms / MODE3_SEARCH_TURN_MS;
        bool search_right = (turn_index % 2 == 0)
            ? initial_search_right
            : !initial_search_right;
        */
        
        if (initial_search_right) {
            set_motorA_state(MOTOR_FORWARD);
            set_motorB_state(MOTOR_BACKWARD);
            set_motor_speed(cmprA, cmprB, MODE3_SEARCH_DUTY, MODE3_SEARCH_INNER_DUTY);
        } else {
            set_motorA_state(MOTOR_BACKWARD);
            set_motorB_state(MOTOR_FORWARD);
            set_motor_speed(cmprA, cmprB, MODE3_SEARCH_INNER_DUTY, MODE3_SEARCH_DUTY);
        }
        legacy_update_encoder_stall_monitor(true, true);
        return;
    
    } else {
        //線を検出した
        led_strip_set_pixel(led_strip, 0, 50, 50, 50);
        led_strip_refresh(led_strip);
        if (legacy_searching_state) {
            //直前が探索状態だったなら回転監視をリセット
            legacy_reset_encoder_stall_monitor();
        }
        //状態フラグリセット
        legacy_recovery_state = false;
        legacy_searching_state = false;
        trial_search_end(esp_timer_get_time() / 1000);
        
        //偏差を計算する
        error = (
            (left_detected ? -1.0f : 0.0f)
            + (right_detected ? 1.0f : 0.0f)
            + (left_outer_detected ? -2.5f : 0.0f)
            + (right_outer_detected ? 2.5f : 0.0f)
        ) / detected_count;
        if (error != 0.0f) {
            legacy_last_line_error = error;
        }
        //P制御の補正値
        int correction = (int)(MODE3_KP * error);
        //補正値を追加
        left_duty = MODE3_BASE_DUTY + correction;
        right_duty = MODE3_BASE_DUTY - correction;

        if (left_duty < MODE3_MIN_DUTY)left_duty = MODE3_MIN_DUTY;
        if(right_duty < MODE3_MIN_DUTY)right_duty = MODE3_MIN_DUTY; 
        if (left_duty > PERIOD_TICKS) left_duty = PERIOD_TICKS;
        if (right_duty > PERIOD_TICKS) right_duty = PERIOD_TICKS; 

        int64_t now_ms = esp_timer_get_time() / 1000;
        // 片輪でも止まったら、一度PWMを0にして再始動準備
        if ((legacy_left_motor_stalled || legacy_right_motor_stalled) && !legacy_restart_waiting) {
            legacy_applied_left_duty = 0;
            legacy_applied_right_duty = 0;
            legacy_restart_waiting = true;
            trial.stall_restart++;
            legacy_restart_started_ms = now_ms;

            legacy_reset_encoder_stall_monitor();

            set_motorA_state(MOTOR_FORWARD);
            set_motorB_state(MOTOR_FORWARD);
            set_motor_speed(cmprA, cmprB, 0, 0);
            return;
        }
        // 100 msだけ無通電にしてから、通常の段階加速へ戻る
        if (legacy_restart_waiting) {
            if (now_ms - legacy_restart_started_ms < LEGACY_RESTART_PAUSE_MS) {
                return;
              }
            legacy_restart_waiting = false;
        }
        // 目標PWMへ段階的に近づける
        legacy_applied_left_duty = legacy_ramp_duty(legacy_applied_left_duty, left_duty);
        legacy_applied_right_duty = legacy_ramp_duty(legacy_applied_right_duty, right_duty);
        set_motorA_state(MOTOR_FORWARD);
        set_motorB_state(MOTOR_FORWARD);
        set_motor_speed(cmprA, cmprB, legacy_applied_left_duty, legacy_applied_right_duty);
        legacy_update_encoder_stall_monitor(legacy_applied_left_duty >= MODE3_MIN_DUTY,legacy_applied_right_duty >= MODE3_MIN_DUTY);
    }

}

/* 試行開始時の制御状態の初期化 */
// 選択した制御の履歴を初期化し、前の試行の偏差・駆動指令・復帰状態を持ち越さない。
static void reset_trial_controller(int mode)
{
    if (mode == 1) {
        applied_left_duty = 0;
        applied_right_duty = 0;
        last_line_error = 0.0f;
        last_sensor_log_ms = 0;
        consecutive_detections = 0;
        searching_state = false;
        search_started_ms = 0;
        recovery_state = false;
        recovery_started_ms = 0;
        restart_waiting = false;
        restart_started_ms = 0;
        left_motor_stalled = false;
        right_motor_stalled = false;
        encoder_monitor_started = false;
        consecutive_misses = 0;
        restart_drive_sent = false;
        restart_left_wheel = false;
        restart_kick_started_ms = 0;
        restart_outer_pulse_start = 0;
    } else {
        legacy_applied_left_duty = 0;
        legacy_applied_right_duty = 0;
        legacy_last_line_error = 0.0f;
        legacy_last_sensor_log_ms = 0;
        legacy_consecutive_detections = 0;
        legacy_searching_state = false;
        legacy_search_started_ms = 0;
        legacy_recovery_state = false;
        legacy_recovery_started_ms = 0;
        legacy_restart_waiting = false;
        legacy_restart_started_ms = 0;
        legacy_left_motor_stalled = false;
        legacy_right_motor_stalled = false;
        legacy_encoder_monitor_started = false;
    }
}

/* 起動処理と走行ループ */
void app_main() {
    //初期化処理
    ESP_LOGD(TAG, "initialize GPIO");
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
    ESP_LOGD(TAG, "create timer");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .resolution_hz = PWM_FREQ_HZ * PERIOD_TICKS,//モータ周波数*分解能=制御周波数
        .period_ticks = PERIOD_TICKS
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config,&timer));
    ESP_LOGD(TAG, "timer created");
    
    //2,MCPWM オペレータの設定
    ESP_LOGD(TAG, "create operator");
    mcpwm_oper_handle_t operA = NULL;
    mcpwm_operator_config_t operatorA_config = { .group_id = 0 };
    mcpwm_new_operator(&operatorA_config, &operA);
    ESP_LOGD(TAG, "connect operator");
    mcpwm_operator_connect_timer(operA, timer);

    mcpwm_oper_handle_t operB = NULL;
    mcpwm_operator_config_t operatorB_config = { .group_id = 0 };
    mcpwm_new_operator(&operatorB_config, &operB);
    ESP_LOGD(TAG, "connect operator");
    mcpwm_operator_connect_timer(operB, timer);

    //3,MCPWM コンパレータの設定
    ESP_LOGD(TAG, "create comparator");
    mcpwm_cmpr_handle_t cmprA = NULL;
    mcpwm_comparator_config_t cmprA_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operA,&cmprA_config, &cmprA);

    mcpwm_cmpr_handle_t cmprB = NULL;
    mcpwm_comparator_config_t cmprB_config = {.flags.update_cmp_on_tez = true};
    mcpwm_new_comparator(operB,&cmprB_config, &cmprB);

    //4,MCPWM ジェネレータの設定
    ESP_LOGD(TAG, "create generator");
    mcpwm_gen_handle_t genA = NULL;
    mcpwm_generator_config_t genA_config = { .gen_gpio_num = PWM_A};
    mcpwm_new_generator(operA,&genA_config,&genA);

    mcpwm_gen_handle_t genB = NULL;
    mcpwm_generator_config_t genB_config = { .gen_gpio_num = PWM_B};
    mcpwm_new_generator(operB,&genB_config,&genB);

    //5,波形生成ルールの設定
    
    //5-1,タイマイベント設定(ジェネレータA,タイマイベント(カウントアップ中,カウンタ値0の瞬間,ジェネレータは出力信号をHIGHに));
    ESP_LOGD(TAG, "configure timer event");
    mcpwm_generator_set_action_on_timer_event(genA,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_timer_event(genB,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,MCPWM_TIMER_EVENT_EMPTY,MCPWM_GEN_ACTION_HIGH));
    //5-2,コンパレータ一致時イベント設定
    ESP_LOGD(TAG, "configure compare event");
    mcpwm_generator_set_action_on_compare_event(genA,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmprA,MCPWM_GEN_ACTION_LOW));
    mcpwm_generator_set_action_on_compare_event(genB,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,cmprB,MCPWM_GEN_ACTION_LOW));
    //6,タイマスタート
    ESP_LOGD(TAG, "enable timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
     ESP_LOGD(TAG, "start timer");
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer,MCPWM_TIMER_START_NO_STOP));
    
    //オンボードフルカラーLED(WS2812)各種設定
    ESP_LOGD(TAG, "Onboard LED initializing");
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

    led_strip_set_pixel(led_strip,0,100,0,0);
    led_strip_refresh(led_strip);

    bt_log_start();
    ESP_LOGI(TAG, "READY fw=" FW_VERSION " mode=%d", selected_mode);

    while (1){
        handle_sw0();
        trial_check_wheel_motion();
        update_mode_indicator();
        if(car_state == CAR_RUNNING){
            switch (selected_mode) {
            case 1:
                run_mode1(cmprA, cmprB);
                break;
            
            case 2:
                run_mode2(cmprA,cmprB);
                break;
            case 3:
                run_mode3(cmprA, cmprB);
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
