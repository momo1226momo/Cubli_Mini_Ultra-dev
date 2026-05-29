#include <Adafruit_NeoPixel.h>  //用于控制WS2812B可编程LED灯
#include <Arduino.h>  // Arduino核心库，提供基础函数如Serial、delay等

#include "bsp/battery.h"  // 电池管理板级支持包
#include "bsp/can.h"   // CAN通信驱动
#include "bsp/key.h"   // 按键驱动
#include "bsp/servo_motor.h"    // 舵机电机驱动
#include "comm/comm.h"   // 通用通信模块
#include "comm/time.h"  // 时间相关工具
#include "config/config.h"   // 系统配置（引脚定义、常量等）
#include "control/angle_offset_calibration.h"  // 角度偏移自动校准
#include "control/p_balance_control.h"  // 点平衡控制器
#include "control/param.h"  // 参数管理（EEPROM存储）
#include "control/pid.h"  // PID控制器
#include "control/serial_commander.h"  // 串口命令解析器
#include "control/u_balance_control.h"  // 边平衡控制器
#include "control/wifi_commander.h"   // WiFi命令控制器
#include "imu/ahrs.h"   // 姿态解算算法
#include "imu/qmi8658_driver.h"  // QMI8658 IMU传感器驱动

using namespace CubliMini::Bsp;
using namespace CubliMini::Comm;
using namespace CubliMini::Control;

//系统状态、电机对应LED灯带
Adafruit_NeoPixel system_strip =
    Adafruit_NeoPixel(SYSTEM_LED_NUM, SYSTEM_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel motor1_strip =
    Adafruit_NeoPixel(MOTOR_LED_NUM, MOTOR1_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel motor2_strip =
    Adafruit_NeoPixel(MOTOR_LED_NUM, MOTOR2_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel motor3_strip =
    Adafruit_NeoPixel(MOTOR_LED_NUM, MOTOR3_LED_PIN, NEO_GRB + NEO_KHZ800);

// 创建三个舵机电机对象
ServoMotor servo_motor1(SERVO_MOTOR_1_PIN, 13);
ServoMotor servo_motor2(SERVO_MOTOR_2_PIN, 14);
ServoMotor servo_motor3(SERVO_MOTOR_3_PIN, 15);

// 按键对象定义，暂时不使用key
KeyDriver key1;
KeyDriver key2;

//任务函数声明
void CmdTask(void *parameter);
void ImuTask(void *parameter);
void CanRecTask(void *parameter);
void ControlTask(void *parameter);
void CanSendTask(void *parameter);
void MotorLedTask(void *parameter);


//全局对象定义 
CubliMini::Bsp::CanDriver can_;
CubliMini::ImuDriver::ImuDriverBase *imu_driver_ptr_  = nullptr;
CubliMini::Imu::AHRS *ahrs_ptr                        = nullptr;
CubliMini::Control::WifiCommander *wifi_commander_ptr = nullptr;
Battery battery_;

//FreeRTOS队列句柄
QueueHandle_t xQueueImu;  // IMU数据队列，传递姿态数据给控制任务
QueueHandle_t xQueueCanSend;  // CAN发送队列，传递电机控制指令
QueueHandle_t xQueueCanGet;  // CAN接收队列，传递电机速度反馈

float cal_angle_kp = 0.000022f;
// U轴（边）平衡控制器
UBalanceControl U_balance_control(Param::GetInstance()->GetUAxisParam(), cal_angle_kp);
// P轴（点）平衡控制器
PBalanceControl p_balance_control(Param::GetInstance()->GetPAxisParam(), cal_angle_kp);

void setup()
{
    Serial.begin(115200);

    system_strip.begin();
    system_strip.setBrightness(50);
    system_strip.setPixelColor(0, system_strip.Color(255, 0, 0));
    system_strip.show();

    servo_motor1.re_init();
    servo_motor2.re_init();
    servo_motor3.re_init();

    imu_driver_ptr_ = new CubliMini::ImuDriver::Qmi8658Driver(&Wire);
    //尝试初始化IMU
    if (imu_driver_ptr_->Init() != 0)
    {
        for(;;)
        {
            Serial.println("Failed to find imu - check your wiring!");
            delay(1000);
        }
    }
    // 创建AHRS姿态解算对象
    ahrs_ptr = new CubliMini::Imu::AHRS(imu_driver_ptr_, IMU_STATIC_GYRO_THRESHOLD);

    // 从EEPROM加载保存的PID等参数
    Param::GetInstance()->Load();

    // ========== 电机LED初始化 ==========
    motor1_strip.begin();
    motor1_strip.setBrightness(50);
    motor1_strip.show();

    motor2_strip.begin();
    motor2_strip.setBrightness(50);
    motor2_strip.show();

    motor3_strip.begin();
    motor3_strip.setBrightness(50);
    motor3_strip.show();

    // ========== 按键初始化 ==========
    key1.Init(SYSTEM_KEY1_PIN);
    key2.Init(SYSTEM_KEY2_PIN);

    // ========== FreeRTOS队列创建 ==========
    xQueueImu     = xQueueCreate(1, sizeof(ImuData_t));
    xQueueCanSend = xQueueCreate(1, sizeof(PBalanceControl::MotorControl));
    xQueueCanGet  = xQueueCreate(1, sizeof(PBalanceControl::MotorSpeed));
    can_.Init(CAN_TXD_PIN, CAN_RXD_PIN, CAN_SPEED_1000KBPS);

    xTaskCreatePinnedToCore(CmdTask, "cmd task", 30000, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ImuTask, "imu task", 10000, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(CanRecTask, "can task", 10000, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(CanSendTask, "can1 task", 10000, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(ControlTask, "control task", 20000, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(MotorLedTask, "MOTOR1 LED task", 10000, NULL, 2, NULL, 1);
    // 优先级说明：数字越大优先级越高
    // ImuTask和CanRecTask优先级最高(6)，保证传感器数据及时读取
    // ControlTask优先级(5)次之，保证控制实时性
    delay(1000);
}

//CAN接收任务
PBalanceControl::MotorSpeed g_get_motor_speed;
void CanRecTask(void *parameter)
{
    for (;;)  // FreeRTOS任务通常是无限循环
    {
        PBalanceControl::MotorSpeed get_motor_speed;
        if (can_.CanGetMotorSpeed(get_motor_speed.ch1, get_motor_speed.ch2, get_motor_speed.ch3))
        {
            g_get_motor_speed = get_motor_speed;
        }
        // 注意：这里没有delay，表示尽可能快地轮询CAN数据
        // 由于CAN驱动内部有阻塞机制，不会占用全部CPU
    }
}

//CAN发送任务
void CanSendTask(void *parameter)
{
    for (;;)
    {
        PBalanceControl::MotorControl send_motor_control;
        if (xQueueReceive(xQueueCanSend, &send_motor_control, portMAX_DELAY) == pdTRUE)
        {
            can_.CanSendMotorSpeed(
                send_motor_control.ch1,
                send_motor_control.ch2,
                send_motor_control.ch3,
                send_motor_control.value,
                send_motor_control.control_mode);
        }
    }
}

struct ImuRotationAngle_t
{
    float x;
    float y;
    float z;
};

ImuRotationAngle_t imu_rotation_angle = {0, -40, 0};

void ImuTask(void *parameter)
{
    ConvertAngularVelocity gyro_conv;
    for (;;)
    {
        // 运行AHRS姿态解算算法
        // 该函数内部会读取IMU原始数据并计算姿态角
        ahrs_ptr->AhrsLoop();
        // 角速度坐标转换
        // 将IMU坐标系的角速度转换到机器人本体坐标系
        gyro_conv.Loop(
            imu_rotation_angle.y,
            ahrs_ptr->imu_data_.gyro.gy,
            ahrs_ptr->imu_data_.gyro.gx,
            ahrs_ptr->imu_data_.gyro.gz,
            ahrs_ptr->imu_data_.gyro.gy,
            ahrs_ptr->imu_data_.gyro.gz);
        ahrs_ptr->imu_data_.gyro.gz = -ahrs_ptr->imu_data_.gyro.gz;
        xQueueSend(xQueueImu, &ahrs_ptr->imu_data_, 0);
        // 注意：此任务没有delay，以最高频率运行
        // AHRS算法通常需要固定周期运行（如1kHz）
    }
}

//运行模式枚举
enum RunMode_e
{
    NORMAL_MODE = 0x00,  // 普通模式
    U_JUMP_MODE,         // 边起跳模式
    U_JUMPING_MODE,      // 起跳中
    U_BALANCE_MODE,      // 边平衡模式
    POINT_JUMP_MODE,     // 点起跳模式
    POINT_JUMPING_MODE,  // 点起跳中
    POINT_BALANCE_MODE   // 点平衡
};

int mode          = NORMAL_MODE;
float yaw_control = 0.0f;  // 控制yaw的自旋
float slot_param  = 1.3;   // 刹车系数

// 单个电机的起跳参数
struct MotorjumpValue_t
{
    float set_motor_speed;
    int set_servo_pos;
    int servo_int_pos;
};

// 三个电机的起跳参数集合
struct jumpValue_t
{
    MotorjumpValue_t m1;
    MotorjumpValue_t m2;
    MotorjumpValue_t m3;
};

// 起跳参数初始化
jumpValue_t jump_value = {
    .m1 {-340, 100, 70},
     .m2 {-340, 100, 56},
     .m3 {-570, 100, 62}
};

//WiFi命令任务
void WifiCommanderTask(void *parameter)
{
    for (;;)
    {
        wifi_commander_ptr->WaitClientConnect();
        wifi_commander_ptr->TcpClientUnpack();
    }
}

void CmdTask(void *parameter)
{
    CubliMini::Control::SerialCommander serial_commander;
    wifi_commander_ptr = new CubliMini::Control::WifiCommander();

    REGISTER_COMMAND_FLOAT(serial_commander, "M", mode);
    AxisParam_t &u_param = Param::GetInstance()->GetUAxisParam();
    REGISTER_COMMAND_FLOAT(serial_commander, "SUA", u_param.angle_offset);
    REGISTER_COMMAND_FLOAT(serial_commander, "SUP", u_param.kp);
    REGISTER_COMMAND_FLOAT(serial_commander, "SUV", u_param.kv);
    REGISTER_COMMAND_FLOAT(serial_commander, "SUS", u_param.ks);

    PBalanceControl::PAxisParam &p_param = Param::GetInstance()->GetPAxisParam();
    REGISTER_COMMAND_FLOAT(serial_commander, "SPXA", p_param.x.angle_offset);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPXP", p_param.x.kp);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPXV", p_param.x.kv);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPXS", p_param.x.ks);

    REGISTER_COMMAND_FLOAT(serial_commander, "SPYA", p_param.y.angle_offset);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPYP", p_param.y.kp);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPYV", p_param.y.kv);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPYS", p_param.y.ks);

    REGISTER_COMMAND_FLOAT(serial_commander, "SPZA", p_param.z.angle_offset);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPZP", p_param.z.kp);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPZV", p_param.z.kv);
    REGISTER_COMMAND_FLOAT(serial_commander, "SPZS", p_param.z.ks);

    REGISTER_COMMAND_FLOAT(serial_commander, "M1_JUMP_SPEED", jump_value.m1.set_motor_speed);
    REGISTER_COMMAND_FLOAT(serial_commander, "M2_JUMP_SPEED", jump_value.m2.set_motor_speed);
    REGISTER_COMMAND_FLOAT(serial_commander, "M3_JUMP_SPEED", jump_value.m3.set_motor_speed);

    REGISTER_COMMAND_FLOAT(serial_commander, "IMU_ROTATION_X", imu_rotation_angle.x);
    REGISTER_COMMAND_FLOAT(serial_commander, "IMU_ROTATION_Y", imu_rotation_angle.y);
    REGISTER_COMMAND_FLOAT(serial_commander, "IMU_ROTATION_Z", imu_rotation_angle.z);

    REGISTER_COMMAND_FLOAT(serial_commander, "YAW_CONTROL", yaw_control);
    REGISTER_COMMAND_FLOAT(serial_commander, "slot_param", slot_param);
    REGISTER_COMMAND_FLOAT(serial_commander, "cal_angle_kp", cal_angle_kp);

    // ========== 注册舵机控制命令（使用Lambda表达式） ==========
    serial_commander.Register("SERVO1", [&](const std::string &value) {
        float modeVariable = std::stof(value);
        servo_motor1.setDegree(modeVariable);
        jump_value.m1.servo_int_pos = modeVariable;
        printf("SERVO1 setDegree: %.3f\n", modeVariable);
    });

    serial_commander.Register("SERVO2", [&](const std::string &value) {
        float modeVariable          = std::stof(value);
        jump_value.m2.servo_int_pos = modeVariable;
        servo_motor2.setDegree(modeVariable);
        printf("SERVO2 setDegree: %.3f\n", modeVariable);
    });

    serial_commander.Register("SERVO3", [&](const std::string &value) {
        float modeVariable = std::stof(value);
        servo_motor3.setDegree(modeVariable);
        jump_value.m3.servo_int_pos = modeVariable;
        printf("SERVO3 setDegree: %.3f\n", modeVariable);
    });

    // ========== 注册电机速度测试命令 ==========
    serial_commander.Register("SET_MOTOR1_SPEED", [&](const std::string &value) {
        float modeVariable = std::stof(value);
        printf("MOTOR1 set speed: %d rad/s\n", modeVariable);
    });

    serial_commander.Register("SET_MOTOR2_SPEED", [&](const std::string &value) {
        float modeVariable = std::stof(value);
        printf("MOTOR2 set speed: %d rad/s\n", modeVariable);
    });

    serial_commander.Register("SET_MOTOR3_SPEED", [&](const std::string &value) {
        float modeVariable = std::stof(value);
        printf("MOTOR3 set speed: %d rad/s\n", modeVariable);
    });

    serial_commander.Register("SET_SERVO3_ANGLE", [&](const std::string &value) {
        float modeVariable = std::stof(value);
        printf("SERVO3_ANGLE set angle: %0.3f d\n", modeVariable);
    });

    // ========== 注册参数查询命令 ==========
    // "GU" - 获取U轴参数
    serial_commander.Register("GU", [](const std::string &value) {
        AxisParam_t &param = Param::GetInstance()->GetUAxisParam();
        printf(
            "GU angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f]\n",
            param.angle_offset,
            param.kp,
            param.kv,
            param.ks);
    });

    // "GP" - 获取P轴参数（三轴）
    serial_commander.Register("GP", [](const std::string &value) {
        PBalanceControl::PAxisParam &param = Param::GetInstance()->GetPAxisParam();
        printf(
            "GP\n x angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n y "
            "angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n z angle_offset:[%0.3f] "
            "kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n",
            param.x.angle_offset,
            param.x.kp,
            param.x.kv,
            param.x.ks,
            param.y.angle_offset,
            param.y.kp,
            param.y.kv,
            param.y.ks,
            param.z.angle_offset,
            param.z.kp,
            param.z.kv,
            param.z.ks);
    });

    // ========== 注册参数保存命令 ==========
    // "SAVE_P" - 保存P轴参数到EEPROM
    serial_commander.Register("SAVE_P", [](const std::string &value) {
        PBalanceControl::PAxisParam param = Param::GetInstance()->GetPAxisParam();
        printf(
            "SAVE\nP\n x angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n y "
            "angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n z angle_offset:[%0.3f] "
            "kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n",
            param.x.angle_offset,
            param.x.kp,
            param.x.kv,
            param.x.ks,
            param.y.angle_offset,
            param.y.kp,
            param.y.kv,
            param.y.ks,
            param.z.angle_offset,
            param.z.kp,
            param.z.kv,
            param.z.ks);
        Param::GetInstance()->SavePBalanceParam();
    });

    // "SAVE_U" - 保存U轴参数到EEPROM
    serial_commander.Register("SAVE_U", [](const std::string &value) {
        AxisParam_t u_param = Param::GetInstance()->GetUAxisParam();
        printf(
            "u\n angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f]\n",
            u_param.angle_offset,
            u_param.kp,
            u_param.kv,
            u_param.ks);
        Param::GetInstance()->SaveUBalanceParam();
    });

    // "RESET" - 重置所有参数到默认值
    serial_commander.Register("RESET", [](const std::string &value) {
        Param::GetInstance()->FirstWriteParamToEeprom();
        Param::GetInstance()->LoadParam();
        PBalanceControl::PAxisParam param = Param::GetInstance()->GetPAxisParam();
        printf(
            "SAVE\nP\n x angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n y "
            "angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n z angle_offset:[%0.3f] "
            "kp:[%0.3f] kv:[%0.3f] ks:[%0.3f] \n",
            param.x.angle_offset,
            param.x.kp,
            param.x.kv,
            param.x.ks,
            param.y.angle_offset,
            param.y.kp,
            param.y.kv,
            param.y.ks,
            param.z.angle_offset,
            param.z.kp,
            param.z.kv,
            param.z.ks);
        AxisParam_t u_param = Param::GetInstance()->GetUAxisParam();
        printf(
            "u\n angle_offset:[%0.3f] kp:[%0.3f] kv:[%0.3f] ks:[%0.3f]\n",
            u_param.angle_offset,
            u_param.kp,
            u_param.kv,
            u_param.ks);
    });

    // ========== WiFi连接 ==========
    wifi_commander_ptr->SetCmds(serial_commander.GetCmds());
    wifi_commander_ptr->ConnectWifi();
    if (wifi_commander_ptr->GetConnectStatus() == WifiConnectStatus_e::eWIFI_SUCCESS)
    {
        // core 0
        xTaskCreatePinnedToCore(WifiCommanderTask, "WifiCommanderTask", 30000, NULL, 7, NULL, 0);
    }

    // ========== 主循环 ==========
    uint32_t count = 0;  // 循环计数器
    for (;;)
    {
        ++count;
        // 运行串口命令解析器，检查是否有新命令
        serial_commander.Run(Serial);
        // 每10次循环打印一次IMU数据（约每100ms）
        if (count % 10 == 0)
        {
            float angle_d = ahrs_ptr->imu_data_.angle.pitch -
                            Param::GetInstance()->GetUAxisParam().angle_offset;

            // 打印6个数据：俯仰角、横滚角、偏航角、三轴角速度
            printf(
                "%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n",
                ahrs_ptr->imu_data_.angle.pitch,  // 俯仰角
                ahrs_ptr->imu_data_.angle.roll,  // 横滚角
                ahrs_ptr->imu_data_.angle.yaw,  // 偏航角
                ahrs_ptr->imu_data_.gyro.gy,
                ahrs_ptr->imu_data_.gyro.gx,
                ahrs_ptr->imu_data_.gyro.gz);
        }
        // ========== 按键检测（暂不使用） ==========
        {
            // 暂不使用
            KeyAction_e key1_action = key1.GetKeyAction();
            KeyAction_e key2_action = key2.GetKeyAction();

            if (key1_action == KeyAction_e::eKEY_ACTION_SHORT_PRESS)
            {
                printf("key1 eKEY_ACTION_SHORT_PRESS\n");
            }

            if (key1_action == KeyAction_e::eKEY_ACTION_LONG_PRESS)
            {
                printf("key1 eKEY_ACTION_LONG_PRESS\n");
            }

            if (key2_action == KeyAction_e::eKEY_ACTION_SHORT_PRESS)
            {
                printf("key2 eKEY_ACTION_SHORT_PRESS\n");
            }

            if (key2_action == KeyAction_e::eKEY_ACTION_LONG_PRESS)
            {
                printf("key2 eKEY_ACTION_LONG_PRESS\n");
            }
        }
        delay(10);
    }
}

void ControlTask(void *parameter)
{
    // ========== 控制器参数初始化 ==========
    // 设置角度自动校准的步长（每次调整-0.01度）
    U_balance_control.angle_offset_auto_cal_.angle_update_step_size_   = -0.01f;
    p_balance_control.x_angle_offset_auto_cal_.angle_update_step_size_ = -0.01f;

    // ========== 舵机初始化到中间位置 ==========
    servo_motor1.setDegree(jump_value.m1.servo_int_pos);
    servo_motor2.setDegree(jump_value.m2.servo_int_pos);
    servo_motor3.setDegree(jump_value.m3.servo_int_pos);

    // ========== 时间计时器 ==========
    CubliMini::Comm::CumulativeTime servo_protection_time;
    CubliMini::Comm::CumulativeTime jump_up_time;
    CubliMini::Comm::CumulativeTime jump_up_acceleration_time;

    // ========== 点起跳相关变量 ==========
    bool do_point_jump_flag             = false;  // 是否执行点起跳标志
    AxisParam_t default_point_x = Param::GetInstance()->GetPAxisParam().x;
    AxisParam_t default_point_y = Param::GetInstance()->GetPAxisParam().y;
    bool point_jump_flag        = false;  // 点起跳完成标志
    
    // ========== 主控制循环 ==========
    for (;;)
    {
        ImuData_t rx_imu_data;
        PBalanceControl::MotorControl send_motor_control = {0, 0, 0, 0};
        MotorTransMode motor_trans_mode                  = {TOUCH_e, TOUCH_e, TOUCH_e};

        // ========== 等待IMU数据（最多2000ms） ==========
        // if (xQueueReceive(xQueueCanGet, &get_motor_speed, portMAX_DELAY) == pdTRUE)
        if (xQueueReceive(xQueueImu, &rx_imu_data, 2000) == pdTRUE) // wait max 2000ms
        {
            // ========== 构建P轴传感器数据 ==========
            PBalanceControl::PSensor p_sensor = {
                {rx_imu_data.angle.pitch, rx_imu_data.gyro.gy, 0},
                {rx_imu_data.angle.roll,  rx_imu_data.gyro.gx, 0},
                {rx_imu_data.angle.yaw,   rx_imu_data.gyro.gz, 0}
            };

            // ========== 构建电机速度数据 ==========
            PBalanceControl::MotorSpeed p_motor_speed {
                -g_get_motor_speed.ch1, 
                g_get_motor_speed.ch3, 
                g_get_motor_speed.ch2};

            // ========== 构建U轴传感器数据 ==========
            AxisSensor_t u_sensor = {
                rx_imu_data.angle.pitch, rx_imu_data.gyro.gz, g_get_motor_speed.ch3};

            // ========== 状态机：普通模式 ==========
            if (mode == NORMAL_MODE)
            {
                //舵机复位
                jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                jump_value.m3.set_servo_pos = jump_value.m3.servo_int_pos;
                //电机输出为0
                send_motor_control.ch1      = 0;
                send_motor_control.ch2      = 0;
                send_motor_control.ch3      = 0;
                jump_up_acceleration_time.reset();//重置加速计时
                do_point_jump_flag = false;
            }
            // ========== 状态机：边起跳模式 ==========
            else if (mode == U_JUMP_MODE)
            {
                motor_trans_mode.motor3_mode = VELOCITY_e;
                send_motor_control.ch3       = jump_value.m3.set_motor_speed;
                if (jump_up_acceleration_time.GetTimeS() > 1.5f ||
                    fabs(p_motor_speed.ch3) >= fabs(jump_value.m3.set_motor_speed) - 10)
                {
                    // motor_trans_mode.motor3_mode = BRAKE_e;
                    jump_up_acceleration_time.reset();//重置计时
                    jump_value.m3.set_servo_pos = jump_value.m3.servo_int_pos - 25;
                    servo_motor3.setDegree(jump_value.m3.set_servo_pos);
                    servo_protection_time.reset();
                    send_motor_control.ch3 = -jump_value.m3.set_motor_speed;
                    mode                   = U_JUMPING_MODE;  // 进入起跳中状态
                }
            }
            // ========== 状态机：边起跳中 ==========
            else if (mode == U_JUMPING_MODE)
            {
                motor_trans_mode.motor3_mode    = VELOCITY_e;
                send_motor_control.control_mode = 1;  // 控制模式标志
                // 0.2秒后停止电机
                if (servo_protection_time.GetTimeS() >= 0.2f)
                {
                    send_motor_control.ch3 = 0;
                }
                // 0.7秒后复位舵机，返回普通模式
                if (servo_protection_time.GetTimeS() >= 0.7f)
                {
                    send_motor_control.ch3      = 0;
                    jump_value.m3.set_servo_pos = jump_value.m3.servo_int_pos;
                    servo_motor3.setDegree(jump_value.m3.set_servo_pos);
                    mode = NORMAL_MODE;
                }
                // 如果速度降到50以下，停止电机
                if (fabs(p_motor_speed.ch3) <= 50)
                    send_motor_control.ch3 = 0;
                // 检查是否成功站立（角度偏差小于15度）
                if (fabs(u_sensor.angle - Param::GetInstance()->GetUAxisParam().angle_offset) <
                    15.0f)
                {
                    jump_value.m3.set_servo_pos = jump_value.m3.servo_int_pos;
                    servo_motor3.setDegree(jump_value.m3.set_servo_pos);
                    send_motor_control.ch3 = 0;
                    mode                   = U_BALANCE_MODE;  // 进入边平衡模式
                }
            }
            // ========== 状态机：边平衡模式 或 点起跳准备 ==========
            if (mode == U_BALANCE_MODE || mode == POINT_JUMP_MODE)
            {
                send_motor_control.control_mode = 1;
                // 运行U轴平衡控制器
                send_motor_control.ch3          = U_balance_control.Loop(u_sensor, true);
                send_motor_control.ch1          = 0;
                send_motor_control.ch2          = 0;
            }
            // ========== 状态机：点起跳模式 ==========
            if (mode == POINT_JUMP_MODE)
            {
                send_motor_control.control_mode = 0;  // 直接控制模式
                if (do_point_jump_flag == false)
                {
                    do_point_jump_flag = true;
                    jump_up_acceleration_time.reset();  // 开始计时
                }
                // 设置舵机到初始位置
                jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                servo_motor2.setDegree(jump_value.m2.set_servo_pos);

                // 两个电机加速旋转
                motor_trans_mode.motor2_mode = VELOCITY_e;
                motor_trans_mode.motor1_mode = VELOCITY_e;
                send_motor_control.ch2       = jump_value.m2.set_motor_speed;
                send_motor_control.ch1       = jump_value.m1.set_motor_speed;

                // 加速1秒后起跳
                if (jump_up_acceleration_time.GetTimeS() > 1.0f)
                {
                    motor_trans_mode.motor2_mode = BRAKE_e;// 刹车模式
                    motor_trans_mode.motor1_mode = BRAKE_e;
                    send_motor_control.ch3       = 0;
                    send_motor_control.ch2       = 0;
                    send_motor_control.ch1       = 0;

                    // 舵机角度变化
                    jump_value.m1.set_servo_pos  = jump_value.m1.servo_int_pos - 15;
                    jump_value.m2.set_servo_pos  = jump_value.m2.servo_int_pos - 15;
                    servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                    servo_motor2.setDegree(jump_value.m2.set_servo_pos);
                    jump_up_acceleration_time.reset();
                    mode            = POINT_JUMPING_MODE;  // 进入点起跳中状态

                    // 保存当前参数用于恢复
                    default_point_x = Param::GetInstance()->GetPAxisParam().x;
                    default_point_y = Param::GetInstance()->GetPAxisParam().y;
                    point_jump_flag = true;
                }
            }

            // ========== 状态机：点起跳中 ==========
            if (mode == POINT_JUMPING_MODE)
            {
                do_point_jump_flag = false;
                //0.7s后复位舵机
                if (jump_up_acceleration_time.GetTimeS() > 0.7f)
                {
                    jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                    jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                    servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                    servo_motor2.setDegree(jump_value.m2.set_servo_pos);
                }

                // 计算Y轴角度偏差
                float y_delta_angle =
                    p_sensor.y.angle - Param::GetInstance()->GetPAxisParam().y.angle_offset;

                // 检测是否成功站立（三种条件）
                // 条件1：Y轴角速度大于100且角度偏差大于-8度
                    if (p_sensor.y.gyro > 100 && y_delta_angle > -8)
                {
                    jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                    jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                    servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                    servo_motor2.setDegree(jump_value.m2.set_servo_pos);
                    Param::GetInstance()->GetPAxisParam().y.kv *= slot_param;// 增大阻尼系数帮助稳定
                    mode = POINT_BALANCE_MODE;
                }
                // 条件2：角速度小于10且角度偏差小于8度
                else if (p_sensor.y.gyro < 10 && fabs(y_delta_angle) < 8)
                {
                    jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                    jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                    servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                    servo_motor2.setDegree(jump_value.m2.set_servo_pos);
                    Param::GetInstance()->GetPAxisParam().y.kv *= slot_param;
                    mode = POINT_BALANCE_MODE;
                }
                // 条件3：X和Y角度偏差都在允许范围内
                if (fabs(p_sensor.x.angle - Param::GetInstance()->GetPAxisParam().x.angle_offset) <
                        8 &&
                    fabs(p_sensor.y.angle - Param::GetInstance()->GetPAxisParam().y.angle_offset) <
                        4)
                {
                    jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                    jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                    servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                    servo_motor2.setDegree(jump_value.m2.set_servo_pos);
                    Param::GetInstance()->GetPAxisParam().y.kv *= slot_param;
                    mode = POINT_BALANCE_MODE;
                }
            }
            // ========== 状态机：点平衡模式 ==========
            if (mode == POINT_BALANCE_MODE)
            {
                // 如果角度偏差小于4度，恢复正常参数
                if (fabs(p_sensor.y.angle - Param::GetInstance()->GetPAxisParam().y.angle_offset) <
                        4 &&
                    point_jump_flag)
                {
                    point_jump_flag                         = false;
                    Param::GetInstance()->GetPAxisParam().y = default_point_y;// 恢复原始KV值
                }

                // 0.7秒后复位舵机
                if (jump_up_acceleration_time.GetTimeS() > 0.7f)
                {
                    jump_value.m1.set_servo_pos = jump_value.m1.servo_int_pos;
                    jump_value.m2.set_servo_pos = jump_value.m2.servo_int_pos;
                    servo_motor1.setDegree(jump_value.m1.set_servo_pos);
                    servo_motor2.setDegree(jump_value.m2.set_servo_pos);
                }

                // 运行P轴平衡控制器
                PBalanceControl::MotorControl motor_control =
                    p_balance_control.Loop(p_sensor, p_motor_speed, true, yaw_control);

                // 重新映射电机输出
                send_motor_control.ch1 = -motor_control.ch1;
                send_motor_control.ch2 = motor_control.ch3;
                send_motor_control.ch3 = motor_control.ch2;

                // 检查是否失去平衡（角度偏差过大）
                if (fabs(p_sensor.x.angle - Param::GetInstance()->GetPAxisParam().x.angle_offset) >
                        15 &&
                    fabs(p_sensor.y.angle - Param::GetInstance()->GetPAxisParam().y.angle_offset) >
                        15)
                {
                    mode = NORMAL_MODE;
                }
            }
            // ========== 发送控制指令到队列 ==========
            send_motor_control.value = MotorTransMode::GetValue(motor_trans_mode);
            xQueueSend(xQueueCanSend, &send_motor_control, 0);
        }
    }
}

//LED彩虹效果函数 
// 色轮转换函数：将0-255的位置转换为RGB颜色
uint32_t Wheel(byte WheelPos)
{
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85)
    {
        return system_strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170)
    {
        WheelPos -= 85;
        return system_strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return system_strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

// Fill the dots one after the other with a color
// 颜色填充函数：逐个点亮LED
void colorWipe(uint32_t c, uint8_t wait)
{
    for (uint16_t i = 1; i < system_strip.numPixels(); i++)
    {
        system_strip.setPixelColor(i, c);
        system_strip.show();
        delay(wait);
    }
}

//// 彩虹效果：所有LED同时变色
void rainbow(uint8_t wait)
{
    uint16_t i, j;

    for (j = 0; j < 256; j++)  //// 遍历所有颜色
    {
        for (i = 1; i < system_strip.numPixels(); i++)
        {
            system_strip.setPixelColor(i, Wheel((i + j) & 255));
        }
        system_strip.show();
        delay(wait);
    }
}

// Slightly different, this makes the rainbow equally distributed throughout
// 彩虹循环效果：彩虹在灯带上流动
void rainbowCycle(uint8_t wait)
{
    uint16_t i, j;

    for (j = 0; j < 256 * 5; j++)
    {  // 5 cycles of all colors on wheel
        for (i = 1; i < system_strip.numPixels(); i++)
        {
            system_strip.setPixelColor(i, Wheel(((i * 256 / system_strip.numPixels()) + j) & 255));
        }
        system_strip.show();
        delay(wait);
    }
}

// Theatre-style crawling lights.
// 剧院追逐效果：闪烁的灯光移动
void theaterChase(uint32_t c, uint8_t wait)
{
    for (int j = 0; j < 10; j++)
    {  // do 10 cycles of chasing
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 1; i < system_strip.numPixels(); i = i + 3)
            {
                system_strip.setPixelColor(i + q, c);  // turn every third pixel on
            }
            system_strip.show();

            delay(wait);

            for (uint16_t i = 1; i < system_strip.numPixels(); i = i + 3)
            {
                system_strip.setPixelColor(i + q, 0);  // turn every third pixel off
            }
        }
    }
}

// Theatre-style crawling lights with rainbow effect
// 彩虹剧院追逐效果
void theaterChaseRainbow(uint8_t wait)
{
    for (int j = 0; j < 256; j++)
    {  // cycle all 256 colors in the wheel
        for (int q = 0; q < 3; q++)
        {
            for (uint16_t i = 1; i < system_strip.numPixels(); i = i + 3)
            {
                system_strip.setPixelColor(
                    i + q, Wheel((i + j) % 255));  // turn every third pixel on
            }
            system_strip.show();

            delay(wait);

            for (uint16_t i = 1; i < system_strip.numPixels(); i = i + 3)
            {
                system_strip.setPixelColor(i + q, 0);  // turn every third pixel off
            }
        }
    }
}

//电机LED控制类 
class MotorLedControl
{
    struct LedColor
    {
        uint16_t r;
        uint16_t g;
        uint16_t b;
    };

   public:
    MotorLedControl() = default;
    MotorLedControl(Adafruit_NeoPixel *strip) : motor_strip_(strip) {}

    void StaticControl(float motor_speed)
    {
        offset = (offset + 1) % (5 + 2);  // 每次调用 loop() 函数时，offset 增加 1

        for (int i = 0; i < motor_strip_->numPixels(); ++i)
        {
            // 根据偏移量和位置确定颜色
            int index = (i + offset) % (5 + 2);
            if (index < 5)
            {
                // 主色区域
                motor_strip_->setPixelColor(i, motor_strip_->Color(255, 0, 0));  // 红色
            }
            else
            {
                // 过渡区域
                float transition = (float)(index - 5) / 2;
                // 过渡色，这里以红色到绿色的渐变为例
                uint8_t red   = 255 * (1 - transition);
                uint8_t green = 255 * transition;
                motor_strip_->setPixelColor(i, motor_strip_->Color(red, green, 0));
            }
        }

        motor_strip_->show();  // 更新 LED 灯带的显示
    }

    void RuningControl(float motor_speed)
    {
        LedColor rgb {0, 0, 0};

        // 根据速度计算亮起的LED数量
        int index  = fabs(motor_speed) / 5;  // 每5单位速度点亮一个LED
        int index1 = fabs(motor_speed) / 5;
        if (index1 > motor_strip_->numPixels())
            index1 = motor_strip_->numPixels();
        int temp = 50 + index * 10;

        if (temp <= 255)
        {
            // 低速：红色
            rgb.r = temp;
        }
        else if (temp > 255 && temp <= 255 * 2)
        {
            // 中速：红色 + 蓝色
            int get_value = (uint8_t)(100 + temp % 255);
            if (get_value > 255)
                get_value = 255;
            rgb.r = 220;
            rgb.b = get_value;
        }
        else if (temp > 255 * 2)
        {
            // 高速：红色 + 蓝色 + 绿色
            int get_value = (uint8_t)(100 + temp % 255);
            if (get_value > 255)
                get_value = 255;
            rgb.r = 220;
            rgb.b = 220;
            rgb.g = get_value;
        }

        for (int i = 0; i < index1; ++i)
        {
            motor_strip_->setPixelColor(i, motor_strip_->Color(rgb.r, rgb.g, rgb.b));
        }
        for (int i = index1; i < motor_strip_->numPixels(); ++i)
        {
            motor_strip_->setPixelColor(i, motor_strip_->Color(0, 0, 0));
        }
        motor_strip_->show();
    }

    void Loop(float motor_speed, bool is_static)
    {
        // if (is_static)
        // {
        //     StaticControl(motor_speed);
        //     delay(100);
        // }
        // else
        {
            RuningControl(motor_speed);
            // delay(10);
        }
    }

   private:
    Adafruit_NeoPixel *motor_strip_;
    int offset = 0;
};

void MotorLedTask(void *parameter)
{
    // 为每个电机创建LED控制器
    MotorLedControl motor1_led_control(&motor1_strip);
    MotorLedControl motor2_led_control(&motor2_strip);
    MotorLedControl motor3_led_control(&motor3_strip);
    for (;;)
    {
        motor1_led_control.Loop(g_get_motor_speed.ch1, ahrs_ptr->IsStatic());
        motor2_led_control.Loop(g_get_motor_speed.ch2, ahrs_ptr->IsStatic());
        motor3_led_control.Loop(g_get_motor_speed.ch3, ahrs_ptr->IsStatic());
        delay(20);
    }
}

void loop()
{
    rainbow(20);
    rainbowCycle(20);
}