/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"

static Referee_Interactive_info_t *Interactive_data; // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;            // 接收到的裁判系统数据
uint8_t UI_Seq;                                      // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID; // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data); // 模式切换检测
// static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data); // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle); // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                            // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    // RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

static String_Data_t UI_State_sta[6]; // 机器人模式状态,静态只需画一次 静态(始终存在画面)
static String_Data_t UI_State_dyn[6]; // 机器人模式状态,动态先add才能change（始终存在画面）

static Graph_Data_t UI_body_line[4]; // 车身准线 （有两条 来回改）
static Graph_Data_t UI_Pump[5];      // 泵和阀的状态
// 车身准线起始xy坐标
static uint32_t body_line_start_location[10] = {0, 0,     // 0,1 正常模式 左线
                                                311, 0,   // 2,3 取矿行进模式 左线
                                                0, 0,     // 4,5 正常模式 右线
                                                1557, 0}; // 6,7 取矿行进模式 右线
// 车身准线终点xy坐标
static uint32_t body_line_end_location[10] = {0, 0,
                                              722, 390,
                                              0, 0,
                                              1151, 390};

static Graph_Data_t UI_get_line[4]; // 取矿准线 （一条来回改）
// 取矿准线起始xy坐标
static uint32_t get_line_start_location[10] = {540, 960,  // 0,1 开采银矿模式
                                               444, 555,  // 2,3 一键开采三个银矿模式 中间的银矿框的起始坐标
                                               666, 777,  // 4,5 开采金矿模式
                                               888, 999}; // 6,7 开采地面矿模式
// 取矿准线终点xy坐标
static uint32_t get_line_end_location[10] = {540, 960,
                                             10, 50,
                                             60, 100,
                                             110, 150};

static Graph_Data_t UI_shoot_line[10]; // 射击准线
static Graph_Data_t UI_Energy[3];      // 电容能量条
static uint32_t shoot_line_location[10] = {540, 960, 490, 515, 565};
#define word_width 3

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL); // 如果没有初始化裁判系统则直接删除ui任务
    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100); // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                            // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0); // 清空UI

    // 绘制车辆状态标志指示，静态  （保留）
    UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_ADD, 8, UI_Color_Orange, 20, 2, 5, 840, "pump:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
    UICharDraw(&UI_State_sta[1], "ss1", UI_Graph_ADD, 8, UI_Color_Black, 15, 2, 0, 750, "chassis:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[1]);
    UICharDraw(&UI_State_sta[2], "ss2", UI_Graph_ADD, 8, UI_Color_Black, 15, 2, 0, 700, "upper:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[2]);
    UICharDraw(&UI_State_sta[3], "ss3", UI_Graph_ADD, 8, UI_Color_Black, 15, 2, 0, 650, "gimbal:");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[3]);
    // UICharDraw(&UI_State_sta[4], "ss4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 550, "lid:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[4]);

    // 绘制车辆状态标志，动态  （保留）
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 5, 840, "close");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 120, 750, "ZEROFORCE");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 150, 700, "UPPER_ZERO_FORCE");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 150, 650, "GIMBAL_ZERO_FORCE");
    UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    // UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 550, "open ");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // 气泵和电磁阀，动态
    UICircleDraw(&UI_Pump[0], "pmp", UI_Graph_ADD, 7, UI_Color_Main, 2, 50, 850, 25); // 满的（填满）参数是25，50，850，25
    UICircleDraw(&UI_Pump[1], "vl1", UI_Graph_ADD, 7, UI_Color_Main, 2, 50, 850, 25); // 满的（填满）参数是25，50，850，25
    UICircleDraw(&UI_Pump[2], "vl2", UI_Graph_ADD, 7, UI_Color_Main, 2, 50, 850, 25); // 满的（填满）参数是25，50，850，25
    UICircleDraw(&UI_Pump[3], "vl3", UI_Graph_ADD, 7, UI_Color_Main, 2, 50, 850, 25); // 满的（填满）参数是25，50，850，25
    UICircleDraw(&UI_Pump[4], "vl4", UI_Graph_ADD, 7, UI_Color_Main, 2, 50, 850, 25); // 满的（填满）参数是25，50，850，25
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_Energy[0], UI_Energy[1], UI_Energy[2], UI_Energy[3], UI_Energy[4]);

    // 车身准线，（根据模式切换）
    UILineDraw(&UI_body_line[0], "sb0", UI_Graph_ADD, 7, UI_Color_Yellow, 3, body_line_start_location[0], body_line_start_location[1], body_line_end_location[0], body_line_end_location[1]);
    UILineDraw(&UI_body_line[1], "sb1", UI_Graph_ADD, 7, UI_Color_Yellow, 3, body_line_start_location[4], body_line_start_location[5], body_line_end_location[4], body_line_end_location[5]);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_body_line[0], UI_body_line[1]);
    // 取矿准线
    UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_ADD, 7, UI_Color_Yellow, 3, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_get_line[0]);

    // // 绘制发射基准线
    // UILineDraw(&UI_shoot_line[0], "sl0", UI_Graph_ADD, 7, UI_Color_White, 3, 710, shoot_line_location[0], 1210, shoot_line_location[0]);
    // UILineDraw(&UI_shoot_line[1], "sl1", UI_Graph_ADD, 7, UI_Color_White, 3, shoot_line_location[1], 340, shoot_line_location[1], 740);
    // UILineDraw(&UI_shoot_line[2], "sl2", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[2], 1110, shoot_line_location[2]);
    // UILineDraw(&UI_shoot_line[3], "sl3", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[3], 1110, shoot_line_location[3]);
    // UILineDraw(&UI_shoot_line[4], "sl4", UI_Graph_ADD, 7, UI_Color_Yellow, 2, 810, shoot_line_location[4], 1110, shoot_line_location[4]);
    // UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_shoot_line[0], UI_shoot_line[1], UI_shoot_line[2], UI_shoot_line[3], UI_shoot_line[4]);

    // // 底盘功率显示，静态
    // UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // // 底盘功率显示,动态
    // UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);

    // // 能量条框，静态
    // UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1220, 180);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);
    // // 能量条初始状态，动态
    // UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
}

/* // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
static uint8_t count = 0;
static uint16_t count1 = 0;
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data) // 测试用函数，实现模式自动变化
{
    count++;
    if (count >= 50)
    {
        count = 0;
        count1++;
    }
    switch (count1 % 4)
    {
    case 0:
    {
        _Interactive_data->chassis_mode = CHASSIS_ZERO_FORCE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        _Interactive_data->Chassis_Power_Data.chassis_power_mx += 3.5;
        if (_Interactive_data->Chassis_Power_Data.chassis_power_mx >= 18)
            _Interactive_data->Chassis_Power_Data.chassis_power_mx = 0;
        break;
    }
    case 1:
    {
        _Interactive_data->chassis_mode = CHASSIS_NORMAL;
        _Interactive_data->gimbal_mode = GIMBAL_FREE_MODE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    case 2:
    {
        _Interactive_data->chassis_mode = CHASSIS_MINING;
        _Interactive_data->gimbal_mode = GIMBAL_GYRO_MODE;
        _Interactive_data->shoot_mode = SHOOT_ON;
        _Interactive_data->friction_mode = FRICTION_ON;
        _Interactive_data->lid_mode = LID_OPEN;
        break;
    }
    case 3:
    {
        _Interactive_data->chassis_mode = CHASSIS_NO_MOVE;
        _Interactive_data->gimbal_mode = GIMBAL_ZERO_FORCE;
        _Interactive_data->shoot_mode = SHOOT_OFF;
        _Interactive_data->friction_mode = FRICTION_OFF;
        _Interactive_data->lid_mode = LID_CLOSE;
        break;
    }
    default:
        break;
    }
} */

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // 模式切换显示部分
    //  pump
    if (_Interactive_data->Referee_Interactive_Flag.pump_flag == 1)
    {
        if (_Interactive_data->pump_mode & VAVLVE_ARM)
        {
            // code
        }
        else
        {
            // code
        }
        if (_Interactive_data->pump_mode & VAVLVE_T1)
        {
            // code
        }
        else
        {
            // code
        }
        if (_Interactive_data->pump_mode & VAVLVE_T2)
        {
            // code
        }
        else
        {
            // code
        }
        if (_Interactive_data->pump_mode & VAVLVE_T3)
        {
            // code
        }
        else
        {
            // code
        }
        if (_Interactive_data->pump_mode)
        {
            // code
        }
        else
        {
            // code
        }

        switch (_Interactive_data->pump_mode)
        {
        case 1:
            UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_Change, 8, UI_Color_Orange, 0, 2, 5, 840, "pump:");
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Orange, 0, 2, 5, 840, "close");
            UICircleDraw(&UI_Energy[1], "sd6", UI_Graph_Change, 7, UI_Color_Yellow, 30, 50, 850, 20); // 满的（填满）参数是25，50，850，25
            break;
        case 0:
            UICharDraw(&UI_State_sta[0], "ss0", UI_Graph_Change, 8, UI_Color_Orange, 0, 2, 5, 840, "pump:");
            UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_Change, 8, UI_Color_Orange, 0, 2, 5, 840, "close");
            UICircleDraw(&UI_Energy[1], "sd6", UI_Graph_Change, 7, UI_Color_Yellow, 0, 50, 850, 25); // 满的（填满）参数是25，50，850，25
            break;
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[0]);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[1]);
        _Interactive_data->Referee_Interactive_Flag.pump_flag = 0;
    }
    // chassis
    if (_Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        switch (_Interactive_data->chassis_mode)
        {
        case CHASSIS_ZERO_FORCE:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Orange, 15, 2, 150, 750, "zeroforce");
            break;
        case CHASSIS_NORMAL:
            UILineDraw(&UI_body_line[0], "sb0", UI_Graph_Change, 7, UI_Color_Yellow, 3, body_line_start_location[0], body_line_start_location[1], body_line_end_location[0], body_line_end_location[1]);
            UILineDraw(&UI_body_line[1], "sb1", UI_Graph_Change, 7, UI_Color_Yellow, 3, body_line_start_location[4], body_line_start_location[5], body_line_end_location[4], body_line_end_location[5]);
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 750, "normal   ");
            // 此处注意字数对齐问题，字数相同才能覆盖掉
            break;
        case CHASSIS_MINING:
            UILineDraw(&UI_body_line[0], "sb0", UI_Graph_Change, 7, UI_Color_Yellow, 3, body_line_start_location[2], body_line_start_location[3], body_line_end_location[2], body_line_end_location[3]);
            UILineDraw(&UI_body_line[1], "sb1", UI_Graph_Change, 7, UI_Color_Yellow, 3, body_line_start_location[6], body_line_start_location[7], body_line_end_location[6], body_line_end_location[7]);
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 750, "mining   ");
            break;
        case CHASSIS_NO_MOVE:
            UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 750, "no_move  ");
            break;
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_body_line[0], UI_body_line[1]);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 0;
    }
    // upper
    if (_Interactive_data->Referee_Interactive_Flag.upper_flag == 1)
    {
        switch (_Interactive_data->upper_mode)
        {
        case UPPER_ZERO_FORCE:
            // 清除取矿线框
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 0, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "zeroforce       ");
            break;
        case UPPER_NO_MOVE:
            // 清除取矿线框
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 0, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "upper_no_move   ");
            break;
        case UPPER_CALI:
            // 清除取矿线框
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 0, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "cali            ");
            break;
        case UPPER_SINGLE_MOTOR:
            // 清除取矿线框
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 0, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "single          ");
            break;
        case UPPER_SLIVER_MINING:
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 3, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "sliver          ");
            break;
        case UPPER_THREE_SLIVER_MINING:
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 3, get_line_start_location[2], get_line_start_location[3], get_line_end_location[2], get_line_end_location[3]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "three_sliver    ");
            break;
        case UPPER_GET_THREE_SLIVER_MINING:
            // 清除取矿线框
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 0, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "get_three_sliver");
            break;
        case UPPER_GLOD_MINING:
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 3, get_line_start_location[4], get_line_start_location[5], get_line_end_location[4], get_line_end_location[5]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "glod            ");
            break;
        case UPPER_GROUND_MINING:
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 3, get_line_start_location[6], get_line_start_location[7], get_line_end_location[6], get_line_end_location[7]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "ground          ");
            break;
        case UPPER_EXCHANGE:
            // 清除取矿线框
            UIRectangleDraw(&UI_get_line[0], "sg0", UI_Graph_Change, 7, UI_Color_Yellow, 0, get_line_start_location[0], get_line_start_location[1], get_line_end_location[0], get_line_end_location[1]);
            UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Orange, 15, word_width, 150, 700, "exchange        ");
            break;
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_get_line[0]);
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
        _Interactive_data->Referee_Interactive_Flag.upper_flag = 0;
    }
    // gimbal
    if (_Interactive_data->Referee_Interactive_Flag.gimbal_flag == 1)
    {
        switch (_Interactive_data->gimbal_mode)
        {
        case GIMBAL_FREE_MODE:
        {
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Yellow, 15, word_width, 150, 650, "free             ");
            break;
        }
        case GIMBAL_NORMAL:
        {
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Yellow, 15, word_width, 150, 650, "normal           ");
            break;
        }
        case GIMBAL_MINING:
        {
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Yellow, 15, word_width, 150, 650, "mining           ");
            break;
        }
        case Steering_gear_door_open:
        {
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Yellow, 15, word_width, 150, 650, "door_open        ");
            break;
        }
        case Steering_gear_door_close:
        {
            UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Yellow, 15, word_width, 150, 650, "door_close       ");
            break;
        }
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 0;
    }

    // //显示机械臂位姿部分
    // if (_Interactive_data->chassis_mode == CHASSIS_MINING)
    // {
    //     /* code */
    // }
    // else
    // {
    //     /* code */
    // }

    // // shoot
    // if (_Interactive_data->Referee_Interactive_Flag.shoot_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 650, _Interactive_data->shoot_mode == SHOOT_ON ? "on " : "off");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    //     _Interactive_data->Referee_Interactive_Flag.shoot_flag = 0;
    // }
    // // friction
    // if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[3], "sd3", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 600, _Interactive_data->friction_mode == FRICTION_ON ? "on " : "off");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[3]);
    //     _Interactive_data->Referee_Interactive_Flag.friction_flag = 0;
    // }
    // // lid
    // if (_Interactive_data->Referee_Interactive_Flag.lid_flag == 1)
    // {
    //     UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_Change, 8, UI_Color_Pink, 15, 2, 270, 550, _Interactive_data->lid_mode == LID_OPEN ? "open " : "close");
    //     UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);
    //     _Interactive_data->Referee_Interactive_Flag.lid_flag = 0;
    // }
    // // power
    // if (_Interactive_data->Referee_Interactive_Flag.Power_flag == 1)
    // {
    //     UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_Change, 8, UI_Color_Green, 18, 2, 2, 750, 230, _Interactive_data->Chassis_Power_Data.chassis_power_mx * 1000);
    //     UILineDraw(&UI_Energy[2], "sd6", UI_Graph_Change, 8, UI_Color_Pink, 30, 720, 160, (uint32_t)750 + _Interactive_data->Chassis_Power_Data.chassis_power_mx * 30, 160);
    //     UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);
    //     _Interactive_data->Referee_Interactive_Flag.Power_flag = 0;
    // }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }

    if (_Interactive_data->upper_mode != _Interactive_data->upper_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.upper_flag = 1;
        _Interactive_data->upper_last_mode = _Interactive_data->upper_mode;
    }

    if (_Interactive_data->gimbal_mode != _Interactive_data->gimbal_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.gimbal_flag = 1;
        _Interactive_data->gimbal_last_mode = _Interactive_data->gimbal_mode;
    }

    if (_Interactive_data->pump_mode != _Interactive_data->pump_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.pump_flag = 1;
        _Interactive_data->pump_last_mode = _Interactive_data->pump_mode;
    }
}
