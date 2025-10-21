#include "FOC.h"

// ============================================================================
// 函数：setMotorTorque
// 功能：力矩控制函数（电流环控制）
// 参数：Target - 目标电流值（力矩指令）
// 说明：这是最内层的电流环控制，直接控制电机的输出力矩
// ============================================================================
void setMotorTorque(float Target) {
    // 计算电流环PID输出：目标电流 - 实际测量电流
    float current_error = Target - getMotorCurrent();
    float pid_output = current_loop_M0(current_error);
    
    // 使用PID输出和电角度设置电机力矩
    setTorque(pid_output, electricalAngle());
}

// ============================================================================
// 函数：setMotorVelocityWithAngle
// 功能：位置-速度-电流三环控制（外环到内环的级联控制）
// 参数：Target - 目标位置（弧度）
// 说明：实现完整的三环控制策略：
//       位置环（外环）→ 速度环（中环）→ 电流环（内环）
// ============================================================================
void setMotorVelocityWithAngle(float Target) {
    // 1. 位置环控制：计算位置误差并转换为角度PID
    //    将弧度转换为角度（180/PI），计算位置误差
    float position_error = (Target - getMotorAngle()) * 180 / PI;
    float angle_pid_output = calculateAnglePID(position_error);
    
    // 2. 速度环控制：将位置环输出作为速度环的参考值
    //    速度环输入 = 位置环输出 - 实际速度
    float velocity_error = angle_pid_output - getMotorVelocity();
    float iq_ref = calculateVelocityPID(velocity_error);
    
    // 3. 电流限幅：限制q轴电流参考值在安全范围内
    iq_ref = _constrain(iq_ref, -I_MAX_CMD, I_MAX_CMD);
    
    // 4. 调用力矩控制函数（电流环）
    setMotorTorque(iq_ref);
}

// ============================================================================
// 函数：runFOC
// 功能：FOC主控制循环
// 说明：每个控制周期需要执行的核心任务
//       1. 更新传感器数据（角度）
//       2. 更新电流传感器数据
// ============================================================================
void runFOC() {
    // 更新磁编码器角度数据
    S0.Sensor_update();
    
    // 更新三相电流测量值
    CS_M0.getPhaseCurrents();
}

// ============================================================================
// 函数：readSerialCommand
// 功能：串口通信命令处理
// 返回值：接收到的完整命令字符串
// 说明：处理来自串口的控制命令，支持多字符命令的接收和解析
// ============================================================================
String readSerialCommand() {
    static String received_chars;  // 静态变量，保存未完成的命令字符
    String command = "";           // 完整的命令字符串

    // 循环读取所有可用的串口数据
    while (Serial.available()) {
        char inChar = (char)Serial.read();  // 读取一个字符
        received_chars += inChar;           // 添加到接收缓冲区

        // 检测到换行符表示命令结束
        if (inChar == '\n') {
            command = received_chars;  // 获取完整命令
            
            // 查找换行符位置（命令结束标志）
            int commaPosition = command.indexOf('\n');
            if (commaPosition != -1) {
                // 提取命令数值并转换为浮点数
                String target_str = command.substring(0, commaPosition);
                motor_target = target_str.toDouble();
                
                // 回显接收到的目标值（用于调试）
                Serial.println(motor_target);
            }
            
            // 清空接收缓冲区，准备接收下一条命令
            received_chars = "";
        }
    }
    return command;  // 返回处理后的命令
}

// ============================================================================
// 函数：getSerialMotorTarget
// 功能：BLE蓝牙目标值处理
// 返回值：处理后的电机目标位置（弧度）
// 说明：处理来自蓝牙的电机控制命令，支持角度到弧度的转换和去重处理
// ============================================================================
float getSerialMotorTarget() {
    static float last_target = 0.0f;  // 保存上一次的目标值，用于去重
    
    // 检查是否有新的BLE命令到达
    if (new_command) {
        new_command = false;  // 重置命令标志
        
        // BLE传输的是输出角度（度），需要转换为电机轴角度
        float out_deg = ble_motor_target;
        
        // 角度转换：输出角度 → 电机机械角度（弧度）
        // 考虑减速比和角度单位转换
        float motor_rad = out_deg * GEAR_RATIO * (PI / 180.0f);

        // 去重处理：只有当目标值变化超过阈值时才更新
        if (fabs(motor_rad - last_target) > 0.0001f) {
            last_target = motor_rad;     // 更新上一次目标值
            motor_target = motor_rad;    // 设置新的电机目标
            
            // 调试信息：显示转换过程和参数
            Serial.printf("[DEBUG] BLE输出角度 %.2f° -> 电机目标 %.4f rad (ratio=%g)\n",
                          out_deg, motor_rad, GEAR_RATIO);
        } else {
            // 目标值未变化时的调试信息
            Serial.printf("[DEBUG] BLE目标未改变: 输出角度 %.2f°\n", out_deg);
        }
    }
    
    // 返回当前电机目标位置
    return motor_target;
}