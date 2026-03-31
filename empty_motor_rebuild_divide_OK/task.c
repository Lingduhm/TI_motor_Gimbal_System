#include "task.h"

// 任务状态枚举
typedef enum {
    TASK_IDLE = 0,          // 空闲状态
    TASK_RUNNING,           // 执行任务状态
    TASK_COMPLETED          // 任务完成状态
} TaskState_t;

// 任务类型枚举
typedef enum {
    TASK_NONE = 0,          // 无任务
    TASK_ONE_CIRCLE,        // 一圈任务
    TASK_TWO_CIRCLES,       // 两圈任务
    TASK_THREE_CIRCLES,     // 三圈任务
    TASK_FOUR_CIRCLES,      // 四圈任务
    TASK_FIVE_CIRCLES       // 五圈任务
} TaskType_t;

// 任务控制结构体
typedef struct {
    TaskState_t state;          // 当前任务状态
    TaskType_t task_type;       // 任务类型
    unsigned int target_turns;  // 目标转弯次数
    unsigned int current_turns; // 当前转弯次数
    unsigned int task_timer;    // 任务定时器
    unsigned char last_key;     // 上次按键状态
} TaskManager_t;

// 全局任务管理器实例
TaskManager_t g_task_manager;

// /**
//  * @brief 初始化任务管理器
//  */
void TaskManager_Init(void)
{
    g_task_manager.state = TASK_IDLE;
    g_task_manager.task_type = TASK_NONE;
    g_task_manager.target_turns = 0;
    g_task_manager.current_turns = 0;
    g_task_manager.task_timer = 0;
}

// /**
//  * @brief 启动任务
//  */
void TaskManager_StartTask(TaskType_t task_type)
{
    g_task_manager.state = TASK_RUNNING;
    g_task_manager.task_type = task_type;
    g_task_manager.current_turns = 0;
    g_task_manager.task_timer = 0;
    
    // 设置目标转弯次数 (一圈 = 4次转弯)
    switch (task_type) {
        case TASK_ONE_CIRCLE:
            g_task_manager.target_turns = 5;
            break;
        case TASK_TWO_CIRCLES:
            g_task_manager.target_turns = 9;
            break;
        case TASK_THREE_CIRCLES:
            g_task_manager.target_turns = 13;
            break;
        case TASK_FOUR_CIRCLES:
            g_task_manager.target_turns = 17;
            break;
        case TASK_FIVE_CIRCLES:
            g_task_manager.target_turns = 21;
            break;
        default:
            g_task_manager.target_turns = 0;
            break;
    }
}

// /**
//  * @brief 处理按键输入
//  */
void TaskManager_HandleKey(unsigned char key)
{
    // 只有在空闲状态才响应按键
    if (g_task_manager.state != TASK_IDLE) {
        return;
    }
    
    TaskType_t task_type = TASK_NONE;
    
    switch (key) {
        case 1:
            task_type = TASK_ONE_CIRCLE;
            break;
        case 2:
            task_type = TASK_TWO_CIRCLES;
            break;
        case 3:
            task_type = TASK_THREE_CIRCLES;
            break;
        case 4:
            task_type = TASK_FOUR_CIRCLES;
            break;
        case 5:
            task_type = TASK_FIVE_CIRCLES;
            break;
        default:
            return;
    }
    
    TaskManager_StartTask(task_type);
}

// 主循环专用的按键去抖处理（替代中断内的去抖）
void TaskManager_KeyDebounce(void)
{
    static uint8_t last_key = 0; // 静态局部变量
    static uint8_t debounce_cnt = 0; // 静态局部变量
    
    // 去抖逻辑：连续n次采样相同按键值，才判定为有效
    if (g_raw_key == last_key) {
        debounce_cnt++;
        if (debounce_cnt >= 2) {  // 去抖阈值n = 2（可调整）
            if (g_raw_key != 0 && g_task_manager.state == TASK_IDLE) {
                TaskManager_HandleKey(g_raw_key);  // 处理有效按键
            }
            debounce_cnt = 0;
        }   
    } else {
        last_key = g_raw_key; // 更新历史按键
        debounce_cnt = 0;
    }
}

// /**
//  * @brief 处理运行状态
//  */
void TaskManager_ProcessRunning(void)
{
    // 执行传感器PID控制
    sensor_read();
    sensor_pid();
    
    // 转弯计数（需保护全局变量，避免中断干扰）
    __disable_irq();  // 原子操作：关闭中断保护
    if (turn_completed_flag == 1) {
        turn_completed_flag = 0;
        g_task_manager.current_turns++;
        if (g_task_manager.current_turns >= g_task_manager.target_turns) {
            g_task_manager.state = TASK_COMPLETED;
        }
    }
    __enable_irq();   // 恢复中断
}

// /**
//  * @brief 显示任务状态
//  */
void TaskManager_DisplayStatus(void)
{
    static unsigned int blink_counter = 0;
    
    // 闪烁效果 - 使用LED指示任务完成状态
    blink_counter++;
    if (blink_counter > 50) { // 每50个单位时间翻转电平
        blink_counter = 0;
        // LED闪烁指示任务完成
        DL_GPIO_togglePins(LED1_PORT, LED1_PIN_22_PIN);
    }
}

// /**
//  * @brief 重置任务管理器到空闲状态
//  */
void TaskManager_Reset(void)
{
    g_task_manager.state = TASK_IDLE;
    g_task_manager.task_type = TASK_NONE;
    g_task_manager.target_turns = 0;
    g_task_manager.current_turns = 0;
    g_task_manager.task_timer = 0;
    
    // 确保电机停止
    go_stop();

    // 关闭LED
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_22_PIN);
}

// /**
//  * @brief 任务管理器主更新函数 - 在定时器中断中调用
//  */
void TaskManager_Update(void)
{
    // 按键去抖
    if (g_flag_need_update) {// 5ms一次，控制步长，防止高速循环
        TaskManager_KeyDebounce();

        // 状态机处理
        switch (g_task_manager.state) {
            case TASK_IDLE:
                // 空闲状态，等待按键输入
                // sensor_pid不执行，电机停止
                go_stop();
                break;
            
            case TASK_RUNNING:
                // 运行状态，执行线程跟随和转弯计数
                TaskManager_ProcessRunning();
                break;
            
            case TASK_COMPLETED:
                // 任务完成状态
                go_stop();
                TaskManager_DisplayStatus();
            
                // 自动重置到空闲状态倒计时
                g_task_manager.task_timer++;
                if (g_task_manager.task_timer > 300) { // 重置1.5s
                    TaskManager_Reset();
                }
                break;
        }

        g_flag_need_update = 0;  // 清除标记
    }
}