#include "sensor.h"

// ==========================================
// 一、硬件相关宏定义 (保持原样)
// ==========================================
// 灰度传感器的GPIO设置为下拉输入（Pull-Down resistor）遇到黑线时，电平置高，宏定义变为0
#define L2  (DL_GPIO_readPins(L2_PORT, L2_PIN_27_PIN) == 0)
#define L1  (DL_GPIO_readPins(L1_PORT, L1_PIN_23_PIN) == 0)
#define M0  (DL_GPIO_readPins(M0_PORT, M0_PIN_12_PIN) == 0)
#define R1  (DL_GPIO_readPins(R1_PORT, R1_PIN_8_PIN) == 0)
#define R2  (DL_GPIO_readPins(R2_PORT, R2_PIN_6_PIN) == 0)

// static inline bool Sensor_L2_IsBlack(void) {
//     return (DL_GPIO_readPins(L2_PORT, L2_PIN_27_PIN) == 0);
// }用static inline来替换#define，有类型检查，会更加安全，但作用域都一样，在本.c文件中可用，外部.c无法访问

// ==========================================
// 二、状态机核心定义 (重构重点)
// ==========================================

// 1. 主状态枚举
typedef enum {
    STATE_STRAIGHT,    // 0: 直线跟踪
    STATE_TURN_LEFT    // 1: 左转弯
} MainState;

// 2. 转弯子状态枚举
typedef enum {
    PHASE_ENTER_BRAKE, // 0: 进入转弯前刹车
    PHASE_SPINNING,    // 1: 原地旋转
    PHASE_EXIT_BRAKE   // 2: 转弯后停稳复位
} TurnPhase;

// 全局变量
volatile unsigned char turn_completed_flag = 0;  // 转弯完成标志，用于任务管理器计数

// 3. 变量结构体：封装所有车控相关变量，实现解耦管理
typedef struct {
    // ---------------- PID 参数 ----------------
    float Kp;         // 比例系数：控制偏差响应灵敏度，越大纠偏越快但易超调
    float Ki;         // 积分系数：消除稳态静差，越大静差消除越快但易振荡
    float Kd;         // 微分系数：抑制超调、阻尼振荡，越大超调抑制越强但易放大噪声
    float integral_sep_thresh; // 积分分离阈值：偏差大于此值时关闭积分，避免超调
    float diff_filter_alpha;   // 微分滤波系数（0-1）：值越小滤波越强，微分项越平滑但响应越慢
    float delta_max;  // 单周期增量上限：限制单次最大转向幅度
    float delta_min;  // 单周期增量下限：限制单次最大反向转向幅度

    // ---------------- PID 计算变量（增量式专属） ----------------
    float bias;         // 当前横向偏差 e(k)：由传感器读取，正值左偏、负值右偏
    float err_last1;    // 上一次采样偏差 e(k-1)：用于增量式PID计算
    float err_last2;    // 上上次采样偏差 e(k-2)：【增量式新增】用于微分项计算
    float smooth_bias;  // 平滑后的横向偏差：经一阶低通滤波，减少传感器噪声
    float working_bias; // 最终用于PID的偏差：丢线时用原始bias，否则用smooth_bias
    float delta_out;    // 单周期输出增量 Δu(k)
    float pid_output;   // 最终PID输出 u(k)：累加增量后的差速值
    float diff_raw;     // 原始微分项：未经过滤波
    float diff_filtered;// 滤波后的微分项：经一阶低通滤波，抑制噪声
    float left_v;       // 左轮速度：基础速度±PID输出，范围0-199
    float right_v;      // 右轮速度：基础速度∓PID输出，范围0-199
    float v_base;       // 直线行驶基础速度：PID差速叠加在此基础上

    // ---------------- 传感器与逻辑 ----------------
    int last_valid_bias;    // 最后一次有效横向偏差：丢线时用于判断纠偏方向
    unsigned char line_lost;// 丢线标志：1=所有传感器未检测到黑线，0=检测到黑线

    // ---------------- 状态机控制 ----------------
    MainState  main_state;       // 当前主状态：直线跟踪/左转弯
    TurnPhase  turn_phase;       // 当前转弯子状态：刹车/旋转/停稳
    unsigned int turn_counter;    // 转弯阶段计时计数器：每个子状态内的周期计数
    unsigned int turn_cooldown;  // 转弯冷却计数器：冷却期内禁止触发新转弯
    unsigned int m0_detect_count;// M0传感器连续检测计数：用于智能退出转弯

    // ---------------- 转弯参数 ----------------
    unsigned int time_compensation;   // 转弯超时时间：智能退出的备用机制
    float initial_bias_before_turn;   // 转弯前初始偏差（未调用）
} CarContext;

// 4. 函数指针类型定义：统一状态动作函数的接口
typedef void (*StateAction)(CarContext *ctx); // 传入CarContext指针，修改车控状态

// 5. 映射表结构体：将「主状态+子状态」绑定到对应的执行函数
typedef struct {
    MainState   state;  // 主状态：匹配当前运行的核心模式
    TurnPhase   phase;  // 子状态：匹配转弯的子阶段（直线模式忽略）
    StateAction action; // 动作函数指针：对应状态下的具体执行逻辑
} FSM_Map;

// ==========================================
// 三、全局对象与前置声明
// ==========================================

// 实例化全局车控对象 (代替原来的几十个全局变量)
CarContext g_car;

// ==========================================
// 四、辅助工具函数 (仅在本文件中调用）
// ==========================================

// /**
//  * @brief 整数取绝对值
//  * @param x: 输入整数
//  * @return 输入值的绝对值
//  */
static int abs_int(int x) { return x < 0 ? -x : x; }

// /**
//  * @brief 浮点数取绝对值
//  * @param x: 输入浮点数
//  * @return 输入值的绝对值
//  */
static float abs_float(float x) { return x < 0.0f ? -x : x; }

// 实际上两个滤波函数的参数相同 都是alpha*旧值 + (1-alpha)*新值，alpha为0.7f
// /**
//  * @brief 传感器偏差一阶低通滤波
//  * @param ctx: CarContext指针，访问smooth_bias和bias
//  * @note 滤波公式：smooth_bias = 0.7*旧值 + 0.3*新值，减少传感器突变
//  */
static void smooth_sensor_bias(CarContext *ctx) {
    if(ctx == NULL) return;
    ctx->smooth_bias = ctx->smooth_bias * 0.7f + ctx->bias * (1.0f - 0.7f);
}

// /**
//  * @brief 微分项一阶低通滤波
//  * @param ctx: CarContext指针，访问diff_filter_alpha、diff_raw和diff_filtered
//  * @note 滤波公式：diff_filtered = alpha*旧值 + (1-alpha)*新值，抑制微分项噪声
//  */
static void filter_differential(CarContext *ctx) {
    if(ctx == NULL) return;
    ctx->diff_filtered = ctx->diff_filtered * ctx->diff_filter_alpha + 
                         ctx->diff_raw * (1.0f - ctx->diff_filter_alpha);
}

// ==========================================
// 五、具体的状态逻辑实现 (拆分后的函数)
// ==========================================

// /**
//  * @brief 直线跟踪状态：使用增量式PID进行横向纠偏
//  * @param ctx: CarContext指针，访问和修改所有车控变量
//  */
void Action_Straight(CarContext *ctx) {
    if(ctx == NULL) return;

    // 转弯冷却计时：冷却期内递减，禁止触发新转弯
    if(ctx->turn_cooldown > 0) ctx->turn_cooldown--;

    // 传感器平滑处理：未丢线时，对原始偏差做一阶低通滤波
    if(ctx->line_lost == 0) {
        smooth_sensor_bias(ctx);
    }

    // 选择最终工作偏差：丢线用原始bias强制纠偏，否则用平滑后的偏差
    ctx->working_bias = ctx->line_lost ? ctx->bias : ctx->smooth_bias;

    // 死区控制：小偏差（≤4.0）时不纠偏，避免频繁微调导致蛇形
    float deadzone = 4.0f;
    if(abs_float(ctx->smooth_bias) <= deadzone && ctx->line_lost == 0) {
        ctx->working_bias = 0.0f;
    }

    // ==========================================
    // 增量式PID核心计算
    // ==========================================
    float p_term = 0.0f, i_term = 0.0f, d_term = 0.0f;

    // 1. 比例项：ΔP = Kp*[e(k)-e(k-1)]
    p_term = ctx->Kp * (ctx->working_bias - ctx->err_last1);

    // 2. 积分项（带积分分离）：ΔI = Ki*e(k)，大偏差时关闭积分
    if(abs_float(ctx->working_bias) < ctx->integral_sep_thresh) {
        i_term = ctx->Ki * ctx->working_bias;
    } else {
        i_term = 0.0f; // 大偏差关闭积分，避免超调
    }

    // 3. 微分项（带一阶低通滤波）：ΔD = Kd*[e(k)-2e(k-1)+e(k-2)]
    ctx->diff_raw = ctx->working_bias - 2 * ctx->err_last1 + ctx->err_last2;
    filter_differential(ctx); // 微分滤波，抑制噪声
    d_term = ctx->Kd * ctx->diff_filtered;

    // 4. 计算本次输出增量 Δu(k)
    ctx->delta_out = p_term + i_term + d_term;

    // 增量限幅：限制单周期最大转向幅度，避免猛打方向
    if(ctx->delta_out > ctx->delta_max) ctx->delta_out = ctx->delta_max;
    else if(ctx->delta_out < ctx->delta_min) ctx->delta_out = ctx->delta_min;

    // 5. 增量累加得到最终PID输出 u(k) = u(k-1) + Δu(k)
    ctx->pid_output += ctx->delta_out;

    // 6. 最终输出总限幅：防止差速超出执行机构范围
    float max_output = ctx->line_lost ? 25 : 15;
    if(ctx->pid_output > max_output) ctx->pid_output = max_output;
    else if(ctx->pid_output < -max_output) ctx->pid_output = -max_output;

    // 7. 更新历史偏差：为下一次计算做准备
    ctx->err_last2 = ctx->err_last1;
    ctx->err_last1 = ctx->working_bias;

    // ==========================================
    // 电机差速控制（完全兼容原有逻辑）
    // ==========================================
    ctx->left_v = ctx->v_base + ctx->pid_output;
    ctx->right_v = ctx->v_base - ctx->pid_output;

    // 电机速度限幅（0-99，对应PWM占空比范围）
    if(ctx->left_v > 99) ctx->left_v = 99;
    if(ctx->left_v < 0) ctx->left_v = 0;
    if(ctx->right_v > 99) ctx->right_v = 99;
    if(ctx->right_v < 0) ctx->right_v = 0;

    // 驱动电机
    go_forward_a((int)ctx->left_v);
    go_forward_b((int)ctx->right_v);
}

// /**
//  * @brief 左转弯阶段1：进入转弯前刹车减速
//  * @param ctx: CarContext指针，访问和修改转弯状态
//  */
void Action_TurnLeft_Brake(CarContext *ctx) {
    if(ctx == NULL) return;

    ctx->turn_counter++;
    
    // 执行刹车：左右轮同时倒车，快速减速
    go_backward_a((int)ctx->v_base);
    go_backward_b((int)ctx->v_base);

    // 状态切换：刹车15个周期后，进入原地旋转阶段
    if(ctx->turn_counter >= 15) {
        ctx->turn_phase = PHASE_SPINNING;
        ctx->turn_counter = 0;
    }
}

// /**
//  * @brief 左转弯阶段2：原地旋转，智能检测退出
//  * @param ctx: CarContext指针，访问和修改旋转状态
//  */
void Action_TurnLeft_Spin(CarContext *ctx) {
    if(ctx == NULL) return;

    ctx->turn_counter++;

    // 执行原地旋转：左轮停止，右轮前进37，实现左转
    go_forward_a(0); 
    go_forward_b(37);  

    // 智能退出检测：旋转200个周期后，开始检测M0传感器
    if(ctx->turn_counter > 200) {
        if(M0 == 0) {
            ctx->m0_detect_count++;
            if(ctx->m0_detect_count >= 3) {
                ctx->turn_phase = PHASE_EXIT_BRAKE;
                ctx->turn_counter = 0;
                return;
            }
        } else {
            ctx->m0_detect_count = 0;
        }
    }

    // 超时退出：作为智能退出的备用机制
    if(ctx->turn_counter >= ctx->time_compensation) {
        ctx->turn_phase = PHASE_EXIT_BRAKE;
        ctx->turn_counter = 0;
    }
}

// /**
//  * @brief 左转弯阶段3：转弯后停稳，复位到直线模式
//  * @param ctx: CarContext指针，复位所有状态和变量
//  */
void Action_TurnLeft_Stop(CarContext *ctx) {
    if(ctx == NULL) return;
    
    ctx->turn_counter++;

    // 执行停止刹车
    go_backward_a(120);

    if(ctx->turn_counter >= 15) {
        // 转弯完成标志置为1
        turn_completed_flag = 1;  

        // 复位到直线模式，参数重置
        ctx->main_state = STATE_STRAIGHT;
        ctx->turn_phase = PHASE_ENTER_BRAKE;
        ctx->turn_counter = 0;
        ctx->turn_cooldown = 500;
        
        // 增量式PID重置
        ctx->pid_output = 0.0f;
        ctx->delta_out = 0.0f;
        ctx->err_last1 = 0.0f;
        ctx->err_last2 = 0.0f;
        
        // 原有变量重置
        ctx->last_valid_bias = 0;
        ctx->smooth_bias = 0.0f;
        ctx->diff_filtered = 0.0f;
        ctx->bias = 0.0f;
        ctx->working_bias = 0.0f;
        ctx->m0_detect_count = 0;
    }
}

// ==========================================
// 六、映射表与调度器
// ==========================================

const FSM_Map Car_FSM_Table[] = {
    {STATE_STRAIGHT,   0,                     Action_Straight},
    {STATE_TURN_LEFT,  PHASE_ENTER_BRAKE,    Action_TurnLeft_Brake},
    {STATE_TURN_LEFT,  PHASE_SPINNING,       Action_TurnLeft_Spin},
    {STATE_TURN_LEFT,  PHASE_EXIT_BRAKE,     Action_TurnLeft_Stop},
};

void sensor_pid() {
    int size = sizeof(Car_FSM_Table) / sizeof(Car_FSM_Table[0]);

    for(int i = 0; i < size; i++) {
        if(Car_FSM_Table[i].state == g_car.main_state) {
            if( (Car_FSM_Table[i].state == STATE_STRAIGHT) || 
                (Car_FSM_Table[i].phase == g_car.turn_phase) ) {
                Car_FSM_Table[i].action(&g_car);
                break;
            }
        }
    }
}

// ==========================================
// 七、传感器读取与状态切换逻辑
// ==========================================

void sensor_read() {
    // 1. 转弯检测 (只有在直线且冷却结束时才检测)
    if(L2 == 0 && g_car.main_state == STATE_STRAIGHT && g_car.turn_cooldown == 0) {
        g_car.initial_bias_before_turn = g_car.working_bias;
        g_car.time_compensation = 1000;
        
        // 切换状态
        g_car.main_state = STATE_TURN_LEFT;
        g_car.turn_phase = PHASE_ENTER_BRAKE;
        g_car.turn_counter = 0;
        g_car.m0_detect_count = 0;
        // 【增量式PID专属重置】
        g_car.pid_output = 0.0f;
        g_car.delta_out = 0.0f;
        g_car.err_last1 = 0.0f;
        g_car.err_last2 = 0.0f;
        g_car.line_lost = 0;
        return;
    }

    // 2. 直线传感器读取逻辑
    unsigned char line_state = (L1 << 2) | (M0 << 1) | R1;
    unsigned char all_sensors = (L2 << 4) | (L1 << 3) | (M0 << 2) | (R1 << 1) | R2;

    // 丢线检测
    if(all_sensors == 0) {
        g_car.line_lost = 1;
        if(g_car.last_valid_bias >= 0) g_car.bias = 50;
        else if(g_car.last_valid_bias < 0) g_car.bias = -50;
        return;
    } else {
        if(g_car.line_lost == 1) {
            g_car.line_lost = 0;
            // 【增量式PID专属重置：丢线恢复后清零输出】
            g_car.pid_output = 0.0f;
            g_car.delta_out = 0.0f;
            g_car.err_last1 = 0.0f;
            g_car.err_last2 = 0.0f;
            g_car.diff_filtered = 0.0f;
        }
    }

    // 正常线处理
    switch(line_state) {
        case 0b010: g_car.bias = 0; g_car.last_valid_bias = 0; break;
        case 0b110: g_car.bias = -10; g_car.last_valid_bias = -10; break;
        case 0b011: g_car.bias = 10; g_car.last_valid_bias = 10; break;
        case 0b100: g_car.bias = -30; g_car.last_valid_bias = -30; break;
        case 0b001: g_car.bias = 30; g_car.last_valid_bias = 30; break;
        default: break;
    }
}

// ==========================================
// 八、初始化
// ==========================================
void car_init() {
    // ---------------- 初始化PID参数 ----------------
    g_car.Kp = 2.2f;
    g_car.Ki = 0.02f;
    g_car.Kd = 0.35f;
    g_car.integral_sep_thresh = 12.0f;
    g_car.diff_filter_alpha = 0.7f;
    g_car.v_base = 41;
    // 【增量式新增】增量限幅参数：单周期最大±3的差速变化
    g_car.delta_max = 3.0f;
    g_car.delta_min = -3.0f;
    
    // ---------------- 初始化PID计算变量（增量式） ----------------
    g_car.bias = 0.0f;
    g_car.err_last1 = 0.0f;
    g_car.err_last2 = 0.0f;
    g_car.smooth_bias = 0.0f;
    g_car.working_bias = 0.0f;
    g_car.delta_out = 0.0f;
    g_car.pid_output = 0.0f;
    g_car.diff_raw = 0.0f;
    g_car.diff_filtered = 0.0f;
    g_car.left_v = 0.0f;
    g_car.right_v = 0.0f;
    
    // ---------------- 初始化传感器与逻辑 ----------------
    g_car.last_valid_bias = 0;
    g_car.line_lost = 0;
    
    // ---------------- 初始化状态机控制 ----------------
    g_car.main_state = STATE_STRAIGHT;
    g_car.turn_phase = PHASE_ENTER_BRAKE;
    g_car.turn_counter = 0;
    g_car.turn_cooldown = 0;
    g_car.m0_detect_count = 0;
    
    // ---------------- 初始化转弯参数 ----------------
    g_car.time_compensation = 1000;
    g_car.initial_bias_before_turn = 0.0f;
}