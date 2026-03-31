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

// 3. 全局变量结构体：封装所有车控相关变量，实现解耦管理
typedef struct {
    // ---------------- PID 参数 ----------------
    float Kp;         // 比例系数：控制偏差响应灵敏度，越大纠偏越快但易超调
    float Ki;         // 积分系数：消除稳态静差，越大静差消除越快但易振荡
    float Kd;         // 微分系数：抑制超调、阻尼振荡，越大超调抑制越强但易放大噪声
    float integral_sep_thresh; // 积分分离阈值：偏差大于此值时停止积分累加，避免积分饱和
    float diff_filter_alpha;   // 微分滤波系数（0-1）：值越小滤波越强，微分项越平滑但响应越慢

    // ---------------- PID 计算变量 ----------------
    float bias;         // 当前横向偏差：由传感器读取，正值左偏、负值右偏
    float bias_last;    // 上一次采样的横向偏差：用于计算微分项
    float smooth_bias;  // 平滑后的横向偏差：经一阶低通滤波，减少传感器噪声
    float working_bias; // 最终用于PID的偏差：丢线时用原始bias，否则用smooth_bias
    float I;            // 积分项：偏差历史累加和，用于消除稳态静差
    float diff_raw;     // 原始微分项：e(k)-e(k-1)，未经过滤波
    float diff_filtered;// 滤波后的微分项：经一阶低通滤波，抑制噪声
    float left_v;       // 左轮速度：基础速度±PID差速，范围0-199
    float right_v;      // 右轮速度：基础速度∓PID差速，范围0-199
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

// // 前置声明硬件相关函数
// void go_forward_a(int speed);
// void go_forward_b(int speed);
// void go_backward_a(int speed);
// void go_backward_b(int speed);

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
    ctx->smooth_bias = ctx->smooth_bias * 0.7f + ctx->bias * (1.0f - 0.7f);
}

// /**
//  * @brief 微分项一阶低通滤波
//  * @param ctx: CarContext指针，访问diff_filter_alpha、diff_raw和diff_filtered
//  * @note 滤波公式：diff_filtered = alpha*旧值 + (1-alpha)*新值，抑制微分项噪声
//  */
static void filter_differential(CarContext *ctx) {
    ctx->diff_filtered = ctx->diff_filtered * ctx->diff_filter_alpha + 
                         ctx->diff_raw * (1.0f - ctx->diff_filter_alpha);
}

// ==========================================
// 五、具体的状态逻辑实现 (拆分后的函数)
// ==========================================

// ctx->bias == g_car.bias为原始偏差值
// /**
//  * @brief 直线跟踪状态：使用位置式PID进行横向纠偏
//  * @param ctx: CarContext指针，访问和修改所有车控变量
//  */
void Action_Straight(CarContext *ctx) {
    // 转弯冷却计时：冷却期内递减，禁止触发新转弯
    if(ctx->turn_cooldown > 0) ctx->turn_cooldown--;

    // 传感器平滑处理与偏差选择
    if(ctx->line_lost == 0) { // 未丢线时，对偏差做滤波
        smooth_sensor_bias(ctx);
    }

    // 选择工作偏差：丢线用原始bias，否则用平滑后的bias
    ctx->working_bias = ctx->line_lost ? ctx->bias : ctx->smooth_bias;

    // 死区控制：小偏差（≤4.0）时不纠偏（0->3.0->5.1)
    float deadzone = 4.0f;
    if(abs_float(ctx->smooth_bias) <= deadzone && ctx->line_lost == 0) {
        ctx->working_bias = 0;
    }

    // 位置式PID计算
    // 比例项，直接使用当前工作偏差
    float P = ctx->working_bias;

    // 积分项（带积分分离思想），当偏差大于死区时，累加积分，小于死区时缓慢衰减，避免小偏差时因积分导致蛇形
    if(abs_float(ctx->working_bias) > deadzone) {
        // 只有偏差在积分分离阈值内时才累加积分，超出阈值则不再累加
        if(abs_float(ctx->working_bias) < ctx->integral_sep_thresh) {
             ctx->I += ctx->working_bias;
        }
    } else {
        ctx->I *= 0.95f; // 小偏差时缓慢衰减积分
    }

    // 积分限幅
    if(ctx->I > 18) ctx->I = 18;
    if(ctx->I < -18) ctx->I = -18;

    // 微分项 (带滤波，因为原始偏差值是离散数值，会产生跳变，用平滑后的差值系统更加稳定）
    ctx->diff_raw = ctx->working_bias - ctx->bias_last;
    filter_differential(ctx);
    float D = ctx->diff_filtered;

    // PID 输出
    float PID_value = ctx->Kp * P + ctx->Ki * ctx->I + ctx->Kd * D;
    ctx->bias_last = ctx->working_bias;

    // 输出限幅
    float max_output = ctx->line_lost ? 25 : 15;
    if(PID_value > max_output) PID_value = max_output;
    if(PID_value < -max_output) PID_value = -max_output;

    // 电机差速控制
    ctx->left_v = ctx->v_base + PID_value;
    ctx->right_v = ctx->v_base - PID_value;

    // 电机速度限幅（0-99，对应PWM占空比范围）
    if(ctx->left_v > 99) ctx->left_v = 99;
    if(ctx->left_v < 0) ctx->left_v = 0;
    if(ctx->right_v > 99) ctx->right_v = 99;
    if(ctx->right_v < 0) ctx->right_v = 0;

    go_forward_a((int)ctx->left_v);
    go_forward_b((int)ctx->right_v);
}

// --- 状态 2: 左转 - 阶段 A: 刹车 ---
// /**
//  * @brief 左转弯阶段1：进入转弯前刹车减速
//  * @param ctx: CarContext指针，访问和修改转弯状态
//  */
void Action_TurnLeft_Brake(CarContext *ctx) {
    ctx->turn_counter++; // 本阶段计时
    
    // 执行刹车：左右轮同时倒车，快速减速
    go_backward_a((int)ctx->v_base);
    go_backward_b((int)ctx->v_base);

    // 状态切换：刹车15个周期后，进入原地旋转阶段
    if(ctx->turn_counter >= 15) {
        ctx->turn_phase = PHASE_SPINNING;// 旋转
        ctx->turn_counter = 0; // 重置计数器，用于下一阶段
    }
}

// --- 状态 2: 左转 - 阶段 B: 旋转 ---
// /**
//  * @brief 左转弯阶段2：原地旋转，智能检测退出
//  * @param ctx: CarContext指针，访问和修改旋转状态
//  */
void Action_TurnLeft_Spin(CarContext *ctx) {
    ctx->turn_counter++; // 本阶段计时

    // 执行原地旋转：左轮前进37，右轮停止，实现左转
    go_forward_a(37); 
    go_forward_b(0);  

    // 智能退出检测：旋转200个周期后，开始检测M0传感器
    if(ctx->turn_counter > 200) {
        if(M0 == 0) { // M0检测到黑线（回到中线）
            ctx->m0_detect_count++; // 连续检测计数+1
            if(ctx->m0_detect_count >= 5) { // 连续检测5次，一定转弯完成，车辆姿态回正
                // 立即切换到停稳阶段
                ctx->turn_phase = PHASE_EXIT_BRAKE;// 停稳
                ctx->turn_counter = 0;
                return;
            }
        } else {
            ctx->m0_detect_count = 0; // 未检测到，重置连续计数
        }
    }

    // 超时退出：作为智能退出的备用机制，防止无限旋转
    if(ctx->turn_counter >= ctx->time_compensation) {
        ctx->turn_phase = PHASE_EXIT_BRAKE;
        ctx->turn_counter = 0;
    }
}

// /**
//  * @brief 左转弯阶段3：转弯后停稳，复位到直线模式
//  * @param ctx: CarContext指针，复位所有状态和变量
//  */
// --- 状态 2: 左转 - 阶段 C: 停稳 ---
void Action_TurnLeft_Stop(CarContext *ctx) {
    ctx->turn_counter++;

    // 执行停止刹车
    go_backward_a(120);

    if(ctx->turn_counter >= 15) {
        // 复位到直线模式，参数重置
        ctx->main_state = STATE_STRAIGHT;
        ctx->turn_phase = PHASE_ENTER_BRAKE;// 刹车
        ctx->turn_counter = 0;
        ctx->turn_cooldown = 500; // 转弯冷却，防止转弯误触发
        ctx->I = 0;
        ctx->smooth_bias = 0;
        ctx->diff_filtered = 0;
        ctx->bias = 0;
        ctx->bias_last = 0;
        ctx->m0_detect_count = 0;
    }
}

// ==========================================
// 六、映射表与调度器
// ==========================================

// // 5. 映射表结构体：将「主状态+子状态」绑定到对应的执行函数
// typedef struct {
//     MainState   state;  // 主状态：匹配当前运行的核心模式
//     TurnPhase   phase;  // 子状态：匹配转弯的子阶段（直线模式忽略）
//     StateAction action; // 动作函数指针：对应状态下的具体执行逻辑
// } FSM_Map;
// 映射表：把状态和函数绑定
const FSM_Map Car_FSM_Table[] = {
    {STATE_STRAIGHT,   0,                     Action_Straight},
    {STATE_TURN_LEFT,  PHASE_ENTER_BRAKE,    Action_TurnLeft_Brake},
    {STATE_TURN_LEFT,  PHASE_SPINNING,       Action_TurnLeft_Spin},
    {STATE_TURN_LEFT,  PHASE_EXIT_BRAKE,     Action_TurnLeft_Stop},
};

// /**
//  * @brief 状态机调度器：遍历映射表，匹配当前状态并执行对应动作
//  * @note 主循环中在sensor_read()之后调用
//  */
// 扩展性：比如增加右转任务，只需要在映射表中增加右转的相关状态并定义右转直行函数，调度器完全不需要修改，避免了if-else多层耦合可能出现的逻辑错误
void sensor_pid() {
    int size = sizeof(Car_FSM_Table) / sizeof(Car_FSM_Table[0]);

    for(int i = 0; i < size; i++) {
        // 1. 主状态必须匹配
        if(Car_FSM_Table[i].state == g_car.main_state) {
            
            // 2. 子状态匹配 (直线忽略子状态)
            if( (Car_FSM_Table[i].state == STATE_STRAIGHT) || 
                (Car_FSM_Table[i].phase == g_car.turn_phase) ) {
                
                // 3. 执行动作
                Car_FSM_Table[i].action(&g_car);
                break;
            }
        }
    }
}

// ==========================================
// 七、传感器读取与状态切换逻辑
// ==========================================
// /**
//  * @brief 传感器读取与状态切换：读取灰度传感器、计算偏差、检测转弯、处理丢线
//  * @note 主循环中在sensor_pid()之前调用
//  */
void sensor_read() {
    // 1. 转弯检测 (只有在直线且冷却结束时才检测)
    if(L2 == 0 && g_car.main_state == STATE_STRAIGHT && g_car.turn_cooldown == 0) {
        // 记录数据
        g_car.initial_bias_before_turn = g_car.working_bias; // 留存转弯状态前的工作偏差值
        g_car.time_compensation = 1000; // 最大转弯时间
        
        // 切换状态
        g_car.main_state = STATE_TURN_LEFT;
        g_car.turn_phase = PHASE_ENTER_BRAKE;
        g_car.turn_counter = 0;
        g_car.m0_detect_count = 0;
        g_car.I = 0;// 清除历史积分
        g_car.line_lost = 0;
        return;
    }

    // 2. 直线传感器读取逻辑
    unsigned char line_state = (L1 << 2) | (M0 << 1) | R1;
    unsigned char all_sensors = (L2 << 4) | (L1 << 3) | (M0 << 2) | (R1 << 1) | R2;

    // 丢线检测
    if(all_sensors == 0) {
        g_car.line_lost = 1;
        if(g_car.last_valid_bias > 0) g_car.bias = 50;
        else if(g_car.last_valid_bias < 0) g_car.bias = -50;
        return;
    } else {
        if(g_car.line_lost == 1) {
            g_car.line_lost = 0;
            g_car.I = 0;
            g_car.diff_filtered = 0;
        }
    }

    // 正常线处理
    switch(line_state) {
        case 0b010: g_car.bias = 0; g_car.last_valid_bias = 0; break;
        case 0b110: g_car.bias = -10; g_car.last_valid_bias = -10; break;
        case 0b011: g_car.bias = 10; g_car.last_valid_bias = 10; break;
        case 0b100: g_car.bias = -30; g_car.last_valid_bias = -30; break;
        case 0b001: g_car.bias = 30; g_car.last_valid_bias = 30; break;
        default: break; // 保持上一次
    }
}

// ==========================================
// 八、初始化
// ==========================================
void car_init() {
    // ---------------- 初始化PID参数 ----------------
    g_car.Kp = 2.5f;
    g_car.Ki = 0.08f;
    g_car.Kd = 0.4f;
    g_car.integral_sep_thresh = 15.0f;
    g_car.diff_filter_alpha = 0.7f;
    g_car.v_base = 41;
    
    // ---------------- 初始化PID计算变量 ----------------
    g_car.bias = 0.0f;
    g_car.bias_last = 0.0f;
    g_car.smooth_bias = 0.0f;  // 平滑偏差初始化为0
    g_car.working_bias = 0.0f; // 工作偏差初始化为0
    g_car.I = 0.0f;
    g_car.diff_raw = 0.0f;     // 原始微分项初始化为0
    g_car.diff_filtered = 0.0f;// 滤波后微分项初始化为0
    g_car.left_v = 0.0f;        // 左轮速度初始化为0
    g_car.right_v = 0.0f;       // 右轮速度初始化为0
    
    // ---------------- 初始化传感器与逻辑 ----------------
    g_car.last_valid_bias = 0;
    g_car.line_lost = 0;        // 丢线标志初始化为0（未丢线）
    
    // ---------------- 初始化状态机控制 ----------------
    g_car.main_state = STATE_STRAIGHT;
    g_car.turn_phase = PHASE_ENTER_BRAKE;
    g_car.turn_counter = 0;      // 转弯计数器初始化为0
    g_car.turn_cooldown = 0;
    g_car.m0_detect_count = 0;
    
    // ---------------- 初始化转弯参数 ----------------
    g_car.time_compensation = 0;     // 转弯超时时间初始化为0
    g_car.initial_bias_before_turn = 0.0f;// 转弯前初始偏差初始化为0
}

//想在函数内部修改一个外部变量，可以在函数输入接口传这个变量的地址，如：
// void change_value(int *p) {
//     *p = 100; // 通过地址修改外部变量
// }

// int main() {
//     int a = 10;
//     change_value(&a); // 传的是 a 的地址
//     return 0;
// }
//同理，想要在函数内修改全局变量，也可以传入全局变量结构体的首地址，即&g_car
//而通用函数指针类型定义typedef void (*StateAction)(CarContext *ctx);
//这其中的*ctx就是CarContext类的指针需要传入CarContext类变量的地址
//全局变量可以在函数内部直接修改，但用传结构体指针的方式利于解藕和扩展