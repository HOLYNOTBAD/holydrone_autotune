# 人工候选试验事件：记录本机发布时间与 FRD 激励语义，不代表飞控接收时刻。
# ROS 事件发布时刻；离线通过 TIMESYNC 映射到 PX4 时间。
time timestamp
# 本机单调时钟秒数；不同进程/会话之间不直接比较。
float64 steady_s
# 配置中的试验标识。
string experiment_id
# 当前候选 ID；选定前允许为空。
string candidate_id
# 事件类型，见 brct_autotune_fsm.cpp 的 event 调用。
string event_type
# 配置采用的 FRD 轴：roll、pitch 或 yaw。
string axis
# 当前 low/high 幅值；deg/s；未开始时为 0。
float64 amplitude_deg_s
# 请求参数的单行 YAML 映射字符串。
string requested_parameters
# 实际读回参数的单行 YAML 映射；未确认时可能为空。
string readback_parameters
# 本会话试验编号；0 表示尚未开始试验。
uint32 trial_id
# 从 0 开始的重复轮次；-1 表示尚未进入。
int32 repetition
# 阶段索引：初始回位 0，脉冲 1/3/5/7，脉冲后回位 2/4/6/8；-1 表示尚未进入。
int32 phase
# 补充信息或失败原因；允许为空。
string detail
