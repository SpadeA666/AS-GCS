# 2D 打点的一个点。kind 决定它去哪条通路。
Header header

uint8 KIND_PLANNER=0     # 交给规划器（EGO/SUPER）
uint8 KIND_PX4=1         # 直接走 PX4 local 位置控制

uint8 kind
geometry_msgs/Point position
float32 yaw
float32 tol
uint8 HEIGHT_HOLD=0      # 保持当前高度
uint8 HEIGHT_ABS=1       # 使用 position.z
uint8 height_mode
