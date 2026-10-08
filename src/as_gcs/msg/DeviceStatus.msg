# 设备管理状态（latched 发布，前端一连上就能知道当前什么在跑）
Header header

string camera_mode       # off | mono | d435i | both
bool yolo_running
string yolo_launch       # 实际在跑的 launch 文件名，空串表示没跑
string[] running_devices # 当前活着的子进程标识
string[] messages        # 人可读的状态说明/错误
