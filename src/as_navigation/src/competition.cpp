# include "api.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "asnav_node");
    ros::NodeHandle nh_;

    ASNAV uav(nh_);
    ros::Rate rate(20);
     
    static constexpr float fly_height = 0.55f;

    if (!uav.takeoff(fly_height))
    {
        ROS_INFO("起飞失败，任务终止");
        return 1;
    }

    int mission_num = 1;

    while (ros::ok())
    {
        switch (mission_num)
        {
        // case 0:  // 阶段0：起飞
        //         ROS_INFO("阶段0：起飞成功，准备开始下一任务");
        //         mission_num = 1;
        //     break;

        case 1:  // 阶段1：前往至目标点1
            // if (uav.navigation(7.0f, 0.0f, fly_height, 0, 0.2f))
            // {
            //     ROS_INFO("阶段1：已到达目标点1，准备执行下一任务");
            //     mission_num = 2;
            // }
            if (uav.position(0.5f, 0.5f ,fly_height, 0.6f))
            {
                ROS_INFO("阶段1: 已到达目标点1");
                mission_num = 2;
            }             
            break;

        case 2:  // 阶段2：导航至目标点2
            if (uav.position(7.0f, 5.4f, fly_height, 0, 0.8f))
            {
                ROS_INFO("阶段2：已到达目标点2，准备执行下一任务");
                mission_num = 3;
            }
            // if (uav.controlYaw(1.57f, 0.1f))
            // {
            //     ROS_INFO("姿态已调整完毕!");
            //     mission_num = 3;
            // }
            break;

        case 3:  // 阶段4：导航至目标点3
            if (uav.position(3.0f, 4.3f, fly_height, 0, 0.4f))
            {
                ROS_INFO("阶段3：已到达目标点4，准备执行下一任务");
                mission_num = 4;
            }
            break;

        case 4:  // 阶段6：下降
            if (uav.flyDown(0.3f))
            {
                ROS_INFO("阶段6：已下降到目标高度，准备执行下一任务");
                mission_num = 5;
            }
            break;

        case 5:  // 阶段7：上升
            if (uav.flyUp(1.0))
            {
                ROS_INFO("阶段7：已上升到目标高度，准备执行下一任务");
                mission_num = 6;
            }
            break;

        case 6:  // 阶段8：自动降落
            if (uav.autoLand())
            {
                ROS_INFO("阶段8：已执行自动降落，任务完成");
                mission_num = 7;
            }
            break;

        case 7:  // 阶段9：任务完成，保持等待
            ROS_INFO_THROTTLE(5, "阶段9：任务已完成");
            ros::shutdown();
            return 0;
            break;
        
            
            
            break;
        
        default:
            ROS_WARN("未知任务阶段：%d", mission_num);
            break;
        }

        uav.setpointPublish();
        ros::spinOnce();
        rate.sleep();  
    }   
    return 0;
}
