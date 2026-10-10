/*
 *  ╔══════════════════════════════════════════════════════════════╗
 *  ║                                                              ║
 *  ║     ███████╗ ██████╗  █████╗ ██████╗ ███████╗ █████╗         ║
 *  ║     ██╔════╝ ██╔══██╗██╔══██╗██╔══██╗██╔════╝██╔══██╗        ║
 *  ║     ███████╗ ██████╔╝███████║██║  ██║█████╗  ███████║        ║
 *  ║     ╚════██║ ██╔═══╝ ██╔══██║██║  ██║██╔══╝  ██╔══██║        ║
 *  ║     ███████║ ██║     ██║  ██║██████╔╝███████╗██║  ██║        ║
 *  ║     ╚══════╝ ╚═╝     ╚═╝  ╚═╝╚═════╝ ╚══════╝╚═╝  ╚═╝        ║
 *  ║                                                              ║
 *  ║     Author    : SpadeA                                       ║
 *  ║     School    : SWPU                                         ║
 *  ║     QQ        : 3402442153                                   ║
 *  ║                                                              ║
 *  ║     ✨ 欢迎交流讨论，有问题或建议欢迎随时联系！ ✨                 ║
 *  ║     Feel free to reach out for questions or suggestions!     ║
 *  ║                                                              ║
 *  ╚══════════════════════════════════════════════════════════════╝
 */

# include "api_3d.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "asnav_node");
    ros::NodeHandle nh_;

    ASNAV uav(nh_);
    ros::Rate rate(50);  // 50Hz: PD外环需要高频更新保证速度阻尼效果
     
    // static constexpr float fly_height = 0.5f, descend_z = 0.3f;

    if (!uav.takeoff(uav.fly_height))
    {
        ROS_INFO("起飞失败，任务终止");
        return 1;
    }

    int mission_num = 0; // 任务阶段计数器

    while (ros::ok())
    {
        switch (mission_num)
        {
        case 0:  // 阶段0：起飞
            ROS_INFO("阶段0：起飞成功，准备开始下一任务");
            // if (uav.positionSmooth(0, 0, uav.fly_height, 0.2f, 20000.0f))
            // {
            //     ROS_INFO("悬停稳定完成，开始穿框");
            //     mission_num = 1;
            // }
            mission_num = 1;
            break;

        case 1:  
            if (uav.navigationSuperRviz(0))
            {
                ROS_WARN("完成！！！");
                mission_num = 2;
            }
            break;
            // if (uav.position(-2.2f, 0.0f, uav.fly_height, 0.0f, 0.3f))
            // {
            //     mission_num =2;
            // }
            // break;

        // case 2:  //阶段2：导航出避障区，准备投放打靶
        //     // 2026-08-20: 原目标(-1.96,-2.5)在南墙外+西墙膨胀层内, SUPER走廊重叠不足无法规划;
        //     // 改到场地内开阔点(-1.3,-1.5), 出墙投放交给 case3 putShoot(positionSmooth, 不走规划器)
        //     if (uav.navigationSuper(-1.8f, -2.5f, uav.fly_height, -M_PI_2, 0.3f))
        //     {
        //         ROS_WARN("完成！！！");
        //         mission_num = 3;
        //     }
        //     break;

        // case 3:  //阶段3：投放和打靶
        //     if (uav.putShoot(-0.4, -2.2, uav.fly_height, 0.0f, 0.2f))
        //     {
        //         ROS_WARN("任务三完成！！！");
        //         mission_num = 4;
        //     }
        //     break;

        // case 4:  //阶段4：准备返航
        //     if (uav.positionSmooth(-0.4f, -2.7f, uav.fly_height, 0.2f, 0.0f))
        //     {
        //         ROS_INFO("返航，穿柱中");
        //         mission_num = 5;
        //     }
        //     break;

        // case 5:  //阶段5：返航
        //     if (uav.navigationSuper(-1.8f, -2.8f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_INFO("返航，穿柱中");
        //         mission_num = 6;
        //     }
        //     break;

        // case 6:  //阶段6：从避障区返航
        //     if (uav.navigationSuper(-2.4f, -0.5f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_WARN("完成！！！");
        //         mission_num = 7;
        //     }
        //     break;

        // case 7:  //阶段7：导航回起飞点
        //     if (uav.navigationSuper(0.0f, 0.0f, uav.fly_height, NAN, 0.2f))
        //     {
        //         ROS_WARN("完成！！！");
        //         mission_num = 14;
        //     }
        //     break;
       
        // case 2:
        //     if (uav.arTrackLanding(0.0f, uav.fly_height, 0.1f, 0.15f, 0.0f, 0.08f))
        //     {
        //         ROS_INFO("AR码跟踪降落完成，准备前往下一个任务");
        //         uav.reset_target();
        //         mission_num = 15;
        //     }
        //     break;

        // case 21:
        //     if (uav.trackYoloForward(0.01f, 0.005f, 0.005f, 80.0, 20, 20))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("阶段2: 穿框完成，开始 %f 秒悬停制动...", hover_duration);
        //         mission_num = 30;
        //     }
        //     break;               

        // case 8:
        //     if (uav.flyDown(uav.descend_z))
        //     {
        //         ROS_WARN("准备开始调整yaw");
        //         uav.reset_target();
        //         mission_num = 9;
        //     }
        //     break;

        // case 9:
        //     if (uav.controlYaw(0.3f, -1.2f, uav.descend_z, -M_PI/2, 0.2f))
        //     {
        //         uav.reset_target();
        //         ROS_INFO("调整yaw结束,准备开始扎气球");
        //         mission_num = 10;
        //     }
        //     break;
                 
        case 14: 
            if(uav.autoLand())
            {
                ROS_INFO("降落成功，任务完成");
                mission_num = 15;
            }
            break;
                        
        case 15:  
            ROS_INFO_THROTTLE(5, "任务已完成");
            ros::shutdown();
            return 0;
            break;
        
        default:
            ROS_WARN_THROTTLE(2.0, "未知任务阶段：%d", mission_num);
            break;
        }

        uav.setpointPublish();
        ros::spinOnce();
        rate.sleep();  
    }   
    return 0;
}
