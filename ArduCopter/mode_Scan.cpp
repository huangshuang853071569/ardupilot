#include "Copter.h"

/*
 * Init and run calls for guided flight mode
 */


// guided_init - initialise guided controller
bool Copter::ModeDrawStar::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        // initialise yaw
        auto_yaw.set_mode_to_default(false);

        path_num = 0;               //二开添加：当前航点号清0
        generate_path();            //二开添加：生成航点

        // start in position control mode
        pos_control_start();
        return true;
    }else{
        return false;
    }
}

//二开添加：生成巡逻航线的航点
//航线坐标定义：X轴正向指向正北，Y轴正向指向正东，Z轴正向垂直向下
void Copter::ModeDrawStar::generate_path()
{
    float scan_long_cm = 1000;              //扫描长度
    float scan_width_cm = 500;              //扫描宽度
    int i = 1;

    wp_nav->get_wp_stopping_point(path[0]);                 //将当前的位置坐标存放在第0航点，作为起始点

    //生成航点
    for(i = 1; i <= scan_times; i++)
    {
        path[(i-1)*4 + 0] =  path[0] + Vector3f(0,              (i-1)*scan_width_cm*2,                    0);
        path[(i-1)*4 + 1] =  path[0] + Vector3f(scan_long_cm,   (i-1)*scan_width_cm*2,                    0);
        path[(i-1)*4 + 2] =  path[0] + Vector3f(scan_long_cm,   (i-1)*scan_width_cm*2 + scan_width_cm,    0);
        path[(i-1)*4 + 3] =  path[0] + Vector3f(0,              (i-1)*scan_width_cm*2 + scan_width_cm,    0);
    }

}


// initialise guided mode's position controller
void Copter::ModeDrawStar::pos_control_start()
{
    // set to position control mode
    //guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();


    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false); //二开修改：目标航点不再使用停止点，而是第0个航点

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}



// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeDrawStar::run()
{
    if(wp_nav->reached_wp_destination()){                       //二开添加：如果到达了期望航点
        if(path_num < (scan_times*4-1)){                        //二开添加：如果还没到最后一个航点
            path_num ++;
            wp_nav->set_wp_destination(path[path_num], false);  //二开添加：将目标航点设置为下一个航点
        }
    }
    pos_control_run();
 }


// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::ModeDrawStar::pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}
