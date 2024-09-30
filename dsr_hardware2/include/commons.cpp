#include "commons.hpp"

#include <chrono>
#include <DRFC.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>

using DRAFramework::CDRFLEx;
using std::cerr;
using std::endl;

std::tuple<Mat6X, Mat6X, Mat6X>
plan_motion(const Vec6& qcurr, const double dt, const double t_tot) {
    const double freq_hz   = 0.125;
    const long   n_samples = std::ceil(t_tot / dt);
    const Vec6   qdelta{0.0, 0.0, 3.0, 0.0, 0.0, 0.0};
    Mat6X        q_des(6, n_samples);
    Mat6X        qd_des(6, n_samples);
    Mat6X        qdd_des(6, n_samples);

    for (long i{0}; i < n_samples; ++i) {
        const double t   = i * dt;
        const double arg = 2.0 * M_PI * freq_hz;
        q_des.col(i)     = qcurr + qdelta * std::sin(arg * t);
        qd_des.col(i)    = arg * qdelta * std::cos(arg * t);
        qdd_des.col(i)   = -arg * arg * qdelta * std::sin(arg * t);
    }
    return {q_des, qd_des, qdd_des};
}

void
on_state_change(const ROBOT_STATE eState) {
    std::string desc;
    switch (eState) {
        case STATE_INITIALIZING:
            desc = "(0) INITIALIZING";
            break;
        case STATE_STANDBY:
            desc = "(1) STANDBY";
            break;
        case STATE_MOVING:
            desc = "(2) MOVING";
            break;
        case STATE_SAFE_OFF:
            desc = "(3) SAFE_OFF";
            break;
        case STATE_TEACHING:
            desc = "(4) TEACHING";
            break;
        case STATE_SAFE_STOP:
            desc = "(5) SAFE_STOP";
            break;
        case STATE_EMERGENCY_STOP:
            desc = "(6) EMERGENCY_STOP";
            break;
        case STATE_HOMMING:
            desc = "(7) HOMMING";
            break;
        case STATE_RECOVERY:
            desc = "(8) RECOVERY";
            break;
        case STATE_SAFE_STOP2:
            desc = "(9) SAFE_STOP2";
            break;
        case STATE_SAFE_OFF2:
            desc = "(10) SAFE_OFF2";
            break;
        case STATE_RESERVED1:
            desc = "(11) RESERVED1";
            break;
        case STATE_RESERVED2:
            desc = "(12) RESERVED2";
            break;
        case STATE_RESERVED3:
            desc = "(13) RESERVED3";
            break;
        case STATE_RESERVED4:
            desc = "(14) RESERVED4";
            break;
        case STATE_NOT_READY:
            desc = "(15) NOT_READY";
            break;
        default:
            desc = "UNKNOWN";
            break;
    }
    cerr << "Switching to state " << eState << " - " << desc << endl;
}

void
on_rt_monitoring_data(LPRT_OUTPUT_DATA_LIST tData) {
    return;
    static int td = 0;
    if (td++ == 1000) {
        td = 0;
        printf("timestamp : %.3f\n", tData->time_stamp);
        printf("actual_joint_position : %f %f %f %f %f %f\n",
               tData->actual_joint_position[0],
               tData->actual_joint_position[1],
               tData->actual_joint_position[2],
               tData->actual_joint_position[3],
               tData->actual_joint_position[4],
               tData->actual_joint_position[5]);
        printf("actual_motor_torque : %f %f %f %f %f %f\n",
               tData->actual_motor_torque[0],
               tData->actual_motor_torque[1],
               tData->actual_motor_torque[2],
               tData->actual_motor_torque[3],
               tData->actual_motor_torque[4],
               tData->actual_motor_torque[5]);
        printf("actual_grav_torque : %f %f %f %f %f %f\n",
               tData->gravity_torque[0],
               tData->gravity_torque[1],
               tData->gravity_torque[2],
               tData->gravity_torque[3],
               tData->gravity_torque[4],
               tData->gravity_torque[5]);
        printf("target torque : %f %f %f %f %f %f\n",
               tData->target_motor_torque[0],
               tData->target_motor_torque[1],
               tData->target_motor_torque[2],
               tData->target_motor_torque[3],
               tData->target_motor_torque[4],
               tData->target_motor_torque[5]);
    }
}

void
goto_standy(DRAFramework::CDRFLEx* drfl) {
    while (drfl->GetRobotState() == STATE_SAFE_OFF) {
        cerr << "Enablig control servo" << endl;
        drfl->set_robot_control(CONTROL_SERVO_ON);
        drfl->set_robot_mode(ROBOT_MODE_MANUAL);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int
linux_kbhit(void) {
    struct termios oldt, newt;
    int            ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);


    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int
getch() {
    int            c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr);
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);
    newattr.c_cc[VMIN]  = 1;
    newattr.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);
    c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);
    return c;
}

void
plan_trajectory(PlanParam* plan) {
    float ps[6], vs[6], as[6];
    float pf[6], vf[6], af[6];
    float tf;

    tf = plan->time;

    for (int i = 0; i < 6; i++) {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for (int i = 0; i < 6; i++) {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i] / 2;
        plan->A3[i] = (20 * pf[i] - 20 * ps[i] - (8 * vf[i] + 12 * vs[i]) * tf
                       - (3 * as[i] - af[i]) * tf * tf)
                      / (2 * tf * tf * tf);
        plan->A4[i] = (30 * ps[i] - 30 * pf[i] + (14 * vf[i] + 16 * vs[i]) * tf
                       + (3 * as[i] - 2 * af[i]) * tf * tf)
                      / (2 * tf * tf * tf * tf);
        plan->A5[i] = (12 * pf[i] - 12 * ps[i] - (6 * vf[i] + 6 * vs[i]) * tf
                       - (as[i] - af[i]) * tf * tf)
                      / (2 * tf * tf * tf * tf * tf);
    }
}

void
generate_trajectory_sample(PlanParam* plan, TraParam* tra) {
    double A0[6], A1[6], A2[6], A3[6], A4[6], A5[6];
    double t = tra->time;

    for (int i = 0; i < 6; i++) {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for (int i = 0; i < 6; i++) {
        tra->pos[i] = A0[i] + A1[i] * t + A2[i] * t * t + A3[i] * t * t * t
                      + A4[i] * t * t * t * t + A5[i] * t * t * t * t * t;
        tra->vel[i] = A1[i] + 2 * A2[i] * t + 3 * A3[i] * t * t + 4 * A4[i] * t * t * t
                      + 5 * A5[i] * t * t * t * t;
        tra->acc[i] =
                2 * A2[i] + 6 * A3[i] * t + 12 * A4[i] * t * t + 20 * A5[i] * t * t * t;
    }

    // If interpolation time is finished, leave final sample and zero vel/acc
    if (tra->time > plan->time){
        for(int i = 0; i < 6; ++i){
            tra->pos[i] = plan->pf[i];
            tra->vel[i] = 0.0;
            tra->acc[i] = 0.0;
        }
    }
}

void
configure_plan_parameters(PlanParam* plan) {
    plan->time  = 10;
    plan->ps[0] = 0;
    plan->ps[1] = 0;
    plan->ps[2] = 0;
    plan->ps[3] = 0;
    plan->ps[4] = 0;
    plan->ps[5] = 0;
    plan->pf[0] = 0;
    plan->pf[1] = 0;
    plan->pf[2] = 0;
    plan->pf[3] = 30;
    plan->pf[4] = 30;
    plan->pf[5] = 30;
    plan->vs[0] = 0;
    plan->vs[1] = 0;
    plan->vs[2] = 0;
    plan->vs[3] = 0;
    plan->vs[4] = 0;
    plan->vs[5] = 0;
    plan->vf[0] = 0;
    plan->vf[1] = 0;
    plan->vf[2] = 0;
    plan->vf[3] = 0;
    plan->vf[4] = 0;
    plan->vf[5] = 0;
    plan->as[0] = 0;
    plan->as[1] = 0;
    plan->as[2] = 0;
    plan->as[3] = 0;
    plan->as[4] = 0;
    plan->as[5] = 0;
    plan->af[0] = 0;
    plan->af[1] = 0;
    plan->af[2] = 0;
    plan->af[3] = 0;
    plan->af[4] = 0;
    plan->af[5] = 0;
}
