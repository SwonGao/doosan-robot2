#ifndef COMMONS__HPP
#define COMMONS__HPP

#include <DRFS.h>
#include <Eigen/Dense>

#include "DRFC.h"
#include "DRFLEx.h"

using Mat6X   = Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor>;
using Mat6    = Eigen::Matrix<float, 6, 6>;
using Vec6    = Eigen::Vector<float, 6>;
using Vec6Map = Eigen::Map<Vec6>;

struct PlanParam {
    float time;

    float ps[6];
    float vs[6];
    float as[6];
    float pf[6];
    float vf[6];
    float af[6];

    float A0[6];
    float A1[6];
    float A2[6];
    float A3[6];
    float A4[6];
    float A5[6];
};

struct TraParam {
    float time;

    float pos[6];
    float vel[6];
    float acc[6];
};

void on_state_change(const ROBOT_STATE eState);

void on_rt_monitoring_data(LPRT_OUTPUT_DATA_LIST tData);


std::tuple<Mat6X, Mat6X, Mat6X> plan_motion(
        const Vec6& qcurr, const double dt, const double t_tot
);

void goto_standy(DRAFramework::CDRFLEx* drfl);

void plan_trajectory(PlanParam* plan);

void generate_trajectory_sample(PlanParam* plan, TraParam* tra);


/**
 * @brief Assumes initial configuration q = 0
 *
 */
void configure_plan_parameters(PlanParam* plan);

int linux_kbhit(void);
int getch();

namespace callback {
void on_tp_popup(LPMESSAGE_POPUP popup);
void on_tp_log(const char* strLog);
void on_tp_progress(LPMESSAGE_PROGRESS progress);
void on_tp_get_user_input(LPMESSAGE_INPUT input);
void on_homing_completed();
void on_monitoring_data(const LPMONITORING_DATA pData);
void on_monitoring_data_exchange(const LPMONITORING_DATA_EX pData);
void on_monitoring_ctrl_io(const LPMONITORING_CTRLIO pData);
void on_monitoring_ctrl_io_exchange(const LPMONITORING_CTRLIO_EX pData);
void on_log_alarm(LPLOG_ALARM tLog);
}  // namespace callback


#endif  // COMMONS__HPP
