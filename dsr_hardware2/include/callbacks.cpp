#include <iostream>

#include "commons.hpp"

using std::cout, std::endl;

void
callback::on_tp_popup(LPMESSAGE_POPUP tPopup) {
    cout << "Popup Message: " << tPopup->_szText << endl;
    cout << "Message Level: " << tPopup->_iLevel << endl;
    cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void
callback::on_tp_log(const char* strLog) {
    cout << "Log Message: " << strLog << endl;
}

void
callback::on_tp_progress(LPMESSAGE_PROGRESS tProgress) {
    cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
    cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void
callback::on_tp_get_user_input(LPMESSAGE_INPUT tInput) {
    cout << "User Input : " << tInput->_szText << endl;
    cout << "Data Type : " << (int)tInput->_iType << endl;
}

void
callback::on_homing_completed() {
    cout << "homming completed" << endl;
}

void
callback::on_monitoring_data(const LPMONITORING_DATA pData) {
    return;
    cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
         << pData->_tCtrl._tTask._fActualPos[0][1]
         << pData->_tCtrl._tTask._fActualPos[0][2]
         << pData->_tCtrl._tTask._fActualPos[0][3]
         << pData->_tCtrl._tTask._fActualPos[0][4]
         << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void
callback::on_monitoring_data_exchange(const LPMONITORING_DATA_EX pData) {
    return;
    cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
         << pData->_tCtrl._tWorld._fTargetPos[1] << pData->_tCtrl._tWorld._fTargetPos[2]
         << pData->_tCtrl._tWorld._fTargetPos[3] << pData->_tCtrl._tWorld._fTargetPos[4]
         << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void
callback::on_monitoring_ctrl_io(const LPMONITORING_CTRLIO pData) {
    return;
    cout << "# monitoring ctrl 0 data" << endl;
    for (int i = 0; i < 16; i++) { cout << (int)pData->_tInput._iActualDI[i] << endl; }
}

void
callback::on_monitoring_ctrl_io_exchange(const LPMONITORING_CTRLIO_EX pData) {
    return;
    cout << "# monitoring ctrl 1 data" << endl;
    for (int i = 0; i < 16; i++) { cout << (int)pData->_tInput._iActualDI[i] << endl; }
    for (int i = 0; i < 16; i++) { cout << (int)pData->_tOutput._iTargetDO[i] << endl; }
}

void
callback::on_log_alarm(LPLOG_ALARM tLog) {
    cout << "Alarm Info: "
         << "group(" << (unsigned int)tLog->_iGroup << "), index(" << tLog->_iIndex
         << "), param(" << tLog->_szParam[0] << "), param(" << tLog->_szParam[1]
         << "), param(" << tLog->_szParam[2] << ")" << endl;
}
