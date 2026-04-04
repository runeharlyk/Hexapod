#include "communication/comm_base.h"

// Static member definitions
EventBus<CommandMsg>::Handle CommAdapterBase::_cmdSubHandle;
EventBus<ModeMsg>::Handle CommAdapterBase::_modeSubHandle;
EventBus<GaitMsg>::Handle CommAdapterBase::_gaitSubHandle;
EventBus<ServoAnglesMsg>::Handle CommAdapterBase::_angleSubHandle;
EventBus<ServoSignalMsg>::Handle CommAdapterBase::_servoSubHandle;
EventBus<ServoSettingsMsg>::Handle CommAdapterBase::_servoSettingsMsgSubHandle;

std::map<message_topic_t, bool> CommAdapterBase::busActive_;
SemaphoreHandle_t CommAdapterBase::busMutex_ = nullptr;
thread_local bool CommAdapterBase::_incomingMessage = false;
