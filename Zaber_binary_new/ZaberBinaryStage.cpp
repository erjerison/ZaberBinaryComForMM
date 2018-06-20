#ifdef WIN32
#define snprintf _snprintf 
#pragma warning(disable: 4355)
#endif

#include "ZaberBinaryStage.h"

using namespace std;

const char* g_Msg_PORT_CHANGE_FORBIDDEN = "The port cannot be changed once the device is initialized.";
const char* g_Msg_DRIVER_DISABLED = "The driver has disabled itself due to overheating.";
const char* g_Msg_BUSY_TIMEOUT = "Timed out while waiting for device to finish executing a command.";
const char* g_Msg_AXIS_COUNT = "Dual-axis controller required.";
const char* g_Msg_COMMAND_REJECTED = "The device rejected the command.";
const char* g_Msg_NO_REFERENCE_POS = "The device has not had a reference position established.";
const char* g_Msg_SETTING_FAILED = "The property could not be set. Is the value in the valid range?";
const char* g_Msg_INVALID_DEVICE_NUM = "Device numbers must be in the range of 1 to 99.";

const char* g_StageName = "Stage";
const char* g_StageDescription = "Zaber Stage";


ZaberBinaryStage::ZaberBinaryStage()
{
}


ZaberBinaryStage::~ZaberBinaryStage()
{
}
