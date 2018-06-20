#ifndef _ZABER_BINARY_H_
#define _ZABER_BINARY_H_

#include <MMDevice.h>
#include <DeviceBase.h>
#include <ModuleInterface.h>
#include <sstream>
#include <string>

//////////////////////////////////////////////////////////////////////////////
// Various constants: error codes, error messages
//////////////////////////////////////////////////////////////////////////////

#define ERR_PORT_CHANGE_FORBIDDEN    10002
#define ERR_DRIVER_DISABLED          10004
#define ERR_BUSY_TIMEOUT             10008
#define ERR_AXIS_COUNT               10016
#define ERR_COMMAND_REJECTED         10032
#define	ERR_NO_REFERENCE_POS         10064
#define	ERR_SETTING_FAILED           10128
#define	ERR_INVALID_DEVICE_NUM       10256

extern const char* g_Msg_PORT_CHANGE_FORBIDDEN;
extern const char* g_Msg_DRIVER_DISABLED;
extern const char* g_Msg_BUSY_TIMEOUT;
extern const char* g_Msg_AXIS_COUNT;
extern const char* g_Msg_COMMAND_REJECTED;
extern const char* g_Msg_NO_REFERENCE_POS;
extern const char* g_Msg_SETTING_FAILED;
extern const char* g_Msg_INVALID_DEVICE_NUM;

//Stage-specific constants

extern const char* g_StageName;
extern const char* g_StageDescription;

class ZaberBinaryStage: public CStageBase<ZaberBinaryStage>

{
public:
	ZaberBinaryStage();
	~ZaberBinaryStage();

	/*
	// Device API
	// ----------
	int Initialize();
	int Shutdown();
	void GetName(char* name) const;
	bool Busy();

	// Stage API
	// ---------
	int GetPositionUm(double& pos);
	int GetPositionSteps(long& steps);
	int SetPositionUm(double pos);
	int SetRelativePositionUm(double d);
	int SetPositionSteps(long steps);
	int SetRelativePositionSteps(long steps);
	int Move(double velocity);
	int Stop();
	int Home();
	int SetAdapterOriginUm(double d);
	int SetOrigin();
	int GetLimits(double& lower, double& upper);

	int IsStageSequenceable(bool& isSequenceable) const {isSequenceable = false; return DEVICE_OK;}
	bool IsContinuousFocusDrive() const {return false;}

	// action interface
	// ----------------
	int OnPort          (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnDeviceAddress (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnAxisNumber       (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnMotorSteps    (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnLinearMotion  (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnSpeed         (MM::PropertyBase* pProp, MM::ActionType eAct);
	int OnAccel         (MM::PropertyBase* pProp, MM::ActionType eAct);

	protected:
	int ClearPort() const;
	int SendCommand(const std::string command) const;
	int QueryCommand(const std::string command, std::vector<std::string>& reply) const;
	int GetSetting(long device, long axis, std::string setting, long& data) const;
	int SetSetting(long device, long axis, std::string setting, long data) const;
	bool IsBusy(long device) const;
	int Stop(long device) const;
	int GetLimits(long device, long axis, long& min, long& max) const;
	int SendMoveCommand(long device, long axis, std::string type, long data) const;
	int SendAndPollUntilIdle(long device, long axis, std::string command, int timeoutMs) const;

	bool initialized_;
	std::string port_;
	MM::Device *device_;
	MM::Core *core_;
	std::string cmdPrefix_;

private:
	long deviceAddress_;
	long axisNumber_;
	int homingTimeoutMs_;
	double stepSizeUm_;
	double convFactor_; // not very informative name
	std::string cmdPrefix_;
	long resolution_;
	long motorSteps_;
	double linearMotion_;
	*/

};

#endif //_ZABER_BINARY_H_