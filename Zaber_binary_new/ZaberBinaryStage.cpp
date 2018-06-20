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

//////////////////////////////////////////////////////////////////////////////////
// Exported MMDevice API
//////////////////////////////////////////////////////////////////////////////////
MODULE_API void InitializeModuleData()
{
	RegisterDevice(g_StageName, MM::StageDevice, g_StageDescription);
}                                                            


MODULE_API MM::Device* CreateDevice(const char* deviceName)                  
{
	if (strcmp(deviceName, g_StageName) == 0)
	{	
		return new ZaberBinaryStage();
	}
	else
	{	
		return 0;
	}
}


MODULE_API void DeleteDevice(MM::Device* pDevice)
{
	delete pDevice;
}

ZaberBinaryStage::ZaberBinaryStage() :
	deviceAddress_(1),
	axisNumber_(1),
	homingTimeoutMs_(20000),
	stepSizeUm_(0.15625),
	convFactor_(1.6384), // not very informative name
	cmdPrefix_("/"),
	resolution_(64),
	motorSteps_(200),
	linearMotion_(2.0),
	initialized_(false),
	port_("Undefined"),
	core_(0)
{
	this->LogMessage("Stage::Stage\n", true);

	InitializeDefaultErrorMessages();
	SetErrorText(ERR_PORT_CHANGE_FORBIDDEN, g_Msg_PORT_CHANGE_FORBIDDEN);
	SetErrorText(ERR_DRIVER_DISABLED, g_Msg_DRIVER_DISABLED);
	SetErrorText(ERR_BUSY_TIMEOUT, g_Msg_BUSY_TIMEOUT);
	SetErrorText(ERR_COMMAND_REJECTED, g_Msg_COMMAND_REJECTED);
	SetErrorText(ERR_SETTING_FAILED, g_Msg_SETTING_FAILED);

	// Used-to-be baseclass device_
	this->device_ = this;

	// Pre-initialization properties
	CreateProperty(MM::g_Keyword_Name, g_StageName, MM::String, true);

	CreateProperty(MM::g_Keyword_Description, "Zaber stage driver adapter", MM::String, true);

	CPropertyAction* pAct = new CPropertyAction (this, &ZaberBinaryStage::OnPort);
	CreateProperty(MM::g_Keyword_Port, "COM1", MM::String, false, pAct, true);

	pAct = new CPropertyAction (this, &ZaberBinaryStage::OnDeviceAddress);
	CreateIntegerProperty("Controller Device Number", deviceAddress_, false, pAct, true);
	SetPropertyLimits("Controller Device Number", 1, 99);

	pAct = new CPropertyAction(this, &ZaberBinaryStage::OnAxisNumber);
	CreateIntegerProperty("Axis Number", axisNumber_, false, pAct, true);
	SetPropertyLimits("Axis Number", 1, 9);

	pAct = new CPropertyAction(this, &ZaberBinaryStage::OnMotorSteps);
	CreateIntegerProperty("Motor Steps Per Rev", motorSteps_, false, pAct, true);

	pAct = new CPropertyAction(this, &ZaberBinaryStage::OnLinearMotion);
	CreateFloatProperty("Linear Motion Per Motor Rev [mm]", linearMotion_, false, pAct, true);
}

ZaberBinaryStage::~ZaberBinaryStage()
{
	this->LogMessage("Stage::~Stage\n", true);
	Shutdown();
}

///////////////////////////////////////////////////////////////////////////////
// Stage & Device API methods
///////////////////////////////////////////////////////////////////////////////

void ZaberBinaryStage::GetName(char* name) const
{
	CDeviceUtils::CopyLimitedString(name, g_StageName);
}

int ZaberBinaryStage::Initialize()
{
	if (initialized_) return DEVICE_OK;

	core_ = GetCoreCallback();
	
	this->LogMessage("Stage::Initialize\n", true);

	int ret = ClearPort();
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	// Disable alert messages.
	ret = SetSetting(deviceAddress_, 0, "comm.alert", 0);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	// Calculate step size.
	ret = GetSetting(deviceAddress_, axisNumber_, "resolution", resolution_);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}
	stepSizeUm_ = ((double)linearMotion_/(double)motorSteps_)*(1/(double)resolution_)*1000;

	CPropertyAction* pAct;
	// Initialize Speed (in mm/s)
	pAct = new CPropertyAction (this, &ZaberBinaryStage::OnSpeed);
	ret = CreateFloatProperty("Speed [mm/s]", 0.0, false, pAct);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	// Initialize Acceleration (in m/s²)
	pAct = new CPropertyAction (this, &ZaberBinaryStage::OnAccel);
	ret = CreateFloatProperty("Acceleration [m/s^2]", 0.0, false, pAct);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	ret = UpdateStatus();
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	if (ret == DEVICE_OK)
	{
		initialized_ = true;
	}

	return ret;
}

int ZaberBinaryStage::Shutdown()
{
	this->LogMessage("Stage::Shutdown\n", true);
	if (initialized_)
	{
		initialized_ = false;
	}
	return DEVICE_OK;
}

bool ZaberBinaryStage::Busy()
{
	this->LogMessage("Stage::Busy\n", true);
	return IsBusy(deviceAddress_);
}

int ZaberBinaryStage::GetPositionUm(double& pos)
{
	this->LogMessage("Stage::GetPositionUm\n", true);
	
	long steps;
	int ret =  GetSetting(deviceAddress_, axisNumber_, "pos", steps);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}
	pos = steps * stepSizeUm_;
	return DEVICE_OK;
}

int ZaberBinaryStage::GetPositionSteps(long& steps)
{
	this->LogMessage("Stage::GetPositionSteps\n", true);
	return GetSetting(deviceAddress_, axisNumber_, "pos", steps);
}

int ZaberBinaryStage::SetPositionUm(double pos)
{
	this->LogMessage("Stage::SetPositionUm\n", true);
	long steps = nint(pos/stepSizeUm_);
	return SetPositionSteps(steps);
}

int ZaberBinaryStage::SetRelativePositionUm(double d)
{
	this->LogMessage("Stage::SetRelativePositionUm\n", true);
	long steps = nint(d/stepSizeUm_);
	return SetRelativePositionSteps(steps);
}

int ZaberBinaryStage::SetPositionSteps(long steps)
{
	this->LogMessage("Stage::SetPositionSteps\n", true);
	return SendMoveCommand(deviceAddress_, axisNumber_, "abs", steps);
}

int ZaberBinaryStage::SetRelativePositionSteps(long steps)
{
	this->LogMessage("Stage::SetRelativePositionSteps\n", true);
	return SendMoveCommand(deviceAddress_, axisNumber_, "rel", steps);
}

int ZaberBinaryStage::Move(double velocity)
{
	this->LogMessage("Stage::Move\n", true);
	// convert velocity from mm/s to Zaber data value
	long velData = nint(velocity*convFactor_*1000/stepSizeUm_);
	return SendMoveCommand(deviceAddress_, axisNumber_, "vel", velData);
}

int ZaberBinaryStage::Stop()
{
	this->LogMessage("Stage::Stop\n", true);
	return ZaberBinaryStage::Stop(deviceAddress_);
}

int ZaberBinaryStage::Home()
{
	this->LogMessage("Stage::Home\n", true);
	//TODO try tools findrange first?
	ostringstream cmd;
	cmd << cmdPrefix_ << "home";
	return SendAndPollUntilIdle(deviceAddress_, axisNumber_, cmd.str().c_str(), homingTimeoutMs_);
}

int ZaberBinaryStage::SetAdapterOriginUm(double /*d*/)
{
	this->LogMessage("Stage::SetAdapterOriginUm\n", true);
	return DEVICE_UNSUPPORTED_COMMAND;
}

int ZaberBinaryStage::SetOrigin()
{
	this->LogMessage("Stage::SetOrigin\n", true);
	return DEVICE_UNSUPPORTED_COMMAND;
}

int ZaberBinaryStage::GetLimits(double& lower, double& upper)
{
	this->LogMessage("Stage::GetLimits\n", true);

	long min, max;
	int ret = ZaberBinaryStage::GetLimits(deviceAddress_, axisNumber_, min, max);
	if (ret != DEVICE_OK)
	{
		return ret;
	}
	lower = (double)min;
	upper = (double)max;
	return DEVICE_OK;
}

///////////////////////////////////////////////////////////////////////////////
// Action handlers
// Handle changes and updates to property values.
///////////////////////////////////////////////////////////////////////////////

int ZaberBinaryStage::OnPort (MM::PropertyBase* pProp, MM::ActionType eAct)
{
	ostringstream os;
	os << "Stage::OnPort(" << pProp << ", " << eAct << ")\n";
	this->LogMessage(os.str().c_str(), false);

	if (eAct == MM::BeforeGet)
	{
		pProp->Set(port_.c_str());
	}
	else if (eAct == MM::AfterSet)
	{
		if (initialized_)
		{
			// revert
			pProp->Set(port_.c_str());
			return ERR_PORT_CHANGE_FORBIDDEN;
		}

		pProp->Get(port_);
	}
	return DEVICE_OK;
}

int ZaberBinaryStage::OnDeviceAddress (MM::PropertyBase* pProp, MM::ActionType eAct)
{
	this->LogMessage("Stage::OnDeviceAddress\n", true);

	if (eAct == MM::AfterSet)
	{
		pProp->Get(deviceAddress_);

		ostringstream cmdPrefix;
		cmdPrefix << "/" << deviceAddress_ << " ";
		cmdPrefix_ = cmdPrefix.str();
	}
	else if (eAct == MM::BeforeGet)
	{
		pProp->Set(deviceAddress_);
	}
	return DEVICE_OK;
}

int ZaberBinaryStage::OnAxisNumber (MM::PropertyBase* pProp, MM::ActionType eAct)
{
	this->LogMessage("Stage::OnAxisNumber\n", true);

	if (eAct == MM::BeforeGet)
	{
		pProp->Set(axisNumber_);
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(axisNumber_);
	}
	return DEVICE_OK;
}

int ZaberBinaryStage::OnSpeed (MM::PropertyBase* pProp, MM::ActionType eAct)
{
	this->LogMessage("Stage::OnSpeed\n", true);

	if (eAct == MM::BeforeGet)
	{
		long speedData;
		int ret = GetSetting(deviceAddress_, axisNumber_, "maxspeed", speedData);
		if (ret != DEVICE_OK) 
		{
			return ret;
		}

		// convert to mm/s
		double speed = (speedData/convFactor_)*stepSizeUm_/1000;
		pProp->Set(speed);
	}
	else if (eAct == MM::AfterSet)
	{
		double speed;
		pProp->Get(speed);

		// convert to data
		long speedData = nint(speed*convFactor_*1000/stepSizeUm_);
		if (speedData == 0 && speed != 0) speedData = 1; // Avoid clipping to 0.

		int ret = SetSetting(deviceAddress_, axisNumber_, "maxspeed", speedData);
		if (ret != DEVICE_OK) 
		{
			return ret;
		}
	}
	return DEVICE_OK;
}

int ZaberBinaryStage::OnAccel (MM::PropertyBase* pProp, MM::ActionType eAct)
{
	this->LogMessage("Stage::OnAccel\n", true);

	if (eAct == MM::BeforeGet)
	{
		long accelData;
		int ret = GetSetting(deviceAddress_, axisNumber_, "accel", accelData);
		if (ret != DEVICE_OK) 
		{
			return ret;
		}

		// convert to m/s²
		double accel = (accelData*10/convFactor_)*stepSizeUm_/1000;
		pProp->Set(accel);
	}
	else if (eAct == MM::AfterSet)
	{
		double accel;
		pProp->Get(accel);

		// convert to data
		long accelData = nint(accel*convFactor_*100/(stepSizeUm_));
		if (accelData == 0 && accel != 0) accelData = 1; // Only set accel to 0 if user intended it.

		int ret = SetSetting(deviceAddress_, axisNumber_, "accel", accelData);
		if (ret != DEVICE_OK) 
		{
			return ret;
		}
	}
	return DEVICE_OK;
}

int ZaberBinaryStage::OnMotorSteps(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	this->LogMessage("Stage::OnMotorSteps\n", true);

	if (eAct == MM::BeforeGet)
	{
		pProp->Set(motorSteps_);
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(motorSteps_);
	}
	return DEVICE_OK;
}

int ZaberBinaryStage::OnLinearMotion(MM::PropertyBase* pProp, MM::ActionType eAct)
{
	this->LogMessage("Stage::OnLinearMotion\n", true);

	if (eAct == MM::BeforeGet)
	{
		pProp->Set(linearMotion_);
	}
	else if (eAct == MM::AfterSet)
	{
		pProp->Get(linearMotion_);
	}
	return DEVICE_OK;
}

// COMMUNICATION "clear buffer" utility function:
int ZaberBinaryStage::ClearPort() const
{
	core_->LogMessage(device_, "ZaberBinaryStage::ClearPort\n", true);

	const int bufSize = 255;
	unsigned char clear[bufSize];
	unsigned long read = bufSize;
	int ret;

	while ((int) read == bufSize)
	{
		ret = core_->ReadFromSerial(device_, port_.c_str(), clear, bufSize, read); //Replace ReadFromSerial with suitable binary
		if (ret != DEVICE_OK) 
		{
			return ret;
		}
	}

	return DEVICE_OK;      
}


// COMMUNICATION "send" utility function:
int ZaberBinaryStage::SendCommand(const string command) const //Args of SendCommand need to change
{
	core_->LogMessage(device_, "ZaberBinaryStage::SendCommand\n", true);

	const char* msgFooter = "\n"; // required by Zaber ASCII protocol
	string baseCommand = "";
	baseCommand += command;
	return core_->SetSerialCommand(device_, port_.c_str(), baseCommand.c_str(), msgFooter); //Replace SetSerialCommand with suitable binary
}


// COMMUNICATION "send & receive" utility function:
int ZaberBinaryStage::QueryCommand(const string command, vector<string>& reply) const //Args of QueryCommand need to change
{
	core_->LogMessage(device_, "ZaberBinaryStage::QueryCommand\n", true);

	const char* msgFooter = "\r\n"; // required by Zaber ASCII protocol

	const size_t BUFSIZE = 2048;
	char buf[BUFSIZE] = {'\0'};

	int ret = SendCommand(command);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	ret = core_->GetSerialAnswer(device_, port_.c_str(), BUFSIZE, buf, msgFooter); //Replace GetSerialAnswer with suitable binary
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	string resp = buf;
	if (resp.length() < 1)
	{
		return  DEVICE_SERIAL_INVALID_RESPONSE;
	}

	// remove checksum before parsing
	int thirdLast = int(resp.length() - 3);
	if (resp[thirdLast] == ':')
	{
		resp.erase(thirdLast, string::npos);
	}

	CDeviceUtils::Tokenize(resp, reply, " ");
	/* reply[0] = message type and device address, reply[1] = axis number,
	 * reply[2] = reply flags, reply[3] = device status, reply[4] = warning flags,
	 * reply[5] (and possibly reply[6]) = response data, if there is data
	 */
	if (reply.size() < 5)
	{
		return DEVICE_SERIAL_INVALID_RESPONSE;
	}

	if (reply[4] == "FD")
	{
		return ERR_DRIVER_DISABLED;
	}

	if (reply[2] == "RJ")
	{
		return ERR_COMMAND_REJECTED;
	}

	return DEVICE_OK;
}


int ZaberBinaryStage::GetSetting(long device, long axis, string setting, long& data) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::GetSetting\n", true);

	ostringstream cmd;
	cmd << cmdPrefix_ << device << " " << axis << " get " << setting; //cmd needs translation to binary
	vector<string> resp;

	int ret = QueryCommand(cmd.str().c_str(), resp); //args of QueryCommand need to change
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	// extract data
	string dataString = resp[5];
	stringstream(dataString) >> data;
	return DEVICE_OK;
}


int ZaberBinaryStage::SetSetting(long device, long axis, string setting, long data) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::SetSetting\n", true);

	ostringstream cmd; 
	cmd << cmdPrefix_ << device << " " << axis << " set " << setting << " " << data; //cmd needs translation to binary
	vector<string> resp;

	int ret = QueryCommand(cmd.str().c_str(), resp); //args of QueryCommand may need to change
	if (ret != DEVICE_OK)
	{
		return ERR_SETTING_FAILED;
	}

	return DEVICE_OK;
}


bool ZaberBinaryStage::IsBusy(long device) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::IsBusy\n", true);

	ostringstream cmd;
	cmd << cmdPrefix_ << device; //cmd needs translation to binary
	vector<string> resp;

	int ret = QueryCommand(cmd.str().c_str(), resp); //args of QueryCommand may need to change
	if (ret != DEVICE_OK)
	{
		ostringstream os;
		os << "SendSerialCommand failed in ZaberBinaryStage::IsBusy, error code: " << ret;
		core_->LogMessage(device_, os.str().c_str(), false);
		return false;
	}

	return (resp[3] == ("BUSY"));
}


int ZaberBinaryStage::Stop(long device) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::Stop\n", true);

	ostringstream cmd;
	cmd << cmdPrefix_ << device << " stop"; //cmd needs translation to binary
	vector<string> resp;
	return QueryCommand(cmd.str().c_str(), resp); //args of QueryCommand may need to change
}


int ZaberBinaryStage::GetLimits(long device, long axis, long& min, long& max) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::GetLimits\n", true);

	int ret = GetSetting(device, axis, "limit.min", min);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	return GetSetting(device, axis, "limit.max", max);
}


int ZaberBinaryStage::SendMoveCommand(long device, long axis, std::string type, long data) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::SendMoveCommand\n", true);

	ostringstream cmd;
	cmd << cmdPrefix_ << device << " " << axis << " move " << type << " " << data; //cmd needs translation to binary
	vector<string> resp;
	return QueryCommand(cmd.str().c_str(), resp); //args of QueryCommand need to change
}


int ZaberBinaryStage::SendAndPollUntilIdle(long device, long axis, string command, int timeoutMs) const
{
	core_->LogMessage(device_, "ZaberBinaryStage::SendAndPollUntilIdle\n", true);

	ostringstream cmd;
	cmd << cmdPrefix_ << device << " " << axis << " " << command; //cmd needs translation to binary
	vector<string> resp;

	int ret = QueryCommand(cmd.str().c_str(), resp);
	if (ret != DEVICE_OK) 
	{
		return ret;
	}

	int numTries = 0, pollIntervalMs = 100;
	do
	{
		numTries++;
		CDeviceUtils::SleepMs(pollIntervalMs);
	}
	while (IsBusy(device) && (numTries*pollIntervalMs < timeoutMs));
	
	if (numTries*pollIntervalMs >= timeoutMs)
	{
		return ERR_BUSY_TIMEOUT;
	}

	ostringstream os;
	os << "Completed after " << (numTries*pollIntervalMs/1000.0) << " seconds.";
	core_->LogMessage(device_, os.str().c_str(), true);
	return DEVICE_OK;
}


/*
//Functions from UserDefinedSerialImpl.h for communication with a binary device. (Q: why were they in the .h file? Does it matter?)
//This function may be useful for translating escaped strings into bytes

template <template <class> class TBasicDevice, class UConcreteDevice>
int
UserDefSerialBase<TBasicDevice, UConcreteDevice>::
CreateByteStringProperty(const char* name, std::vector<char>& varRef,
      bool preInit)
{
   class Functor : public MM::ActionFunctor, boost::noncopyable
   {
      std::vector<char>& varRef_;
   public:
      Functor(std::vector<char>& varRef) : varRef_(varRef) {}
      virtual int Execute(MM::PropertyBase* pProp, MM::ActionType eAct)
      {
         if (eAct == MM::BeforeGet)
         {
            pProp->Set(EscapedStringFromByteString(varRef_).c_str());
         }
         else if (eAct == MM::AfterSet)
         {
            std::string s;
            pProp->Get(s);
            std::vector<char> bytes;
            int err = ByteStringFromEscapedString(s, bytes);
            if (err != DEVICE_OK)
               return err;
            varRef_ = bytes;
         }
         return DEVICE_OK;
      }
   };

   return Super::CreateStringProperty(name,
         EscapedStringFromByteString(varRef).c_str(), false,
         new Functor(varRef), preInit);
}

//These two functions should give us a template for rewriting QueryCommand above

template <template <class> class TBasicDevice, class UConcreteDevice>
int
UserDefSerialBase<TBasicDevice, UConcreteDevice>::
SendRecv(const std::vector<char>& command,
      const std::vector<char>& expectedResponse)
{
   if (command.empty())
      return DEVICE_OK;

   int err;

   err = Super::PurgeComPort(port_.c_str());
   if (err != DEVICE_OK)
      return err;

   err = Send(command);
   if (err != DEVICE_OK)
      return err;

   if (expectedResponse.empty())
      return DEVICE_OK;

   err = responseDetector_->RecvExpected(Super::GetCoreCallback(), this,
         port_, expectedResponse);
   if (err != DEVICE_OK)
      return err;

   return DEVICE_OK;
}

template <template <class> class TBasicDevice, class UConcreteDevice>
int
UserDefSerialBase<TBasicDevice, UConcreteDevice>::
SendQueryRecvAlternative(const std::vector<char>& command,
      const std::vector< std::vector<char> >& responseAlts,
      size_t& responseAltIndex)
{
   if (command.empty())
      return ERR_QUERY_COMMAND_EMPTY;

   int err;

   err = Super::PurgeComPort(port_.c_str());
   if (err != DEVICE_OK)
      return err;

   err = Send(command);
   if (err != DEVICE_OK)
      return err;

   err = responseDetector_->RecvAlternative(Super::GetCoreCallback(), this,
         port_, responseAlts, responseAltIndex);
   if (err != DEVICE_OK)
      return err;

   return DEVICE_OK;
}

//This is the basic 'Send' function; we can use the BinaryMode_ section to replace the ASCII Send function above

template <template <class> class TBasicDevice, class UConcreteDevice>
int
UserDefSerialBase<TBasicDevice, UConcreteDevice>::
Send(const std::vector<char>& command)
{
   if (command.empty())
      return DEVICE_OK;

   int err;

   if (binaryMode_)
   {
      err = Super::WriteToComPort(port_.c_str(),
            reinterpret_cast<const unsigned char*>(&command[0]),
            static_cast<unsigned int>(command.size()));
      if (err != DEVICE_OK)
         return err;
   }
   else
   {
      // Make sure there are no null bytes in the command
      std::vector<char>::const_iterator foundNull =
         std::find(command.begin(), command.end(), '\0');
      if (foundNull != command.end())
         return ERR_ASCII_COMMAND_CONTAINS_NULL;

      std::string commandString(command.begin(), command.end());
      err = Super::SendSerialCommand(port_.c_str(), commandString.c_str(),
            asciiTerminator_.c_str());
      if (err != DEVICE_OK)
         return err;
   }

   return DEVICE_OK;
}

//We may also need some stuff from ResponseDetector.cpp, but I am confused about how that works
*/