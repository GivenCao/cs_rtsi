#pragma once

#include "rtsi_export.h"

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>


#define MAJOR_VERSION 2
#define CB3_MAJOR_VERSION 3

namespace boost
{
	class thread;
}
namespace cs_rtsi
{
	class DashboardClient;
}
namespace cs_rtsi
{
	class RobotState;
}
namespace cs_rtsi
{
	class RTSI;
}
namespace cs_rtsi
{
	class RTSIReceiveInterface
	{
	public:
		RTSI_EXPORT explicit RTSIReceiveInterface(std::string hostname, std::vector<std::string>variables = {},
			bool verbose = false, bool use_upper_range_registers = false);
		RTSI_EXPORT virtual ~RTSIReceiveInterface();

		enum SafetyStatus
		{
			IS_NORMAL_MODE = 0,
			IS_REDUCED_MODE = 1,
			IS_PROTECTIVE_STOPPED = 2,
			IS_RECOVERY_MODE = 3,
			IS_SAFEGUARD_STOPPED = 4,
			IS_SYSTEM_EMERGENCY_STOPPED = 5,
			IS_ROBOT_EMERGENCY_STOPPED = 6,
			IS_EMERGENCY_STOPPED = 7,
			IS_VIOLATION = 8,
			IS_FAULT = 9,
			IS_STOPPED_DUE_TO_SAFETY = 10
		};

		enum OutputIntRegisters
		{
			OUTPUT_INT_REGISTER_0 = 0,
			OUTPUT_INT_REGISTER_1,
			OUTPUT_INT_REGISTER_2,
			OUTPUT_INT_REGISTER_3,
			OUTPUT_INT_REGISTER_4,
			OUTPUT_INT_REGISTER_5,
			OUTPUT_INT_REGISTER_6,
			OUTPUT_INT_REGISTER_7,
			OUTPUT_INT_REGISTER_8,
			OUTPUT_INT_REGISTER_9,
			OUTPUT_INT_REGISTER_10,
			OUTPUT_INT_REGISTER_11,
			OUTPUT_INT_REGISTER_12,
			OUTPUT_INT_REGISTER_13,
			OUTPUT_INT_REGISTER_14,
			OUTPUT_INT_REGISTER_15,
			OUTPUT_INT_REGISTER_16,
			OUTPUT_INT_REGISTER_17,
			OUTPUT_INT_REGISTER_18,
			OUTPUT_INT_REGISTER_19,
			OUTPUT_INT_REGISTER_20,
			OUTPUT_INT_REGISTER_21,
			OUTPUT_INT_REGISTER_22,
			OUTPUT_INT_REGISTER_23
		};


		enum OutputDoubleRegisters
		{
			OUTPUT_DOUBLE_REGISTER_0 = 0,
			OUTPUT_DOUBLE_REGISTER_1,
			OUTPUT_DOUBLE_REGISTER_2,
			OUTPUT_DOUBLE_REGISTER_3,
			OUTPUT_DOUBLE_REGISTER_4,
			OUTPUT_DOUBLE_REGISTER_5,
			OUTPUT_DOUBLE_REGISTER_6,
			OUTPUT_DOUBLE_REGISTER_7,
			OUTPUT_DOUBLE_REGISTER_8,
			OUTPUT_DOUBLE_REGISTER_9,
			OUTPUT_DOUBLE_REGISTER_10,
			OUTPUT_DOUBLE_REGISTER_11,
			OUTPUT_DOUBLE_REGISTER_12,
			OUTPUT_DOUBLE_REGISTER_13,
			OUTPUT_DOUBLE_REGISTER_14,
			OUTPUT_DOUBLE_REGISTER_15,
			OUTPUT_DOUBLE_REGISTER_16,
			OUTPUT_DOUBLE_REGISTER_17,
			OUTPUT_DOUBLE_REGISTER_18,
			OUTPUT_DOUBLE_REGISTER_19,
			OUTPUT_DOUBLE_REGISTER_20,
			OUTPUT_DOUBLE_REGISTER_21,
			OUTPUT_DOUBLE_REGISTER_22,
			OUTPUT_DOUBLE_REGISTER_23
		};


		enum RuntimeState
		{
			UNKNOWN = 0,
			PLAYING = 1,
			PAUSING = 2,
			STOPPED = 3,
		};


		enum class PausingState
		{
			PAUSED,
			RUNNING,
			RAMPUP
		};

		/***************************
		@brief      : Can be used to disconnect from the robot. 
		To reconnect you have to call the reconnect() function.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT void disconnect();

		/***************************
		@brief      : Can be used to reconnect to the robot after a lost connection.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT bool reconnect();

		/***************************
		@brief      : Connection status for RTSI, useful for checking for lost connection.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT bool isConnected();

		/***************************
		@brief      : 得到连接状态
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int getConState();

		/***************************
		@brief      : 末端负载[kg].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getPayloadMass();

		/***************************
		@brief      : 末端负载质心[m].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getPayloadCog();

		/***************************
		@brief      : 运行的脚本行号.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getScriptControlLine();


		/***************************
		@brief      : 机器人启动至此刻的时间 [s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getTimestamp();

		/***************************
		@brief      : 目标关节位置[rad].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetQ();

		/***************************
		@brief      : 目标关节速度[rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetQd();


		/***************************
		@brief      : 实际的关节力矩[N*m].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualMoment();


		/***************************
		@brief      : 实际关节位置[rad].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualQ();


		/***************************
		@brief      : 实际关节速度[rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualQd();

		/***************************
		@brief      : 实际关节电流[A].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualCurrent();

		/***************************
		@brief      : 实际的TCP位姿[x,y,z,rx,ry,rz][m,rad].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualTCPPose();

		/***************************
		@brief      : 实际的TCP速度[x,y,z,rx,ry,rz][m/s,rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualTCPSpeed();

		/***************************
		@brief      : TCP的目标位姿.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetTCPPose();




		/***************************
		@brief      : Tcp的目标速度[x,y,z,rx,ry,rz][m/s,rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetTCPSpeed();



		/***************************
		@brief      : Tcp端力（基于TCP坐标系）.
		@time       : 2023/04/24
		****************************/
		RTSI_EXPORT std::vector<double> getActualTCPForce();


		/***************************
		@brief      : 传感器力（基于基座坐标系）.
		@time       : 2023/04/2５
		****************************/
		RTSI_EXPORT std::vector<double> getActualSensorForce();

		/***************************
		@brief      :  所有数字输入IO的位值。从低位到高位依次代表
		DDDDDDDDDDDDDDDDCCCCCCCCTTTTxxxx。D为数字输
		入io，C为可配置输入io，T为工具输入io，x无效位。
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getActualDigitalInputBits();

		/***************************
		@brief      : 关节温度[摄氏度].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getJointTemperatures();


		/***************************
		@brief      : 机器人模式.
		* -1 = ROBOT_MODE_NO_CONTROLLER
		* 0 = ROBOT_MODE_DISCONNECTED      未连接
		* 1 = ROBOT_MODE_CONFIRM_SAFETY    确认安全
		* 2	= ROBOT_MODE_BOOTING           初始化
		* 3 = ROBOT_MODE_POWER_OFF         下电
		* 4 = ROBOT_MODE_POWER_ON          上电 
		* 5	= ROBOT_MODE_IDLE              空闲
		* 6	= ROBOT_MODE_BACKDRIVE         反向驱动
		* 7	= ROBOT_MODE_RUNNING           正在运行
		* 8	= ROBOT_MODE_UPDATING_FIRMWARE      升级固件
		* 9 = ROBOT_MODE_WAITING_CALIBRATION    等待编码器标定
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int32_t getRobotMode();


		/***************************
		@brief      : 机器人关节模式.
		重置	235
		关闭伺服	236
		反驱动	238
		已下电	239
		准备下电	240
		未响应	245
		电机初始化	246
		启动中	247
		启动	249
		违规	251
		错误	252
		运行中	253
		空闲	255
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<int32_t> getJointMode();


		/***************************
		@brief      : 主板安全模式.
		SAFETY_MODE_NORMAL = 1	正常模式
		SAFETY_MODE_REDUCED = 2	缩减模式
		SAFETY_MODE_PROTECTIVE_STOP = 3	保护停止
		SAFETY_MODE_RECOVERY = 4	恢复模式
		SAFETY_MODE_SAFEGUARD_STOP = 5	安全停止
		SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6	系统急停
		SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7	机器人急停
		SAFETY_MODE_VIOLATION = 8	安全违规
		SAFETY_MODE_FAULT = 9	安全错误
		SAFETY_MODE_VALIDATE_JOINT_ID = 10	关节违规
		SAFETY_MODE_UNDEFINED_SAFETY_MODE = 11	未知安全模式
		SAFETY_MODE_AUTOMATIC_MODE_SAFEGUARD_STOP = 12	自动模式防护停止状态
		SAFETY_MODE_SYSTEM_THREE_POSITION_ENABLING_STOP = 13	三位开关未使能
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int32_t getSafetyMode();


		/***************************
		@brief      : 安全状态位
		Bits 0-7: Is normal mode | 正常模式
		Is reduced mode | 缩减模式
		Is protective stopped | 保护停止
		Is recovery mode | 恢复模式
		Is safeguard stopped | 防护停止
		Is system emergency stopped | 系统急停
		Is robot emergency stopped | 机器人急停
		Is emergency stopped | 急停
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int32_t getSafetyStatus();

		/***************************
		@brief      : 机器人程序运行真实速度比例，范围: [0.02-1.0]
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getSpeedScaling();

		/***************************
		@brief      : 机器人程序运行目标速度比例限制，范围：[0.001-1.0]
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getTargetSpeedFraction();


		/***************************
		@brief      : 机器人电压，单位：V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getActualRobotVoltage();

		/***************************
		@brief      : 机器人电流，单位：A
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getActualRobotCurrent();

		/***************************
		@brief      : 所有数字输出IO的位值。从低位到高位依次代表
		DDDDDDDDDDDDDDDDCCCCCCCCTTTTxxxx.D为数字输出
		io，C为可配置输出io，T为工具输出io，x无效位。
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getActualDigitalOutputBits();


		/***************************
		@brief      :  程序状态。
		0：未知；
		1：运行；
		2：暂停；
		3：停止
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getRuntimeState();


		/***************************
		@brief      : 机器人肘部实时位置
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getElbowPosition();

		/***************************
		@brief      : 机器人状态
		Bits 0-3: 电源是否打开(bit 0) ,
		程序是否运行(bit 1) , 
		自由驱动按钮是否按下(bit 2)
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getRobotStatusBits();

		/***************************
		@brief      : 安全状态
		Bits 0-3: 电源是否打开(bit 0) ,
		程序是否运行(bit 1) ,
		自由驱动按钮是否按下(bit 2)
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::uint32_t getSafetyStatusBits();

		/***************************
		@brief      : 主板标准模拟输入IO中的模拟量数值0，单位：A或V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogInput0();

		/***************************
		@brief      : 主板标准模拟输入IO中的模拟量数值1，单位：A或V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogInput1();

		/***************************
		@brief      : 主板标准模拟输出IO中的模拟量数值0，单位：A或V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogOutput0();

		/***************************
		@brief      : 主板标准模拟输出IO中的模拟量数值1，单位：A或V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogOutput1();

		/***************************
		@brief      : 在低范围[18-22]或高范围[42-46]中获取指定的输出整数寄存器
	    [0-23]整型输出寄存器的较低范围是保留的现场总线/PLC接口使用情况。
		[24-47]整形寄存器的上范围可用于外部RTSI客户端
		@param[in]  :  output_id要读取的寄存器的id，目前支持的范围是:[18-22]或[42-46]，
		这可以可以通过改变rtsiReceiveInterface输出配方
		和使用use_upper_range_registers来调整构造函数标志在上下限之间切换。
		@return     : 从指定的输出寄存器返回一个整数
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int getOutputIntRegister(int output_id);

		/***************************
		@brief      : 在低范围[18-22]或高范围[42-46]中获取指定的输出double寄存器
		[0-23]浮点数输出寄存器的较低范围是保留的现场总线/PLC接口使用情况。
		[24-47]浮点数寄存器的上范围可用于外部RTSI客户端
		@param[in]  : output_id要读取的寄存器的id，目前支持的范围是:[18-22]或[42-46]，
		这可以可以通过改变rtsiReceiveInterface输出配方
		和使用use_upper_range_registers来调整构造函数标志在上下限之间切换。
		@return     : 从指定的输出寄存器返回一个浮点数
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getOutputDoubleRegister(int output_id);


		void receiveCallback();

		private:
			//设置配置方法
			bool setupRecipes(const double& frequency);

           //初始化输出寄存器的功能映射
			void initOutputRegFuncMap();

			std::string outDoubleReg(int reg) const
			{
				return "output_double_register" + std::to_string(m_register_offset + reg);
			};

			std::string outIntReg(int reg) const
			{
				return "output_int_register" + std::to_string(m_register_offset + reg);
			};

			double getOutputDoubleReg(int reg)
			{
				std::string func_name = "getOutput_double_register" + std::to_string(reg);
				return m_output_reg_func_map[func_name]();
			};

			int getOutputIntReg(int reg)
			{
				std::string func_name = "getOutput_int_register" + std::to_string(reg);
				return m_output_reg_func_map[func_name]();
			};

			template <typename T>
			bool isWithinBounds(const T& value, const T& low, const T& high)
			{
				return (low <= value && value <= high);
			}


			private:
				std::vector<std::string> m_variables;
				std::string m_hostname;
				int m_port;
				bool m_verbose;
				bool m_use_upper_range_registers;
				//寄存器偏置
				int m_register_offset;
				//频率
				double m_frequency;
				double m_delta_time;
				std::shared_ptr<RTSI> mp_rtsi;
				std::atomic<bool> m_stop_thread{ false };
				std::shared_ptr<boost::thread> mp_th;
				std::shared_ptr<RobotState> mp_robot_state;
				std::map <std::string, std::function<double()>> m_output_reg_func_map;

				PausingState m_pausing_state;
				double m_speed_scaling_combined;
				double m_pausing_ramp_up_increment;

	};
}