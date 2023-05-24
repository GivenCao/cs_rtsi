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
		@brief      : �õ�����״̬
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int getConState();

		/***************************
		@brief      : ĩ�˸���[kg].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getPayloadMass();

		/***************************
		@brief      : ĩ�˸�������[m].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getPayloadCog();

		/***************************
		@brief      : ���еĽű��к�.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getScriptControlLine();


		/***************************
		@brief      : �������������˿̵�ʱ�� [s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getTimestamp();

		/***************************
		@brief      : Ŀ��ؽ�λ��[rad].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetQ();

		/***************************
		@brief      : Ŀ��ؽ��ٶ�[rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetQd();


		/***************************
		@brief      : ʵ�ʵĹؽ�����[N*m].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualMoment();


		/***************************
		@brief      : ʵ�ʹؽ�λ��[rad].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualQ();


		/***************************
		@brief      : ʵ�ʹؽ��ٶ�[rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualQd();

		/***************************
		@brief      : ʵ�ʹؽڵ���[A].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualCurrent();

		/***************************
		@brief      : ʵ�ʵ�TCPλ��[x,y,z,rx,ry,rz][m,rad].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualTCPPose();

		/***************************
		@brief      : ʵ�ʵ�TCP�ٶ�[x,y,z,rx,ry,rz][m/s,rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getActualTCPSpeed();

		/***************************
		@brief      : TCP��Ŀ��λ��.
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetTCPPose();




		/***************************
		@brief      : Tcp��Ŀ���ٶ�[x,y,z,rx,ry,rz][m/s,rad/s].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getTargetTCPSpeed();



		/***************************
		@brief      : Tcp����������TCP����ϵ��.
		@time       : 2023/04/24
		****************************/
		RTSI_EXPORT std::vector<double> getActualTCPForce();


		/***************************
		@brief      : �������������ڻ�������ϵ��.
		@time       : 2023/04/2��
		****************************/
		RTSI_EXPORT std::vector<double> getActualSensorForce();

		/***************************
		@brief      :  ������������IO��λֵ���ӵ�λ����λ���δ���
		DDDDDDDDDDDDDDDDCCCCCCCCTTTTxxxx��DΪ������
		��io��CΪ����������io��TΪ��������io��x��Чλ��
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getActualDigitalInputBits();

		/***************************
		@brief      : �ؽ��¶�[���϶�].
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getJointTemperatures();


		/***************************
		@brief      : ������ģʽ.
		* -1 = ROBOT_MODE_NO_CONTROLLER
		* 0 = ROBOT_MODE_DISCONNECTED      δ����
		* 1 = ROBOT_MODE_CONFIRM_SAFETY    ȷ�ϰ�ȫ
		* 2	= ROBOT_MODE_BOOTING           ��ʼ��
		* 3 = ROBOT_MODE_POWER_OFF         �µ�
		* 4 = ROBOT_MODE_POWER_ON          �ϵ� 
		* 5	= ROBOT_MODE_IDLE              ����
		* 6	= ROBOT_MODE_BACKDRIVE         ��������
		* 7	= ROBOT_MODE_RUNNING           ��������
		* 8	= ROBOT_MODE_UPDATING_FIRMWARE      �����̼�
		* 9 = ROBOT_MODE_WAITING_CALIBRATION    �ȴ��������궨
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int32_t getRobotMode();


		/***************************
		@brief      : �����˹ؽ�ģʽ.
		����	235
		�ر��ŷ�	236
		������	238
		���µ�	239
		׼���µ�	240
		δ��Ӧ	245
		�����ʼ��	246
		������	247
		����	249
		Υ��	251
		����	252
		������	253
		����	255
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<int32_t> getJointMode();


		/***************************
		@brief      : ���尲ȫģʽ.
		SAFETY_MODE_NORMAL = 1	����ģʽ
		SAFETY_MODE_REDUCED = 2	����ģʽ
		SAFETY_MODE_PROTECTIVE_STOP = 3	����ֹͣ
		SAFETY_MODE_RECOVERY = 4	�ָ�ģʽ
		SAFETY_MODE_SAFEGUARD_STOP = 5	��ȫֹͣ
		SAFETY_MODE_SYSTEM_EMERGENCY_STOP = 6	ϵͳ��ͣ
		SAFETY_MODE_ROBOT_EMERGENCY_STOP = 7	�����˼�ͣ
		SAFETY_MODE_VIOLATION = 8	��ȫΥ��
		SAFETY_MODE_FAULT = 9	��ȫ����
		SAFETY_MODE_VALIDATE_JOINT_ID = 10	�ؽ�Υ��
		SAFETY_MODE_UNDEFINED_SAFETY_MODE = 11	δ֪��ȫģʽ
		SAFETY_MODE_AUTOMATIC_MODE_SAFEGUARD_STOP = 12	�Զ�ģʽ����ֹͣ״̬
		SAFETY_MODE_SYSTEM_THREE_POSITION_ENABLING_STOP = 13	��λ����δʹ��
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int32_t getSafetyMode();


		/***************************
		@brief      : ��ȫ״̬λ
		Bits 0-7: Is normal mode | ����ģʽ
		Is reduced mode | ����ģʽ
		Is protective stopped | ����ֹͣ
		Is recovery mode | �ָ�ģʽ
		Is safeguard stopped | ����ֹͣ
		Is system emergency stopped | ϵͳ��ͣ
		Is robot emergency stopped | �����˼�ͣ
		Is emergency stopped | ��ͣ
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int32_t getSafetyStatus();

		/***************************
		@brief      : �����˳���������ʵ�ٶȱ�������Χ: [0.02-1.0]
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getSpeedScaling();

		/***************************
		@brief      : �����˳�������Ŀ���ٶȱ������ƣ���Χ��[0.001-1.0]
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getTargetSpeedFraction();


		/***************************
		@brief      : �����˵�ѹ����λ��V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getActualRobotVoltage();

		/***************************
		@brief      : �����˵�������λ��A
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getActualRobotCurrent();

		/***************************
		@brief      : �����������IO��λֵ���ӵ�λ����λ���δ���
		DDDDDDDDDDDDDDDDCCCCCCCCTTTTxxxx.DΪ�������
		io��CΪ���������io��TΪ�������io��x��Чλ��
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getActualDigitalOutputBits();


		/***************************
		@brief      :  ����״̬��
		0��δ֪��
		1�����У�
		2����ͣ��
		3��ֹͣ
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getRuntimeState();


		/***************************
		@brief      : �������ⲿʵʱλ��
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::vector<double> getElbowPosition();

		/***************************
		@brief      : ������״̬
		Bits 0-3: ��Դ�Ƿ��(bit 0) ,
		�����Ƿ�����(bit 1) , 
		����������ť�Ƿ���(bit 2)
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT uint32_t getRobotStatusBits();

		/***************************
		@brief      : ��ȫ״̬
		Bits 0-3: ��Դ�Ƿ��(bit 0) ,
		�����Ƿ�����(bit 1) ,
		����������ť�Ƿ���(bit 2)
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT std::uint32_t getSafetyStatusBits();

		/***************************
		@brief      : �����׼ģ������IO�е�ģ������ֵ0����λ��A��V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogInput0();

		/***************************
		@brief      : �����׼ģ������IO�е�ģ������ֵ1����λ��A��V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogInput1();

		/***************************
		@brief      : �����׼ģ�����IO�е�ģ������ֵ0����λ��A��V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogOutput0();

		/***************************
		@brief      : �����׼ģ�����IO�е�ģ������ֵ1����λ��A��V
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getStandardAnalogOutput1();

		/***************************
		@brief      : �ڵͷ�Χ[18-22]��߷�Χ[42-46]�л�ȡָ������������Ĵ���
	    [0-23]��������Ĵ����Ľϵͷ�Χ�Ǳ������ֳ�����/PLC�ӿ�ʹ�������
		[24-47]���μĴ������Ϸ�Χ�������ⲿRTSI�ͻ���
		@param[in]  :  output_idҪ��ȡ�ļĴ�����id��Ŀǰ֧�ֵķ�Χ��:[18-22]��[42-46]��
		����Կ���ͨ���ı�rtsiReceiveInterface����䷽
		��ʹ��use_upper_range_registers���������캯����־��������֮���л���
		@return     : ��ָ��������Ĵ�������һ������
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT int getOutputIntRegister(int output_id);

		/***************************
		@brief      : �ڵͷ�Χ[18-22]��߷�Χ[42-46]�л�ȡָ�������double�Ĵ���
		[0-23]����������Ĵ����Ľϵͷ�Χ�Ǳ������ֳ�����/PLC�ӿ�ʹ�������
		[24-47]�������Ĵ������Ϸ�Χ�������ⲿRTSI�ͻ���
		@param[in]  : output_idҪ��ȡ�ļĴ�����id��Ŀǰ֧�ֵķ�Χ��:[18-22]��[42-46]��
		����Կ���ͨ���ı�rtsiReceiveInterface����䷽
		��ʹ��use_upper_range_registers���������캯����־��������֮���л���
		@return     : ��ָ��������Ĵ�������һ��������
		@time       : 2023/03/28
		****************************/
		RTSI_EXPORT double getOutputDoubleRegister(int output_id);


		void receiveCallback();

		private:
			//�������÷���
			bool setupRecipes(const double& frequency);

           //��ʼ������Ĵ����Ĺ���ӳ��
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
				//�Ĵ���ƫ��
				int m_register_offset;
				//Ƶ��
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