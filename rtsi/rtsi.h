#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cs_rtsi
{
	class RobotState;
}

namespace cs_rtsi
{
	namespace details
	{
		//方便的别名对于回调函数用RTSI：：recieveData();
		using cb_fun = std::function<void(std::shared_ptr<RobotState>, std::vector<char>&, uint32_t &)>;
		// convenience alias for the callback map of string ids and callback funtion objects
		using cb_map = std::unordered_map<std::string, details::cb_fun>;
	}

	class RTSI
	{
	public:
		explicit RTSI(const std::string hostname, int port = 30004, bool verbose = false);
		virtual ~RTSI();

		class RobotCommand
		{
		public:
			enum Type
			{
				NO_CMD = 0,
				MOVEJ = 1,    
				MOVEJ_IK = 2,
				MOVEL = 3,
				MOVEL_FK = 4,
				FORCE_MODE = 6,
				FORCE_MODE_STOP = 7,
				ZERO_FT_SENSOR = 8,
				SPEEDJ = 9,
				SPEEDL = 10,
				SERVOJ = 11,
				SERVOC = 12,    //
				SET_STD_DIGITAL_OUT = 13,
				SET_TOOL_DIGITAL_OUT = 14,
				SPEED_STOP = 15,
				SERVO_STOP = 16,
				SET_PAYLOAD = 17,
				TEACH_MODE = 18,
				END_TEACH_MODE = 19,
				FORCE_MODE_SET_DAMPING = 20,      //
				FORCE_MODE_SET_GAIN_SCALING = 21, //
				SET_SPEED_SLIDER = 22,
				SET_STD_ANALOG_OUT = 23,
				SERVOL = 24,
				TOOL_CONTACT = 25,
				GET_STEPTIME = 26,
				GET_ACTUAL_JOINT_POSITIONS_HISTORY = 27,  //
				GET_TARGET_WAYPOINT = 28,
				SET_TCP = 29,
				GET_INVERSE_KINEMATICS_ARGS = 30,
				PROTECTIVE_STOP = 31,  //
				STOPL = 33,
				STOPJ = 34,
				SET_WATCHDOG = 35,
				IS_POSE_WITHIN_SAFETY_LIMITS = 36,    //
				IS_JOINTS_WITHIN_SAFETY_LIMITS = 37,  //
				GET_JOINT_TORQUES = 38,
				POSE_TRANS = 39,
				GET_TCP_OFFSET = 40,
				JOG_START = 41,
				JOG_STOP = 42,
				GET_FORWARD_KINEMATICS_DEFAULT = 43, 
				GET_FORWARD_KINEMATICS_ARGS = 44, 
				MOVE_PATH = 45, 
				GET_INVERSE_KINEMATICS_DEFAULT = 46,
				IS_STEADY = 47, //
				SET_CONF_DIGITAL_OUT = 48,
				SET_INPUT_INT_REGISTER = 49,
				SET_INPUT_DOUBLE_REGISTER = 50,
				MOVE_UNTIL_CONTACT = 51,   //
				GET_SENSOR_FORCE = 60, 
				GET_TCP_FORCE =61,
				GET_TOOL_PAYLOAD =62,
				SET_INPUT_BIT_REGISTER_X_TO_Y = 81,
				SET_INPUT_BIT_REGISTER = 82,
				SET_EXTERNAL_FORCE_TORQUE = 96,
				WATCHDOG = 99,
				STOP_SCRIPT = 255

			};
			enum Recipe
			{
				RECIPE_1 = 1,
				RECIPE_2 = 2,
				RECIPE_3 = 3,
				RECIPE_4 = 4,
				RECIPE_5 = 5,
				RECIPE_6 = 6,
				RECIPE_7 = 7,
				RECIPE_8 = 8,
				RECIPE_9 = 9,
				RECIPE_10 = 10,
				RECIPE_11 = 11,
				RECIPE_12 = 12,
				RECIPE_13 = 13,
				RECIPE_14 = 14,
				RECIPE_15 = 15,
				RECIPE_16 = 16

			};

			RobotCommand() : m_type(NO_CMD), m_recipe_id(1)
			{

			}
			Type m_type = NO_CMD;
			std::uint8_t m_recipe_id;
			std::int32_t m_async;
			//输入整型寄存器。
			std::int32_t m_reg_int_val;
			//输入浮点寄存器。
			double m_reg_double_val;

			std::vector<double> m_val;
			std::vector<int> m_selection_vector;
			std::int32_t m_force_mode_type;
			//设置标准数字IO输出，每一个 bit 代表一个 IO 。需要设置
			//	standard_digital_output_mask 对应的bit
			std::uint16_t m_std_digital_out;
			//使能RTSI标准数字IO输出设置。只有当该
		    //值的某个bit设置为1时，才能通过
		   //standard_digital_output 设置对应的IO。
			std::uint16_t m_std_digital_out_mask;
			// 设置可配置数字IO输出，每一个 bit 代表一个IO。需要设置
			//configurable_digital_output_mask 对应的 bit
			std::uint16_t m_configurable_digital_out;
			//使能RTSI可配置数字IO输出设置。只有当该值的某个bit设置为1时，才能通过
			//configurable_digital_output 设置对应的IO。
			std::uint16_t m_configurable_digital_out_mask;

			std::uint16_t m_std_tool_out;
			std::uint16_t m_std_tool_out_mask;

			//使能RTSI标准模拟输出设置。0 - 1bits ：标准模拟输出0设置使能（bit 0），标准
			//模拟输出1设置使能（bit 1）。当该值大于3时设置无效。
			std::uint8_t m_std_analog_output_mask;
			//标准模拟输出模式，0 - 1bits：标准模拟输出0类型（bit 0），标准模拟输出1类
			//型（bit 1）。0: 电流模式；1: 电压模式。当该值大于3时设置无效。
			std::uint8_t m_std_analog_output_type;
			//*1主板标准模拟输出0等级，范围：[0-1]。超出范围时设置无效。
			double m_std_analog_output_0;
			// *1主板标准模拟输出1等级，范围：[0-1]。超出范围时设置无效。
			double m_std_analog_output_1;
			//速度调整滑块的使能与禁用。0为禁用，1
            //	为使能。其余值无效。
			std::uint32_t m_speed_slider_mask;
			//新的速度百分比，范围：[0.001 - 1]。其余值无效。
			double m_speed_slider_fraction;

			std::uint32_t m_steps;


		};

		enum RTSICommand
		{
			//校验协议版本
			RTSI_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
			//请求控制器的主版本、次版本、bug-fix版本、编译版本
			RTSI_GET_ELITECONTROL_VERSION = 118,         // ascii v
			//控制器和外部程序互相发送消息的报文，消息会显示在log窗口内。其中“源”代表的
			//是“消息源”。控制器的消息源是 EliteRTSI
			RTSI_TEXT_MESSAGE = 77,                   // ascii M
			//控制器\外部程序发送给对方的数据包，用于发送 \设置订阅的数据
			RTSI_DATA_PACKAGE = 85,                   // ascii U
			//控制器支持自定义订阅变量名，自定义输出频率
			RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
			//目前控制器支持自定义订阅变量名
			RTSI_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
			//当订阅结束后，发送开始信号以开始“同步循环”
			RTSI_CONTROL_PACKAGE_START = 83,          // ascii S
			//暂停“同步循环”
			RTSI_CONTROL_PACKAGE_PAUSE = 80           // ascii P
		};

		enum class ConnectionState : std::uint8_t
		{
			DISCONNECTED = 0,
			CONNECTED = 1,
			STARTED = 2,
			PAUSED = 3
		};

		static int sm_state;

	public:

		void connect();
		void disconnect();
		bool isConnected();
		bool isStarted();
		void setConState();
		int getConState();

		bool negotiateProtocolVersion();
		std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t> getControllerVersion();
		void receive();
		void receiveData(std::shared_ptr<RobotState> &robot_state);

		void send(const RobotCommand &robot_cmd);
		void sendAll(const std::uint8_t &command, std::string payload = "");
		void sendStart();
		void sendPause();
		bool sendOutputSetup(const std::vector<std::string> &output_names, double frequency);
		bool sendInputSetup(const std::vector<std::string> &input_names);


	private:
		
		//!< creates all callback functions on startup of the controller
		void setupCallbacks(); 

		//!< stores callback functions for handling the messages received by receiveData()
		details::cb_map m_cb_map;

		ConnectionState m_conn_state;

		std::string m_hostname;
		int m_port;
		bool m_verbose;

		std::vector<std::string> m_output_types;
		std::vector<std::string> m_output_names;

		std::shared_ptr<boost::asio::io_service> m_io_service;
		std::shared_ptr<boost::asio::ip::tcp::socket> m_socket;
		std::shared_ptr<boost::asio::ip::tcp::resolver> m_resolver;
		








	};





}