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
		//����ı������ڻص�������RTSI����recieveData();
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
			//�������ͼĴ�����
			std::int32_t m_reg_int_val;
			//���븡��Ĵ�����
			double m_reg_double_val;

			std::vector<double> m_val;
			std::vector<int> m_selection_vector;
			std::int32_t m_force_mode_type;
			//���ñ�׼����IO�����ÿһ�� bit ����һ�� IO ����Ҫ����
			//	standard_digital_output_mask ��Ӧ��bit
			std::uint16_t m_std_digital_out;
			//ʹ��RTSI��׼����IO������á�ֻ�е���
		    //ֵ��ĳ��bit����Ϊ1ʱ������ͨ��
		   //standard_digital_output ���ö�Ӧ��IO��
			std::uint16_t m_std_digital_out_mask;
			// ���ÿ���������IO�����ÿһ�� bit ����һ��IO����Ҫ����
			//configurable_digital_output_mask ��Ӧ�� bit
			std::uint16_t m_configurable_digital_out;
			//ʹ��RTSI����������IO������á�ֻ�е���ֵ��ĳ��bit����Ϊ1ʱ������ͨ��
			//configurable_digital_output ���ö�Ӧ��IO��
			std::uint16_t m_configurable_digital_out_mask;

			std::uint16_t m_std_tool_out;
			std::uint16_t m_std_tool_out_mask;

			//ʹ��RTSI��׼ģ��������á�0 - 1bits ����׼ģ�����0����ʹ�ܣ�bit 0������׼
			//ģ�����1����ʹ�ܣ�bit 1��������ֵ����3ʱ������Ч��
			std::uint8_t m_std_analog_output_mask;
			//��׼ģ�����ģʽ��0 - 1bits����׼ģ�����0���ͣ�bit 0������׼ģ�����1��
			//�ͣ�bit 1����0: ����ģʽ��1: ��ѹģʽ������ֵ����3ʱ������Ч��
			std::uint8_t m_std_analog_output_type;
			//*1�����׼ģ�����0�ȼ�����Χ��[0-1]��������Χʱ������Ч��
			double m_std_analog_output_0;
			// *1�����׼ģ�����1�ȼ�����Χ��[0-1]��������Χʱ������Ч��
			double m_std_analog_output_1;
			//�ٶȵ��������ʹ������á�0Ϊ���ã�1
            //	Ϊʹ�ܡ�����ֵ��Ч��
			std::uint32_t m_speed_slider_mask;
			//�µ��ٶȰٷֱȣ���Χ��[0.001 - 1]������ֵ��Ч��
			double m_speed_slider_fraction;

			std::uint32_t m_steps;


		};

		enum RTSICommand
		{
			//У��Э��汾
			RTSI_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
			//��������������汾���ΰ汾��bug-fix�汾������汾
			RTSI_GET_ELITECONTROL_VERSION = 118,         // ascii v
			//���������ⲿ�����෢����Ϣ�ı��ģ���Ϣ����ʾ��log�����ڡ����С�Դ�������
			//�ǡ���ϢԴ��������������ϢԴ�� EliteRTSI
			RTSI_TEXT_MESSAGE = 77,                   // ascii M
			//������\�ⲿ�����͸��Է������ݰ������ڷ��� \���ö��ĵ�����
			RTSI_DATA_PACKAGE = 85,                   // ascii U
			//������֧���Զ��嶩�ı��������Զ������Ƶ��
			RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
			//Ŀǰ������֧���Զ��嶩�ı�����
			RTSI_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
			//�����Ľ����󣬷��Ϳ�ʼ�ź��Կ�ʼ��ͬ��ѭ����
			RTSI_CONTROL_PACKAGE_START = 83,          // ascii S
			//��ͣ��ͬ��ѭ����
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