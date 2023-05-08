#include "robot_state.h"
#include "rtsi.h"
#include "rtc_utility.h"

//#include "ThirdParty\SpdLog\GLog.h"


#include <boost/asio/connect.hpp>
#include <boost/asio/detail/socket_option.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <tuple>
#include <type_traits>


const unsigned HEADER_SIZE = 3;
#define RTSI_PROTOCOL_VERSION 1;
#define DEBUG_OUTPUT false

#if DEBUG_OUTPUT
#define DEBUG(a)                                                \
  {                                                             \
    std::cout << "RTSI:" << __LINE__ << ": " << a << std::endl; \
  }
#else
#define DEBUG(a) \
  {              \
  }
#endif

using boost::asio::ip::tcp;
using namespace std::chrono;

namespace cs_rtsi
{
	int RTSI::sm_state = 0;

	RTSI::RTSI(const std::string hostname, int port, bool verbose)
		: m_hostname(std::move(hostname)),
		m_port(port),
		m_verbose(verbose),
		m_conn_state(ConnectionState::DISCONNECTED)
	{
		//_m_state = 0;
		setConState();
		setupCallbacks();
	}

	RTSI::~RTSI() = default;

	void RTSI::connect()
	{
		try
		{
			//实现TCP套接字连接的过程
			//<1>io_service_: 是Boost.Asio的核心组件，用于调度异步操作和处理IO事件，例如网络通信。
			//由std::make_shared方法创建了一个shared_ptr类型的智能指针
			m_io_service = std::make_shared<boost::asio::io_service>();
			//<2>socket_: 是一个TCP套接字对象，用于建立网络连接。它由boost::asio::ip::tcp::socket类创建，
			//并将其指针存储在socket_成员变量中，使用reset方法初始化对象。
			//该类实现了socket_base接口，提供了许多socket操作，如connect, read, write等等。
			m_socket.reset(new boost::asio::ip::tcp::socket(*m_io_service));
			//调用open()函数打开一个 IPv4 套接字。
			m_socket->open(boost::asio::ip::tcp::v4());
			//该类用于设置套接字选项，能够设置TCP的nodelay属性，从而达成更佳的网络性能。
			boost::asio::ip::tcp::no_delay no_delay_option(true);
			//同样是用于设置套接字选项，这里是用于开启TCP的SO_REUSEADDR选项，表示在socket连接前，
			//可以重复地使用相同的端口号，并且这样不会出错。
			boost::asio::socket_base::reuse_address sol_reuse_option(true);
			//设置TCP套接字的选项
			m_socket->set_option(no_delay_option);
			m_socket->set_option(sol_reuse_option);
			//创建了一个名为resolver_的新对象，并将其与之前创建的io_service_对象关联。
			m_resolver = std::make_shared<boost::asio::ip::tcp::resolver>(*m_io_service);
			//创建了一个名为query的新对象，该对象表示要连接的远程主机和端口号。
			//用于在resolver上查询主机名(hostname_)和端口号(port_)
			boost::asio::ip::tcp::resolver::query query(m_hostname, std::to_string(m_port));
			//使用resolver_对象和query对象来解析和连接远程主机。
			//通过connect函数尝试对一个地址进行socket连接，此处将socket_与resolver_一起使用，
			//解析query获取host_address之后再进行connect操作
			boost::asio::connect(*m_socket, m_resolver->resolve(query));

			m_conn_state = ConnectionState::CONNECTED;
			setConState();

			if (m_verbose)
			{
				std::cout << "Connected successfully to: " << m_hostname << " at " << m_port << std::endl;
			}

		}
		catch (const boost::system::system_error &)
		{
			std::string error_msg =
				"Error: Could not connect to: " + m_hostname + " at " + std::to_string(m_port) + ", verify the IP";
			throw std::runtime_error(error_msg);
		}
	}



	void RTSI::disconnect()
	{
		/* We use reset() to safely close the socket,
		 * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
		 */
		m_socket.reset();
		m_conn_state = ConnectionState::DISCONNECTED;
		setConState();
		if (m_verbose)
			std::cout << "RTSI - Socket disconnected" << std::endl;
	}

	bool RTSI::isConnected()
	{
		return m_conn_state == ConnectionState::CONNECTED || m_conn_state == ConnectionState::STARTED;
	}

	bool RTSI::isStarted()
	{
		return m_conn_state == ConnectionState::STARTED;
	}

	void RTSI::setConState()
	{
		// if ((int)conn_state_)
		//{
		//  std::cout << (int)conn_state_ << std::endl;
		//}
		switch (m_conn_state)
		{
			// std::cout << int(conn_state_) << std::endl;
		case ConnectionState::DISCONNECTED:
			RTSI::sm_state = 0;
			break;
		case ConnectionState::CONNECTED:
			RTSI::sm_state = 1;
			break;
		case ConnectionState::STARTED:
			RTSI::sm_state = 1;
			break;
		case ConnectionState::PAUSED:
			RTSI::sm_state = 1;
			break;
		default:
			break;
		}


	}


	int RTSI::getConState()
	{
		return RTSI::sm_state;
	}

	bool RTSI::negotiateProtocolVersion()
	{
		std::uint8_t cmd = RTSI_REQUEST_PROTOCOL_VERSION;
		// Pack RTDE_PROTOCOL_VERSION into payload
		uint8_t null_byte = 0;
		uint8_t version = RTSI_PROTOCOL_VERSION;
		std::vector<char>buffer;
		buffer.push_back(null_byte);
		buffer.push_back(version);
		std::string payload(buffer.begin(), buffer.end());
		sendAll(cmd, payload);
		DEBUG("Done sending RTSI_REQUEST_PROTOCOL_VERSION");
		receive();
		return true;
	}

	bool RTSI::sendInputSetup(const std::vector<std::string> &input_names)
	{
		std::uint8_t cmd = RTSI_CONTROL_PACKAGE_SETUP_INPUTS;
		// Concatenate input_names to a single string
		std::string input_names_str;
		for (const auto &input_name : input_names)
			input_names_str += input_name + ",";
		sendAll(cmd, input_names_str);
		DEBUG("Done sending RTSI_CONTROL_PACKAGE_SETUP_INPUTS");
		receive();
		return true;
	}

	bool RTSI::sendOutputSetup(const std::vector<std::string> &output_names, double frequency)
	{
		std::uint8_t cmd = RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS;
		// First save the output_names for use in the receiveData function
		m_output_names = output_names;
		std::string freq_as_hexstr = RTCUtility::double2hexstr(frequency);
		std::vector<char> freq_packed = RTCUtility::hexToBytes(freq_as_hexstr);
		// Concatenate output_names to a single string
		std::string output_names_str;
		for (const auto &output_name : output_names)
			output_names_str += output_name + ",";

		std::copy(output_names_str.begin(), output_names_str.end(), std::back_inserter(freq_packed));
		std::string payload(std::begin(freq_packed), std::end(freq_packed));
		sendAll(cmd, payload);
		DEBUG("Done sending RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS");
		receive();
		return true;

	}

	void RTSI::send(const RobotCommand &robot_cmd)
	{
		std::uint8_t command = RTSI_DATA_PACKAGE;
		std::vector<char> cmd_packed;
		cmd_packed = RTCUtility::packInt32(robot_cmd.m_type);
		if (robot_cmd.m_type == RobotCommand::SET_INPUT_INT_REGISTER)
		{
			std::vector<char> reg_int_packed = RTCUtility::packInt32(robot_cmd.m_reg_int_val);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_int_packed.begin()),
				std::make_move_iterator(reg_int_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::SET_INPUT_DOUBLE_REGISTER)
		{
			std::vector<char> reg_double_packed = RTCUtility::packDouble(robot_cmd.m_reg_double_val);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(reg_double_packed.begin()),
				std::make_move_iterator(reg_double_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::WATCHDOG)
		{
			cmd_packed = RTCUtility::packInt32(RobotCommand::NO_CMD);
		}

		if (robot_cmd.m_type == RobotCommand::FORCE_MODE)
		{
			std::vector<char> force_mode_type_packed = RTCUtility::packInt32(robot_cmd.m_force_mode_type);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(force_mode_type_packed.begin()),
				std::make_move_iterator(force_mode_type_packed.end()));

			std::vector<char> sel_vector_packed = RTCUtility::packVectorNInt32(robot_cmd.m_selection_vector);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(sel_vector_packed.begin()),
				std::make_move_iterator(sel_vector_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::GET_ACTUAL_JOINT_POSITIONS_HISTORY)
		{
			std::vector<char> actual_joint_positions_history_packed = RTCUtility::packUInt32(robot_cmd.m_steps);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(actual_joint_positions_history_packed.begin()),
				std::make_move_iterator(actual_joint_positions_history_packed.end()));
		}

		if (!robot_cmd.m_val.empty())
		{
			std::vector<char> vector_nd_packed = RTCUtility::packVectorNd(robot_cmd.m_val);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(vector_nd_packed.begin()),
				std::make_move_iterator(vector_nd_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::MOVEJ || robot_cmd.m_type == RobotCommand::MOVEJ_IK ||
			robot_cmd.m_type == RobotCommand::MOVEL || robot_cmd.m_type == RobotCommand::MOVEL_FK ||
			robot_cmd.m_type == RobotCommand::MOVE_PATH)
		{
			std::vector<char> async_packed = RTCUtility::packInt32(robot_cmd.m_async);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(async_packed.begin()),
				std::make_move_iterator(async_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::SET_STD_DIGITAL_OUT)
		{

			std::vector<char> std_digital_out_mask_packed = 
				RTCUtility::packInt32(robot_cmd.m_std_digital_out_mask);
			cmd_packed.insert(cmd_packed.end(), 
				std::make_move_iterator(std_digital_out_mask_packed.begin()),
				std::make_move_iterator(std_digital_out_mask_packed.end()));


			std::vector<char> std_digital_out_packed =
				RTCUtility::packInt32(robot_cmd.m_std_digital_out);
			cmd_packed.insert(cmd_packed.end(),
				std::make_move_iterator(std_digital_out_packed.begin()),
				std::make_move_iterator(std_digital_out_packed.end()));

		}

		if (robot_cmd.m_type == RobotCommand::SET_CONF_DIGITAL_OUT)
		{

			std::vector<char> configurable_digital_out_mask_packed =
				RTCUtility::packInt32(robot_cmd.m_configurable_digital_out_mask);
			cmd_packed.insert(cmd_packed.end(),
				std::make_move_iterator(configurable_digital_out_mask_packed.begin()),
				std::make_move_iterator(configurable_digital_out_mask_packed.end()));


			std::vector<char> configurable_digital_out_packed =
				RTCUtility::packInt32(robot_cmd.m_configurable_digital_out);
			cmd_packed.insert(cmd_packed.end(),
				std::make_move_iterator(configurable_digital_out_packed.begin()),
				std::make_move_iterator(configurable_digital_out_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::SET_TOOL_DIGITAL_OUT)
		{
			std::vector<char> std_tool_out_mask_packed =
				RTCUtility::packInt32(robot_cmd.m_std_tool_out_mask);
			cmd_packed.insert(cmd_packed.end(),
				std::make_move_iterator(std_tool_out_mask_packed.begin()),
				std::make_move_iterator(std_tool_out_mask_packed.end()));


			std::vector<char> std_tool_out_packed =
				RTCUtility::packInt32(robot_cmd.m_std_tool_out);
			cmd_packed.insert(cmd_packed.end(),
				std::make_move_iterator(std_tool_out_packed.begin()),
				std::make_move_iterator(std_tool_out_packed.end()));
		}
		if (robot_cmd.m_type == RobotCommand::SET_SPEED_SLIDER)
		{
			std::vector<char> speed_slider_mask_packed = RTCUtility::packUInt32(robot_cmd.m_speed_slider_mask);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_mask_packed.begin()),
				std::make_move_iterator(speed_slider_mask_packed.end()));

			std::vector<char> speed_slider_fraction_packed = RTCUtility::packDouble(robot_cmd.m_speed_slider_fraction);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(speed_slider_fraction_packed.begin()),
				std::make_move_iterator(speed_slider_fraction_packed.end()));
		}

		if (robot_cmd.m_type == RobotCommand::SET_STD_ANALOG_OUT)
		{
			cmd_packed.push_back(robot_cmd.m_std_analog_output_mask);
			cmd_packed.push_back(robot_cmd.m_std_analog_output_type);
			std::vector<char> std_analog_output_0_packed = RTCUtility::packDouble(robot_cmd.m_std_analog_output_0);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_0_packed.begin()),
				std::make_move_iterator(std_analog_output_0_packed.end()));
			std::vector<char> std_analog_output_1_packed = RTCUtility::packDouble(robot_cmd.m_std_analog_output_1);
			cmd_packed.insert(cmd_packed.end(), std::make_move_iterator(std_analog_output_1_packed.begin()),
				std::make_move_iterator(std_analog_output_1_packed.end()));
		}

		cmd_packed.insert(cmd_packed.begin(), robot_cmd.m_recipe_id);
		std::string sent(cmd_packed.begin(), cmd_packed.end());

		sendAll(command, sent);
		DEBUG("Done sending RTSI_DATA_PACKAGE");

	}

	void RTSI::sendAll(const std::uint8_t &command, std::string payload)
	{
		DEBUG("Payload size is : "<< payload.size());
		// Pack size and command into header
		uint16_t size = htons(HEADER_SIZE + (uint16_t)payload.size());
		uint8_t type = command;

		char buffer[3];
		memcpy(buffer + 0, &size, sizeof(size));
		memcpy(buffer + 2, &type, sizeof(type));

		// Create vector<char> that includes the header
		std::vector<char> header_packed;
		std::copy(buffer, buffer + sizeof(buffer), std::back_inserter(header_packed));

		// Add the payload to the header_packed vector
		std::copy(payload.begin(), payload.end(), std::back_inserter(header_packed));

		std::string sent(header_packed.begin(), header_packed.end());

		DEBUG("SENDING buf containing: " << sent << " with len: " << sent.size());

		boost::asio::write(*m_socket, boost::asio::buffer(header_packed, header_packed.size()));
	}

	void RTSI::sendStart()
	{

		std::uint8_t cmd = RTSI_CONTROL_PACKAGE_START;
		sendAll(cmd, "");
		DEBUG("Done sending RTSI_CONTROL_PACKAGE_START");
		receive();
	}

	void RTSI::sendPause()
	{
		std::uint8_t cmd = RTSI_CONTROL_PACKAGE_PAUSE;
		sendAll(cmd, "");
		DEBUG("Done sending RTSI_CONTROL_PACKAGE_PAUSE");
		receive();
	}

	void RTSI::receive()
	{
		DEBUG("Receiving...");
		// Read Header
		std::vector<char> data(HEADER_SIZE);
		boost::asio::read(*m_socket, boost::asio::buffer(data));

		uint32_t message_offset = 0;
		uint16_t msg_size = RTCUtility::getUInt16(data, message_offset);
		uint8_t msg_cmd = data.at(2);

		DEBUG("ControlHeader: ");
		DEBUG("size is: " << msg_size);
		DEBUG("command is: " << static_cast<int>(msg_cmd));

		// Read Body
		data.resize(msg_size - HEADER_SIZE);
		boost::asio::read(*m_socket, boost::asio::buffer(data));

		switch (RTSICommand(msg_cmd))
		{
		case RTSI_TEXT_MESSAGE:
		{
			uint8_t msg_length = data.at(0);
			for (int i = 1; i < msg_length; i++)
			{
				DEBUG(data[i]);
			}
			break;
		}

		case cs_rtsi::RTSI::RTSI_REQUEST_PROTOCOL_VERSION:
		{
			uint8_t negotiate_result = data.at(0);
			std::cout << "Protocol negotiate result : " << unsigned(negotiate_result) << "\n";
			break;

		}
		case cs_rtsi::RTSI::RTSI_GET_ELITECONTROL_VERSION:
		{
			DEBUG("ControlVersion: ");
			//std::uint32_t message_offset = 0;
			//std::uint32_t v_major = RTCUtility::getUInt32(data, message_offset);
			//std::uint32_t v_minor = RTCUtility::getUInt32(data, message_offset);
			//std::uint32_t v_bugfix = RTCUtility::getUInt32(data, message_offset);
			//std::uint32_t v_build = RTCUtility::getUInt32(data, message_offset);
			//DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
			break;
		}
		case cs_rtsi::RTSI::RTSI_CONTROL_PACKAGE_SETUP_INPUTS:
		{

			std::string datatypes(std::begin(data) + 1, std::end(data));
			DEBUG("Datatype:" << datatypes);
			//std::cout << "Datatype:" << datatypes << std::endl;
			std::string in_use_str("IN_USE");
			if (datatypes.find(in_use_str) != std::string::npos)
			{
				throw std::runtime_error(
					"One of the RTSI input registers are already in use! Currently you must disable the EtherNet/IP adapter, "
					"PROFINET or any MODBUS unit configured on the robot. This might change in the future.");
			}
			break;
		}
		case cs_rtsi::RTSI::RTSI_CONTROL_PACKAGE_SETUP_OUTPUTS:
		{
			uint8_t subscription_id = data.at(0);
			std::cout << "Subscription id : " << unsigned(subscription_id) << "\n";
			std::string datatypes(std::begin(data) + 1, std::end(data));
			DEBUG("Datatype:" << datatypes);
			m_output_types = RTCUtility::split(datatypes, ',');

			std::string not_found_str("NOT_FOUND");
			std::vector<int> not_found_indexes;
			if (datatypes.find(not_found_str) != std::string::npos)
			{
				for (unsigned int i = 0; i < m_output_types.size(); i++)
				{
					if (m_output_types[i] == "NOT_FOUND")
						not_found_indexes.push_back(i);
				}

				std::string vars_not_found;
				for (unsigned int i = 0; i < not_found_indexes.size(); i++)
				{
					vars_not_found += m_output_names[not_found_indexes[i]];
					if (i != not_found_indexes.size() - 1)
						vars_not_found += ", ";
				}

				std::string error_str(
					"The following variables was not found by the controller: [" + vars_not_found +
					"]\n ");
				throw std::runtime_error(error_str);
			}
			break;
		}		
		case cs_rtsi::RTSI::RTSI_CONTROL_PACKAGE_START:
		{
			char success = data.at(0);
			DEBUG("success: " << static_cast<bool>(success));
			auto rtde_success = static_cast<bool>(success);
			if (rtde_success)
			{
				m_conn_state = ConnectionState::STARTED;
				setConState();
				if (m_verbose)
					std::cout << "RTSI synchronization started" << std::endl;
			}
			else
				std::cerr << "Unable to start synchronization" << std::endl;

			break;
		}
		case cs_rtsi::RTSI::RTSI_CONTROL_PACKAGE_PAUSE:
		{
			char success = data.at(0);
			auto pause_success = static_cast<bool>(success);
			DEBUG("success: " << pause_success);
			if (pause_success)
			{
				m_conn_state = ConnectionState::PAUSED;
				setConState();
				DEBUG("RTSI synchronization paused!");
			}
			else
				std::cerr << "Unable to pause synchronization" << std::endl;
			break;
		}
		default:
			DEBUG("Unknown Command: " << static_cast<int>(msg_cmd));
			break;
		}

	}

	void RTSI::receiveData(std::shared_ptr<RobotState> &robot_state)
	{
		DEBUG("Receiving...");
		// Read Header
		std::vector<char> data(HEADER_SIZE);
		boost::asio::read(*m_socket, boost::asio::buffer(data));
		// DEBUG("Reply length is: " << reply_length);
		uint32_t message_offset = 0;
		uint16_t msg_size = RTCUtility::getUInt16(data, message_offset);
		uint8_t msg_cmd = data.at(2);

		DEBUG("ControlHeader: ");
		DEBUG("size is: " << msg_size);
		DEBUG("command is: " << static_cast<int>(msg_cmd));


		switch (RTSICommand(msg_cmd))
		{
		case cs_rtsi::RTSI::RTSI_TEXT_MESSAGE:
		{
			// Read Body
			data.resize(msg_size - HEADER_SIZE);
			boost::asio::read(*m_socket, boost::asio::buffer(data));

			message_offset = 0;
			uint8_t msg_length = data.at(0);
			for (int i = 1; i < msg_length; i++)
			{
				DEBUG("{}", data[i]);
			}
			break;
		}
		case cs_rtsi::RTSI::RTSI_DATA_PACKAGE:
		{
			DEBUG("enter RTSI_DATA_PACKAGE");
			// Read Body
			data.resize(msg_size - HEADER_SIZE);
			boost::asio::read(*m_socket, boost::asio::buffer(data));

			// Read ID
			message_offset = 0;
			RTCUtility::getUChar(data, message_offset);

			robot_state->lockUpdateStateMutex();

			// Read all the variables specified by the user.

			for (const auto &output_name : m_output_names)
			{
				// check if key exists
				if (m_cb_map.count(output_name)>0)
				{
					//call handle function
					m_cb_map[output_name](robot_state, data, message_offset);
				}
				else
				{
					DEBUG("Unknown variable name: " << output_name << " please verify the output setup!");
				}


			}

			robot_state->unlockUpdateStateMutex();

			break;
		}
		default:

			DEBUG("Unknown Command: " << static_cast<int>(msg_cmd));

			break;
		}

	}


	std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t> RTSI::getControllerVersion()
	{
		std::uint8_t cmd = RTSI_GET_ELITECONTROL_VERSION;
		sendAll(cmd, "");
		DEBUG("Done sending RTSI_GET_CSCONTROL_VERSION");
		std::vector<char> data(HEADER_SIZE);
		boost::asio::read(*m_socket, boost::asio::buffer(data));
		uint32_t message_offset = 0;
		uint16_t msg_size = RTCUtility::getUInt16(data, message_offset);
		uint8_t msg_cmd = data.at(2);
		// Read Body
		data.resize(msg_size - HEADER_SIZE);
		boost::asio::read(*m_socket, boost::asio::buffer(data));

		if (msg_cmd == RTSI_GET_ELITECONTROL_VERSION)
		{
			message_offset = 0;
			std::uint32_t v_major = RTCUtility::getUInt32(data, message_offset);
			std::uint32_t v_minor = RTCUtility::getUInt32(data, message_offset);
			std::uint32_t v_bugfix = RTCUtility::getUInt32(data, message_offset);
			std::uint32_t v_build = RTCUtility::getUInt32(data, message_offset);
			DEBUG(v_major << "." << v_minor << "." << v_bugfix << "." << v_build);
			return std::make_tuple(v_major, v_minor, v_bugfix, v_build);
		}
		else
		{
			std::uint32_t v_major = 0;
			std::uint32_t v_minor = 0;
			std::uint32_t v_bugfix = 0;
			std::uint32_t v_build = 0;
			return std::make_tuple(v_major, v_minor, v_bugfix, v_build);
		}
	}


	namespace details
	{
		/*! @brief This function creates a callback map entry for a given key
		  @tparam T Fully qualified type of the signature of the function to be called
		  @tparam S Return type of the parsing function, should be of type T with equal or less qualifiers
		  @param map A reference to the callback map
		  @param key The key of the function callback
		  @param fun A pointer to the robot state function that shall be called for the given key with data value T
		  @param parse_fun A pointer to the parsing function, which will parse data and msg_offset to the data value S
		*/

		template <class T,class S>
		void setupCallback(cs_rtsi::details::cb_map &map,
			const std::string &key,
			void (cs_rtsi::RobotState::*fun)(T),
			S(*parse_fun)(const std::vector<char> &, uint32_t &))
		{
			map.emplace(key, [fun, parse_fun](std::shared_ptr<cs_rtsi::RobotState> state_ptr, 
				const std::vector<char> &data,
				uint32_t &msg_offset) 
			{
				// calls robot_state->setVarFun(RTDEUtility::parseVarFun(data,offset))
				(*state_ptr.*fun)((*parse_fun)(data, msg_offset));
			}
			);
		}

		// 帮助宏 to reduce the manually written code for registration of callbacks
		//for output_registers
#define NUMBERED_REGISTER_NAME(type, num) "output_" #type "_register" #num
#define NUMBERED_REGISTER_FUN(type, num) setOutput_##type##_register_##num

#define OUTPUT_REGISTER_CALLBACK(num)                                                                           \
  setupCallback(m_cb_map, NUMBERED_REGISTER_NAME(int, num),    \
		&cs_rtsi::RobotState::NUMBERED_REGISTER_FUN(int, num), \
        &RTCUtility::getInt32);                                                                         \
  setupCallback(m_cb_map, NUMBERED_REGISTER_NAME(double, num),                                                    \
        &cs_rtsi::RobotState::NUMBERED_REGISTER_FUN(double, num),\
	    &RTCUtility::getDouble);

	}

	void RTSI::setupCallbacks()
	{
		using namespace cs_rtsi::details;

		//payload
		setupCallback(m_cb_map, "payload_mass", &cs_rtsi::RobotState::setPayload_mass, &RTCUtility::getDouble);
		setupCallback(m_cb_map, "payload_cog", &cs_rtsi::RobotState::setPayload_cog, &RTCUtility::unpackVector3d);
		// general
		setupCallback(m_cb_map, "timestamp", &cs_rtsi::RobotState::setTimestamp, &RTCUtility::getDouble);
		//setupCallback(m_cb_map, "actual_execution_time", &cs_rtsi::RobotState::setActual_execution_time,
		//	&RTCUtility::getDouble);
		setupCallback(m_cb_map, "robot_mode", &cs_rtsi::RobotState::setRobot_mode, &RTCUtility::getInt32);
		setupCallback(m_cb_map, "joint_mode", &cs_rtsi::RobotState::setJoint_mode, &RTCUtility::unpackVector6Int32);
		setupCallback(m_cb_map, "safety_mode", &cs_rtsi::RobotState::setSafety_mode, &RTCUtility::getInt32);
		setupCallback(m_cb_map, "safety_status", &cs_rtsi::RobotState::setSafety_status, &RTCUtility::getInt32);
		setupCallback(m_cb_map, "runtime_state", &cs_rtsi::RobotState::setRuntime_state, &RTCUtility::getUInt32);

		// joint space
		setupCallback(m_cb_map, "target_joint_positions", &cs_rtsi::RobotState::setTarget_q, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "target_joint_speeds", &cs_rtsi::RobotState::setTarget_qd, &RTCUtility::unpackVector6d);
		//setupCallback(m_cb_map, "target_qdd", &cs_rtsi::RobotState::setTarget_qdd, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "actual_joint_positions", &cs_rtsi::RobotState::setActual_q, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "actual_joint_speeds", &cs_rtsi::RobotState::setActual_qd, &RTCUtility::unpackVector6d);
		



		// cartesian space
		setupCallback(m_cb_map, "actual_TCP_pose", &cs_rtsi::RobotState::setActual_TCP_pose, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "actual_TCP_speed", &cs_rtsi::RobotState::setActual_TCP_speed, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "target_TCP_pose", &cs_rtsi::RobotState::setTarget_TCP_pose, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "target_TCP_speed", &cs_rtsi::RobotState::setTarget_TCP_speed, &RTCUtility::unpackVector6d);


		// drives and control
		//setupCallback(m_cb_map, "joint_control_output", &cs_rtsi::RobotState::setJoint_control_output,
		//	&RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "joint_temperatures", &cs_rtsi::RobotState::setJoint_temperatures,
			&RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "speed_scaling", &cs_rtsi::RobotState::setSpeed_scaling, &RTCUtility::getDouble);
		setupCallback(m_cb_map, "target_speed_fraction", &cs_rtsi::RobotState::setTarget_speed_fraction,
			&RTCUtility::getDouble);


		// forces
		setupCallback(m_cb_map, "actual_TCP_force", &cs_rtsi::RobotState::setActual_TCP_force, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "ft_raw_wrench", &cs_rtsi::RobotState::setActual_sensor_force, &RTCUtility::unpackVector6d);

		// currents and torque
		//setupCallback(m_cb_map, "target_current", &cs_rtsi::RobotState::setTarget_current, &RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "actual_current", &cs_rtsi::RobotState::setActual_current, &RTCUtility::unpackVector6d);
		//setupCallback(m_cb_map, "target_moment", &cs_rtsi::RobotState::setTarget_moment, &RTCUtility::unpackVector6d);
		//setupCallback(m_cb_map, "actual_momentum", &cs_rtsi::RobotState::setActual_momentum, &RTCUtility::getDouble);
		//setupCallback(m_cb_map, "actual_main_voltage", &cs_rtsi::RobotState::setActual_main_voltage, &RTCUtility::getDouble);
		setupCallback(m_cb_map, "actual_robot_voltage", &cs_rtsi::RobotState::setActual_robot_voltage,
			&RTCUtility::getDouble);
		setupCallback(m_cb_map, "actual_robot_current", &cs_rtsi::RobotState::setActual_robot_current,
			&RTCUtility::getDouble);
		setupCallback(m_cb_map, "actual_joint_current", &cs_rtsi::RobotState::setActual_joint_current,
			&RTCUtility::unpackVector6d);
		setupCallback(m_cb_map, "actual_joint_torques", &cs_rtsi::RobotState::setActual_joint_torques,
			&RTCUtility::unpackVector6d);


		/* actual_tool_acc is the only function relying on unpackVec3 which can not be differentiated from unpackVec6
		by the templates of setupCallback() as both are type vec<double>. Therefore pass the parsing function manually (4th
		arg) Long term fix would be to change vec6 to arr6 and and vec3 to arr3 which makes them different types */
		//setupCallback(m_cb_map, "actual_tool_accelerometer", &cs_rtsi::RobotState::setActual_tool_accelerometer,
		//	&RTCUtility::unpackVector3d);

		// I/O
		setupCallback(m_cb_map, "actual_digital_input_bits", &cs_rtsi::RobotState::setActual_digital_input_bits,
			&RTCUtility::getUInt32);
		setupCallback(m_cb_map, "actual_digital_output_bits", &cs_rtsi::RobotState::setActual_digital_output_bits,
			&RTCUtility::getUInt32);
		setupCallback(m_cb_map, "robot_status_bits", &cs_rtsi::RobotState::setRobot_status_bits, &RTCUtility::getUInt32);
		setupCallback(m_cb_map, "safety_status_bits", &cs_rtsi::RobotState::setSafety_status_bits, &RTCUtility::getUInt32);


		// io registers
		setupCallback(m_cb_map, "standard_analog_input0", &cs_rtsi::RobotState::setStandard_analog_input_0,
			&RTCUtility::getDouble);
		setupCallback(m_cb_map, "standard_analog_input1", &cs_rtsi::RobotState::setStandard_analog_input_1,
			&RTCUtility::getDouble);
		setupCallback(m_cb_map, "standard_analog_output0", &cs_rtsi::RobotState::setStandard_analog_output_0,
			&RTCUtility::getDouble);
		setupCallback(m_cb_map, "standard_analog_output1", &cs_rtsi::RobotState::setStandard_analog_output_1,
			&RTCUtility::getDouble);

		    OUTPUT_REGISTER_CALLBACK(0)
			OUTPUT_REGISTER_CALLBACK(1)
			OUTPUT_REGISTER_CALLBACK(2)
			OUTPUT_REGISTER_CALLBACK(3)
			OUTPUT_REGISTER_CALLBACK(4)
			OUTPUT_REGISTER_CALLBACK(5)
			OUTPUT_REGISTER_CALLBACK(6)
			OUTPUT_REGISTER_CALLBACK(7)

			OUTPUT_REGISTER_CALLBACK(8)
			OUTPUT_REGISTER_CALLBACK(9)
			OUTPUT_REGISTER_CALLBACK(10)
			OUTPUT_REGISTER_CALLBACK(11)
			OUTPUT_REGISTER_CALLBACK(12)
			OUTPUT_REGISTER_CALLBACK(13)
			OUTPUT_REGISTER_CALLBACK(14)
			OUTPUT_REGISTER_CALLBACK(15)

			OUTPUT_REGISTER_CALLBACK(16)
			OUTPUT_REGISTER_CALLBACK(17)
			OUTPUT_REGISTER_CALLBACK(18)
			OUTPUT_REGISTER_CALLBACK(19)
			OUTPUT_REGISTER_CALLBACK(20)
			OUTPUT_REGISTER_CALLBACK(21)
			OUTPUT_REGISTER_CALLBACK(22)
			OUTPUT_REGISTER_CALLBACK(23)

			OUTPUT_REGISTER_CALLBACK(24)
			OUTPUT_REGISTER_CALLBACK(25)
			OUTPUT_REGISTER_CALLBACK(26)
			OUTPUT_REGISTER_CALLBACK(27)
			OUTPUT_REGISTER_CALLBACK(28)
			OUTPUT_REGISTER_CALLBACK(29)
			OUTPUT_REGISTER_CALLBACK(30)
			OUTPUT_REGISTER_CALLBACK(31)

			OUTPUT_REGISTER_CALLBACK(32)
			OUTPUT_REGISTER_CALLBACK(33)
			OUTPUT_REGISTER_CALLBACK(34)
			OUTPUT_REGISTER_CALLBACK(35)
			OUTPUT_REGISTER_CALLBACK(36)
			OUTPUT_REGISTER_CALLBACK(37)
			OUTPUT_REGISTER_CALLBACK(38)
			OUTPUT_REGISTER_CALLBACK(39)

			OUTPUT_REGISTER_CALLBACK(40)
			OUTPUT_REGISTER_CALLBACK(41)
			OUTPUT_REGISTER_CALLBACK(42)
			OUTPUT_REGISTER_CALLBACK(43)
			OUTPUT_REGISTER_CALLBACK(44)
			OUTPUT_REGISTER_CALLBACK(45)
			OUTPUT_REGISTER_CALLBACK(46)
			OUTPUT_REGISTER_CALLBACK(47)
	}


}
