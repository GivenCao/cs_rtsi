#pragma once

#include "rtsi_export.h"
#include "dashboard_enums.h"
#include <memory>
#include <string>

//boost库
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/deadline_timer.hpp>

namespace cs_rtsi
{
	/*
	* this class provide the interface foe access to the cs dashboard server
	*/

	class DashboardClient
	{
	public:

		RTSI_EXPORT explicit DashboardClient(std::string hostname, int port = 29999, bool verbose = false);
		RTSI_EXPORT virtual ~DashboardClient();
		
		enum class ConnectionState : std::uint8_t
		{
			DISCONNECTED = 0,
		    CONNECTED = 1
		};

	public:

		/***************************
		@brief      : connect to the dashboard server with the given timeout value
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT bool connect(uint32_t timeout_ms = 2000);

		/***************************
		@brief      : return true if the dashboard client is connected to the server
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT bool isConnected();
		RTSI_EXPORT void disconnect();
		void send(const std::string &str);
		std::string receive();

		/***************************
		@brief      : 清除报警
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void clear();

		/***************************
		@brief      : 检查连接状态
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void echo();



		/***************************
		@brief      : throw the exception if program fails to start
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void play();

		/***************************
		@brief      : 主要用于获取/设置机器人速度(范围为2-100)
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void speed(uint32_t value);


		/***************************
		@brief      : throw the exception if program fails to stop
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void stop();

		/***************************
		@brief      : throw the exception if program fails to pause
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void pause();

		/***************************
		@brief      : close connection
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void quit();

		/***************************
		@brief      :主要用于通过 dashboard shell服务器重启 EliRobot，
		需注意的是重启 EliRobot会连同dashboard shell服务器一并重启，
		因此在重启后需要重连 dashboard shell服务器
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void reboot();

		/***************************
		@brief      :主要用于获取机器人信息(目前主要为机器人类型
		@time       : 2023/03/27
		****************************/	
		RTSI_EXPORT std::string robotType();

		/***************************
		@brief      : shut down and turn off robot and controller
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void shutdown();

		/***************************
		@brief      : execution state enquiry
		@return     : return true if program is running
		@time       : 2023/03/27
		****************************/
		bool running();

		/***************************
		@brief      : 弹出一个显示给定文本的消息框
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void popup(const std::string &message);

		/***************************
		@brief      : 关闭最近由 popup命令的消息框
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void closePopup();

		/***************************
		@brief      : 关闭一个安全的popup
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void closeSafetyPopup();

		/***************************
		@brief      : power on the robot
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void powerOn();

		/***************************
		@brief      : power off the arm
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void powerOff();

		/***************************
		@brief      : release the brake
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void brakeRelease();

		/***************************
		@brief      : close the current popup and unlock protective stop.
		the unlock protective stop command fails with an exception if less than 5 
		seconds has passed since the protective stop occurred.
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void unlockProtectiveStop();

		/***************************
		@brief      : 当机器人出现安全故障或违规时，使用此功能重新启动安全。
		安全装置重新启动后，机器人将处于断电状态。
		你应该始终确保重新启动系统是可以的。
		强烈建议在使用之前检查错误日志
		命令(通过PolyScope或ssh连接)。
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void restartSafety();

		std::string polyscopeVersion();
		std::string programState();

		//主要用于获取机器人模式。 
		RTSI_EXPORT std::string robotMode();

		std::string getRobotModel();
		std::string getLoadedProgram();


		//主要用于获取机器人当前状态信息(速度、机器人模式、安全模式、运行状态等)
		std::string status();




		/***************************
		@brief      : 安全模式查询。由任何类型的保护I/O或
		可配置的I/O三个位置使能设备导致SAFEGUARD_STOP的保护停止。
        该函数不建议使用。相反，应该使用safetystatus()。
		@time       : 2023/03/27
		****************************/
		std::string safetymode();

		/***************************
		@brief      : 安全状态查询。
		这与'safetymode'不同，它指定给定的保护停止是由永久保护I/O停止、
		可配置I/O自动模式保护停止或可配置I/O三个位置引起的
		使能设备停止。因此，这严格来说比safetymode()更详细。
		@time       : 2023/03/27
		****************************/
		std::string safetystatus();



		/***************************
		@brief      : adds log message to the log history
		@time       : 2023/03/27
		****************************/
		void addToLog(const std::string &message);

		/***************************
		@brief      : return the save state of the active program
		@time       : 2023/03/27
		****************************/
		bool isProgramSaved();

		/***************************
		@brief      : Returns the remote control status of the robot.
		If the robot is in remote control it returns true and if remote control
		is disabled or robot is in local control it returns false.
		@time       : 2023/03/27
		****************************/
		bool isInRemoteControl();

		void setUserRole(const UserRole &role);


		/***************************
		@brief      : Returns serial number of the robot. (Serial number like "20175599999")
		@return     : serial number as a std::string
		@time       : 2023/03/27
		****************************/
		std::string getSerialNumber();

		private:
			//for socket timeouts
			void check_deadline();

			std::string m_hostname;
			int m_port;
			bool m_verbose;
			ConnectionState m_conn_state;
			boost::asio::io_service m_io_service;

			std::shared_ptr<boost::asio::ip::tcp::socket> mp_socket;
			std::shared_ptr<boost::asio::ip::tcp::resolver> mp_resolver;

			boost::asio::deadline_timer m_deadline;






	};
}
