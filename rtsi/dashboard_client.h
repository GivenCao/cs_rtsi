#pragma once

#include "rtsi_export.h"
#include "dashboard_enums.h"
#include <memory>
#include <string>

//boost��
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
		@brief      : �������
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void clear();

		/***************************
		@brief      : �������״̬
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void echo();



		/***************************
		@brief      : throw the exception if program fails to start
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void play();

		/***************************
		@brief      : ��Ҫ���ڻ�ȡ/���û������ٶ�(��ΧΪ2-100)
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
		@brief      :��Ҫ����ͨ�� dashboard shell���������� EliRobot��
		��ע��������� EliRobot����ͬdashboard shell������һ��������
		�������������Ҫ���� dashboard shell������
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void reboot();

		/***************************
		@brief      :��Ҫ���ڻ�ȡ��������Ϣ(Ŀǰ��ҪΪ����������
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
		@brief      : ����һ����ʾ�����ı�����Ϣ��
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void popup(const std::string &message);

		/***************************
		@brief      : �ر������ popup�������Ϣ��
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void closePopup();

		/***************************
		@brief      : �ر�һ����ȫ��popup
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
		@brief      : �������˳��ְ�ȫ���ϻ�Υ��ʱ��ʹ�ô˹�������������ȫ��
		��ȫװ�����������󣬻����˽����ڶϵ�״̬��
		��Ӧ��ʼ��ȷ����������ϵͳ�ǿ��Եġ�
		ǿ�ҽ�����ʹ��֮ǰ��������־
		����(ͨ��PolyScope��ssh����)��
		@time       : 2023/03/27
		****************************/
		RTSI_EXPORT void restartSafety();

		std::string polyscopeVersion();
		std::string programState();

		//��Ҫ���ڻ�ȡ������ģʽ�� 
		RTSI_EXPORT std::string robotMode();

		std::string getRobotModel();
		std::string getLoadedProgram();


		//��Ҫ���ڻ�ȡ�����˵�ǰ״̬��Ϣ(�ٶȡ�������ģʽ����ȫģʽ������״̬��)
		std::string status();




		/***************************
		@brief      : ��ȫģʽ��ѯ�����κ����͵ı���I/O��
		�����õ�I/O����λ��ʹ���豸����SAFEGUARD_STOP�ı���ֹͣ��
        �ú���������ʹ�á��෴��Ӧ��ʹ��safetystatus()��
		@time       : 2023/03/27
		****************************/
		std::string safetymode();

		/***************************
		@brief      : ��ȫ״̬��ѯ��
		����'safetymode'��ͬ����ָ�������ı���ֹͣ�������ñ���I/Oֹͣ��
		������I/O�Զ�ģʽ����ֹͣ�������I/O����λ�������
		ʹ���豸ֹͣ����ˣ����ϸ���˵��safetymode()����ϸ��
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
