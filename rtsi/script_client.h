#pragma once


#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read.hpp>
#include <memory>
#include <string>
#include <vector>

namespace cs_rtsi
{
	struct ScriptInjectItem
	{
		std::string search_string;
		std::string inject_string;
		ScriptInjectItem(const std::string& search, const std::string& inject) : search_string(search), inject_string(inject)
		{
		}

	};

	class ScriptClient
	{
	public:
		explicit ScriptClient(std::string hostname, uint32_t major_control_version,
			uint32_t minor_control_version, int port = 30001, bool verbose = false);
		virtual ~ScriptClient();


		enum class ConnectionState : std::uint8_t
		{
			DISCONNECTED = 0,
			CONNECTED = 1,
		};


	public:

		void connect();
		void disconnect();
		bool isConnected();

		void receive(std::vector<char>& data);

		/***************************
		@brief      : ָ��һ���Զ���ű��ļ������ļ������͵��豸��sendScript()���������á�
		����һ���յ�file_name�������Զ���ű����أ����޸Ŀؼ�ʱ������Խű�����Ϊ������Ҫ����
		����������
		@time       : 2023/04/11
		****************************/
		void setScriptFile(const std::string& file_name);


		/***************************
		@brief      : ���ͱ��뵽���е��ڲ����ƽű������Ŀ��ƽű��ļ�
		@time       : 2023/04/11
		****************************/
		bool sendScript();

		/***************************
		@brief      : Send the script file with the given file_name
		@time       : 2023/04/11
		****************************/
		bool sendScript(const std::string& file_name);

		bool sendScriptCommand(const std::string& cmd_str);

		void setScriptInjection(const std::string& search_string, const std::string& inject_string);

		/***************************
		@brief      : Get the corrected rtde_control script as a std::string
		@time       : 2023/04/11
		****************************/
		std::string getScript();

	private:

		std::string m_hostname;
		uint32_t m_major_control_version;
		uint32_t m_minor_control_version;
		int m_port;
		bool m_verbose;
		ConnectionState m_conn_state;
		std::string m_script_file_name;
		std::shared_ptr<boost::asio::io_service> m_io_service;
		std::shared_ptr<boost::asio::ip::tcp::socket> m_socket;
		std::shared_ptr<boost::asio::ip::tcp::resolver> m_resolver;
		std::vector<ScriptInjectItem> m_script_injections;

	};


}
