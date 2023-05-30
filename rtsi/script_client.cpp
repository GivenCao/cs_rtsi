#include "script_client.h"

#include <algorithm>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>

#include "rtsi_control_script.h"

using boost::asio::ip::tcp;

namespace cs_rtsi
{
	ScriptClient::ScriptClient(std::string hostname, uint32_t major_control_version, uint32_t minor_control_version,
		int port, bool verbose)
		: m_hostname(std::move(hostname)),
		m_major_control_version(major_control_version),
		m_minor_control_version(minor_control_version),
		m_port(port),
		m_verbose(verbose),
		m_conn_state(ConnectionState::DISCONNECTED)
	{
	}

	ScriptClient::~ScriptClient() = default;

	void ScriptClient::connect()
	{
		m_io_service = std::make_shared<boost::asio::io_service>();
		m_socket.reset(new boost::asio::ip::tcp::socket(*m_io_service));
		m_socket->open(boost::asio::ip::tcp::v4());
		boost::asio::ip::tcp::no_delay no_delay_option(true);
		boost::asio::socket_base::reuse_address sol_reuse_option(true);
		m_socket->set_option(no_delay_option);
		m_socket->set_option(sol_reuse_option);
		m_resolver = std::make_shared<tcp::resolver>(*m_io_service);
		tcp::resolver::query query(m_hostname, std::to_string(m_port));
		boost::asio::connect(*m_socket, m_resolver->resolve(query));
		m_conn_state = ConnectionState::CONNECTED;
		if (m_verbose)
			std::cout << "Connected successfully to CS script server: " << m_hostname << " at " << m_port << std::endl;
	}

	bool ScriptClient::isConnected()
	{
		return m_conn_state == ConnectionState::CONNECTED;
	}

	void ScriptClient::disconnect()
	{
		/* We use reset() to safely close the socket,
		 * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
		 */
		m_socket.reset();
		m_conn_state = ConnectionState::DISCONNECTED;
		if (m_verbose)
			std::cout << "Script Client - Socket disconnected" << std::endl;
	}

	void ScriptClient::receive(std::vector<char>& data)
	{
		//std::vector<char> data;
		boost::asio::read(*m_socket, boost::asio::buffer(data));

	}

	bool ScriptClient::sendScriptCommand(const std::string& cmd_str)
	{
		if (isConnected() && !cmd_str.empty())
		{
			boost::asio::write(*m_socket, boost::asio::buffer(cmd_str));
		}
		else
		{
			std::cerr << "Please connect to the controller before calling sendScriptCommand()" << std::endl;
			return false;
		}

		return true;
	}

	void ScriptClient::setScriptFile(const std::string& file_name)
	{
		m_script_file_name = file_name;
	}

	//内部私有助手函数，用于加载脚本以避免重复代码
	static bool loadScript(const std::string& file_name, std::string& str)
	{
		// Read in the CS script file
		// Notice! We use this method as it allocates the memory up front, strictly for performance.
		std::ifstream file(file_name.c_str());
		if (file)
		{
			file.seekg(0, std::ios::end);
			str.reserve(file.tellg());
			file.seekg(0, std::ios::beg);
			// Do not remove the redundant parentheses, this is to avoid the most vexing parse!
			str.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
			return true;
		}
		else
		{
			std::cerr << "There was an error reading the provided script file: " << file_name << std::endl;
			return false;
		}
	}

	bool ScriptClient::sendScript()
	{
		std::string cs_script;
		// If the user assigned a custom control script, then we use this one instead
		// of the internal compiled one.
		if (!m_script_file_name.empty())
		{
			// If loading fails, we fall back to the default script file
			if (!loadScript(m_script_file_name, cs_script))
			{
				std::cerr << "Error loading custom script file. Falling back to internal script file." << std::endl;
				cs_script = std::string();
			}
		}

		if (cs_script.empty())
		{
			cs_script = "def rtsi_control():\n";
			cs_script += CS_SCRIPT;
			cs_script +="end\n";
		}

		//std::cout << cs_script << std::endl;
		// Remove lines not fitting for the specific version of the controller
		auto n = cs_script.find("$");

		while (n != std::string::npos)
		{
			const std::string major_str(1, cs_script.at(n + 2));
			const std::string minor_str(1, cs_script.at(n + 3));

			if (!major_str.empty() && !minor_str.empty() && major_str != " " && minor_str != " ")
			{
				uint32_t major_version_needed = uint32_t(std::stoi(major_str));
				uint32_t minor_version_needed = uint32_t(std::stoi(minor_str));

				if ((m_major_control_version > major_version_needed) ||
					(m_major_control_version == major_version_needed && m_minor_control_version >= minor_version_needed))
				{
					// Keep the line
					cs_script.erase(n, 4);
					cs_script.insert(n, "    ");
				}
				else
				{
					// Erase the line
					cs_script.erase(n, cs_script.find("\n", n) - n + 1);
				}
			}
			else
			{
				std::cerr << "Could not read the control version required from the control script!" << std::endl;
				return false;
			}

			n = cs_script.find("$");
		}

		// Now scan the script for injection points where we can inject additional
		// script code
		for (const auto& script_injection : m_script_injections)
		{
			n = cs_script.find(script_injection.search_string);
			if (std::string::npos == n)
			{
				if (m_verbose)
					std::cout << "script_injection [" << script_injection.search_string << "] not found in script" << std::endl;
				continue;
			}

			// Now inject custom script code into the script
			cs_script.insert(n + script_injection.search_string.length(), script_injection.inject_string);
			if (m_verbose)
			{
				std::cout << "script_injection [" << script_injection.search_string << "] found at pos " << n << std::endl;
				std::cout << cs_script.substr(n - 100, n + script_injection.search_string.length() +
					script_injection.inject_string.length() + 100)
					<< std::endl;
			}
		}


		//std::cout << cs_script << std::endl;

		if (isConnected() && !cs_script.empty())
		{
			boost::asio::write(*m_socket, boost::asio::buffer(cs_script));
		}
		else
		{
			std::cerr << "Please connect to the controller before calling sendScript()" << std::endl;
			return false;
		}

		return true;
	}

	bool ScriptClient::sendScript(const std::string& file_name)
	{
		std::string str;
		if (!loadScript(file_name, str))
		{
			return false;
		}

		if (isConnected() && !str.empty())
		{
			boost::asio::write(*m_socket, boost::asio::buffer(str));
		}
		else
		{
			std::cerr << "Please connect to the controller before calling sendScript()" << std::endl;
			return false;
		}

		return true;
	}


	void ScriptClient::setScriptInjection(const std::string& search_string, const std::string& inject_string)
	{
		auto it = std::find_if(m_script_injections.begin(), m_script_injections.end(),
			[&](const ScriptInjectItem& val) { return search_string == val.search_string; });
		if (it != m_script_injections.end())
		{
			it->inject_string = inject_string;
		}
		else
		{
			m_script_injections.push_back({ search_string, inject_string });
		}
	}


	std::string ScriptClient::getScript()
	{
		std::string cs_script;
		// If the user assigned a custom control script, then we use this one instead
		// of the internal compiled one.
		if (!m_script_file_name.empty())
		{
			// If loading fails, we fall back to the default script file
			if (!loadScript(m_script_file_name, cs_script))
			{
				std::cerr << "Error loading custom script file. Falling back to internal script file." << std::endl;
				cs_script = std::string();
			}
		}

		if (cs_script.empty())
		{
			cs_script = CS_SCRIPT;
		}

		// Remove lines not fitting for the specific version of the controller
		auto n = cs_script.find("$");

		while (n != std::string::npos)
		{
			const std::string major_str(1, cs_script.at(n + 2));
			const std::string minor_str(1, cs_script.at(n + 3));

			if (!major_str.empty() && !minor_str.empty() && major_str != " " && minor_str != " ")
			{
				uint32_t major_version_needed = uint32_t(std::stoi(major_str));
				uint32_t minor_version_needed = uint32_t(std::stoi(minor_str));

				if ((m_major_control_version > major_version_needed) ||
					(m_major_control_version == major_version_needed && m_minor_control_version >= minor_version_needed))
				{
					// Keep the line
					cs_script.erase(n, 4);
					cs_script.insert(n, "    ");
				}
				else
				{
					// Erase the line
					cs_script.erase(n, cs_script.find("\n", n) - n + 1);
				}
			}
			else
			{
				std::cerr << "Could not read the control version required from the control script!" << std::endl;
			}

			n = cs_script.find("$");
		}

		// Now scan the script for injection points where we can inject additional
		// script code
		for (const auto& script_injection : m_script_injections)
		{
			n = cs_script.find(script_injection.search_string);
			if (std::string::npos == n)
			{
				if (m_verbose)
					std::cout << "script_injection [" << script_injection.search_string << "] not found in script" << std::endl;
				continue;
			}

			// Now inject custom script code into the script
			cs_script.insert(n + script_injection.search_string.length(), script_injection.inject_string);
			if (m_verbose)
			{
				std::cout << "script_injection [" << script_injection.search_string << "] found at pos " << n << std::endl;
				std::cout << cs_script.substr(n - 100, n + script_injection.search_string.length() +
					script_injection.inject_string.length() + 100)
					<< std::endl;
			}
		}

		return cs_script;
	}


}