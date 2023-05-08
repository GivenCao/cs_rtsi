#include "dashboard_client.h"
#include "rtc_utility.h"



#include <boost/array.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/socket_base.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

//#include <boost/array.hpp>
//#include <boost/asio/connect.hpp>
//#include <boost/asio/socket_base.hpp>
//#include <boost/asio/write.hpp>
//#include <boost/asio/error.hpp>
//#include <boost/bind.hpp>
//#include <boost/lambda/bind.hpp>
//#include <boost/lambda/lambda.hpp>
//#include <boost/date_time/posix_time/posix_time.hpp>
//#include <boost/date_time/posix_time/posix_time_duration.hpp>
//#include <boost/system/error_code.hpp>

#include <cstring>
#include <iostream>
#include <memory>
#include <regex>

using boost::asio::ip::tcp;
using boost::lambda::_1;
using boost::lambda::var;

namespace cs_rtsi
{
	DashboardClient::DashboardClient(std::string hostname, int port, bool verbose)
		:m_hostname(std::move(hostname)),
		m_port(port),
		m_verbose(verbose),
		m_conn_state(ConnectionState::DISCONNECTED),
		m_deadline(m_io_service)
	{
		//在第一个套接字操作启动之前不需要截止日期。我们
		//将截止日期设置为正无穷大，这样行动者就不会采取任何行动
		//直到设定一个具体的截止日期。
		m_deadline.expires_at(boost::posix_time::pos_infin);

		//启动持续行动者，检查截止日期是否终止
		check_deadline();

	}

	DashboardClient::~DashboardClient() = default;

	bool DashboardClient::connect(uint32_t timeout_ms)
	{
		mp_socket.reset(new boost::asio::ip::tcp::socket(m_io_service));
		mp_socket->open(boost::asio::ip::tcp::v4());
		boost::asio::ip::tcp::no_delay no_delay_option(true);
		boost::asio::socket_base::reuse_address sol_reuse_option(true);

		mp_socket->set_option(no_delay_option);
		mp_socket->set_option(sol_reuse_option);

		mp_resolver = std::make_shared<tcp::resolver>(m_io_service);
		tcp::resolver::query query(m_hostname, std::to_string(m_port));


		if (m_verbose)
		{
			std::cout << "Connecting to UR dashboard server ....." << std::endl;
		}

		m_deadline.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
		boost::system::error_code ec = boost::asio::error::would_block;
		boost::asio::async_connect(*mp_socket, mp_resolver->resolve(query), var(ec) = boost::lambda::_1);

		do 
		{
			m_io_service.run_one();
		} while (ec == boost::asio::error::would_block);

		if (ec || !mp_socket->is_open())
		{
			return false;
			//throw std::runtime_error("Timeout connecting to UR dashboard server.");
		}
		else
		{
			m_conn_state = ConnectionState::CONNECTED;
			receive();

		}

		if (m_verbose)
			std::cout << "Connected successfully to UR dashboard server: " 
			<< m_hostname << " at " << m_port << std::endl;
		return true;
	}

	bool DashboardClient::isConnected()
	{
		return m_conn_state == ConnectionState::CONNECTED;
	}

	void DashboardClient::disconnect()
	{
		/* We use reset() to safely close the socket,
		 * see: https://stackoverflow.com/questions/3062803/how-do-i-cleanly-reconnect-a-boostsocket-following-a-disconnect
		 */
		mp_socket.reset();
		m_conn_state = ConnectionState::DISCONNECTED;
		if (m_verbose)
			std::cout << "Dashboard Client - Socket disconnected" << std::endl;
	}

	void DashboardClient::send(const std::string &str)
	{
		boost::asio::write(*mp_socket, boost::asio::buffer(str));
	}

	void DashboardClient::clear()
	{
		std::string clear = "clear -a\n";
		send(clear);
		auto result = receive();
		//需要调试
		if (result != "clear alarm\r")
		{
			throw std::runtime_error(result);
		}
	}

	void DashboardClient::echo()
	{
		std::string echo = "echo\n";
		send(echo);
		auto result = receive();
		//需要调试
		if (result != "Hello ELITE ROBOTS")
		{
			throw std::runtime_error(result);
		}

	}

	void DashboardClient::play()
	{
		std::string play = "play\n";
		send(play);
		auto result = receive();
		//需要调试
		if (result != "Starting program")
		{
			throw std::runtime_error(result);
		}
	}

	void DashboardClient::speed(uint32_t value)
	{
		//std::string speed = "speed -v %d\n";
		std::string speed = "speed -v " + std::to_string(value) + "\n";
		send(speed);
		auto result = receive();
	}
	
	void DashboardClient::stop()
	{
		std::string stop = "stop\n";
		send(stop);
		auto result = receive();
		if (result != "Stopping task\r")
		{
			throw std::runtime_error(result);
		}
	}


	void DashboardClient::pause()
	{
		std::string pause = "pause\n";
		send(pause);
		auto result = receive();
		if (result != "Pausing program")
		{
			throw std::runtime_error(result);
		}
	}

	void DashboardClient::quit()
	{
		std::string quit = "quit\n";
		send(quit);
		receive();
	}

	void DashboardClient::reboot()
	{
		std::string quit = "reboot\n";
		send(quit);
		receive();

	}

	void DashboardClient::shutdown()
	{
		std::string shutdown = "shutdown\n";
		send(shutdown);
		receive();
	}




	void DashboardClient::popup(const std::string &message)
	{
		std::string popup = "popup -s" + message + "\n";
		send(popup);
		receive();
	}

	void DashboardClient::closePopup()
	{
		std::string close_popup = "popup -c\n";
		send(close_popup);
		receive();
	}

	void DashboardClient::closeSafetyPopup()
	{
		std::string str = "closeSafetyDialog\n";
		send(str);
		receive();
	}


	std::string DashboardClient::polyscopeVersion()
	{
		std::string polyscope_version = "version\n";
		send(polyscope_version);
		auto str = receive();
		const std::regex base_regex("\\d+.\\d+.\\d+.\\d+");
		std::smatch base_match;
		std::regex_search(str, base_match, base_regex);
		if (!base_match.empty())
			return std::string(base_match[0]);
		else
			return str;
	}

	void DashboardClient::powerOn()
	{
		std::string power_on = "robotControl -on\n";
		send(power_on);
		auto str=receive();
	}

	void DashboardClient::powerOff()
	{
		std::string power_off = "robotControl -off\n";
		send(power_off);
		receive();
	}

	void DashboardClient::brakeRelease()
	{
		std::string brake_release = "brakeRelease\n";
		send(brake_release);
		auto str=receive();
	}

	void DashboardClient::unlockProtectiveStop()
	{
		std::string unlock_p_stop = "unlockProtectiveStop\n";
		send(unlock_p_stop);
		auto result = receive();
		if (result != "Protective stop unlocking...\r")
		{
			throw std::logic_error("Unlock protective stop failure: " + result);
		}
	}

	std::string DashboardClient::receive()
	{
		boost::array<char, 1024> recv_buffer_;
		boost::system::error_code error_;
		size_t buflen = mp_socket->read_some(boost::asio::buffer(recv_buffer_), error_);
		if (error_.value() != 0)
		{
			throw std::runtime_error("Dashboard client receive function failed with error: " + error_.message());
		}
		return std::string(recv_buffer_.elems, buflen - 1);  // -1 is removing newline
	}

	std::string DashboardClient::robotMode()
	{
		std::string robotmode = "robotMode\n";
		send(robotmode);
		auto state_str = receive();
		return state_str;
	}

	void DashboardClient::addToLog(const std::string &message)
	{
		std::string add_to_lof = "log -a" + message + "\n";
		send(add_to_lof);
		receive();
	}

	std::string DashboardClient::safetymode()
	{
		std::string safetymode = "safety -m\n";
		send(safetymode);
		return receive();
	}

	std::string DashboardClient::robotType()
	{
		std::string robotType = "robot -t\n";
		send(robotType);
		return receive();

	}
	std::string DashboardClient::status()
	{
		std::string safetystatus = "status\n";
		send(safetystatus);
		return receive();

	}

	std::string DashboardClient::safetystatus()
	{
		std::string safetystatus = "safety -s\n";
		send(safetystatus);
		return receive();
	}

	void DashboardClient::restartSafety()
	{
		std::string str = "safety -r\n";
		send(str);
		receive();
	}

	void DashboardClient::check_deadline()
	{
		// Check whether the deadline has passed. We compare the deadline against
		// the current time since a new asynchronous operation may have moved the
		// deadline before this actor had a chance to run.
		if (m_deadline.expires_at() <= boost::asio::deadline_timer::traits_type::now())
		{
			// The deadline has passed. The socket is closed so that any outstanding
			// asynchronous operations are cancelled. This allows the blocked
			// connect(), read_line() or write_line() functions to return.
			boost::system::error_code ignored_ec;
			mp_socket->close(ignored_ec);

			// There is no longer an active deadline. The expiry is set to positive
			// infinity so that the actor takes no action until a new deadline is set.
			m_deadline.expires_at(boost::posix_time::pos_infin);
		}

		// Put the actor back to sleep.
		m_deadline.async_wait(boost::bind(&DashboardClient::check_deadline, this));
	}





}

