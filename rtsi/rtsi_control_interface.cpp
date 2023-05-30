#include "dashboard_client.h"
#include "robot_state.h"
#include "rtsi_control_interface.h"
#include "script_client.h"

#include "rtc_utility.h"

#include <bitset>

#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>


namespace cs_rtsi
{
	RTSIControlInterface::RTSIControlInterface(std::string hostname, uint16_t flags, int cs_cap_port)
		: m_hostname(std::move(hostname)),
		m_upload_script(flags & FLAG_UPLOAD_SCRIPT),
		m_use_external_control_cs_cap(flags & FLAG_USE_EXT_CS_CAP),
		m_verbose(flags & FLAG_VERBOSE),
		m_use_upper_range_registers(flags & FLAG_UPPER_RANGE_REGISTERS),
		m_no_wait(flags & FLAG_NO_WAIT),
		m_custom_script(flags & FLAG_CUSTOM_SCRIPT),
		m_cs_cap_port(cs_cap_port)
	{
		// Create a connection to the dashboard server
		mp_db_client = std::make_shared<DashboardClient>(m_hostname);
		//std::cout << "0000" << std::endl;
		mp_db_client->connect();

		m_port = 30004;
		m_custom_script_running = false;
		mp_rtsi = std::make_shared<RTSI>(m_hostname, m_port, m_verbose);
		mp_rtsi->connect();
		//std::cout << "enter rtsi" << std::endl;
		mp_rtsi->negotiateProtocolVersion();
		auto controller_version = mp_rtsi->getControllerVersion();
		uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);
		uint32_t minor_version = std::get<MINOR_VERSION>(controller_version);

		m_frequency = 250;

		// Set delta time to be used by receiveCallback
		m_delta_time = 1 / m_frequency;

		// Init Robot state
		mp_robot_state = std::make_shared<RobotState>();

		// Map the output registers to functions
		initOutputRegFuncMap();

		// Create a connection to the script server
		mp_script_client = std::make_shared<ScriptClient>(m_hostname, major_version, minor_version);
		mp_script_client->connect();


		// If user want to use upper range of rtsi registers, add the register offset in control script
		if (m_use_upper_range_registers)
		{
			mp_script_client->setScriptInjection("# float register offset\n", "24");
			mp_script_client->setScriptInjection("# int register offset\n", "24");
			m_register_offset = 24;
		}
		else
		{
			mp_script_client->setScriptInjection("# float register offset\n", "0");
			mp_script_client->setScriptInjection("# int register offset\n", "0");
			m_register_offset = 0;
		}

		// Setup default recipes
		setupRecipes(m_frequency);

		// Wait until RTSI data synchronization has started
		if (m_verbose)
			std::cout << "Waiting for RTSI data synchronization to start..." << std::endl;
		std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

		// Start RTSI data synchronization
		mp_rtsi->sendStart();

		while (!mp_rtsi->isStarted())
		{
			// Wait until rtsi data synchronization has started or timeout
			std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
			if (duration > RTSI_START_SYNCHRONIZATION_TIMEOUT)
			{
				break;
			}
		}

		if (!mp_rtsi->isStarted())
			throw std::logic_error("Failed to start rtsi data synchronization, before timeout");

		// Start executing receiveCallback
		mp_th = std::make_shared<boost::thread>(boost::bind(&RTSIControlInterface::receiveCallback, this));

		// Wait until the first robot state has been received
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// Clear command register
		sendClearCommand();

		if (m_upload_script)
		{
			if (!isProgramRunning())
			{
				// Send script to the UR Controller
				mp_script_client->sendScript();
				waitForProgramRunning();
			}
			else
			{
				if (m_verbose)
					std::cout << "A script was running on the controller, killing it!" << std::endl;
				// Stop the running script first
				stopScript();
				mp_db_client->stop();

				// Wait until terminated
				std::this_thread::sleep_for(std::chrono::milliseconds(100));

				// Send script to the UR Controller
				mp_script_client->sendScript();

				while (!isProgramRunning())
				{
					// Wait for program to be running
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}


		// When the user wants to a custom script / program on the controller interacting with cs_rtsi.
		if (!m_upload_script && !m_use_external_control_cs_cap)
		{
			if (!m_no_wait)
			{
				if (!isProgramRunning())
				{
					start_time = std::chrono::high_resolution_clock::now();
					std::cout << "Waiting for RTSI control program to be running on the controller" << std::endl;
					while (!isProgramRunning())
					{
						std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
						auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
						if (duration > WAIT_FOR_PROGRAM_RUNNING_TIMEOUT)
						{
							break;
						}
						// Wait for program to be running
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}

					if (!isProgramRunning())
					{
						disconnect();
						throw std::logic_error("RTSI control program is not running on controller, before timeout of " +
							std::to_string(WAIT_FOR_PROGRAM_RUNNING_TIMEOUT) + " seconds");
					}
				}
			}
		}
	}

	RTSIControlInterface::~RTSIControlInterface()
	{
		disconnect();
	}



	void RTSIControlInterface::disconnect()
	{
		// Stop the receive callback function
		m_stop_thread = true;
		mp_th->interrupt();
		mp_th->join();

		if (mp_rtsi != nullptr)
		{
			if (mp_rtsi->isConnected())
				mp_rtsi->disconnect();
		}

		if (mp_script_client != nullptr)
		{
			if (mp_script_client->isConnected())
				mp_script_client->disconnect();
		}

		if (mp_db_client != nullptr)
		{
			if (mp_db_client->isConnected())
				mp_db_client->disconnect();
		}

		// Wait until everything has disconnected
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	bool RTSIControlInterface::isConnected()
	{
		return mp_rtsi->isConnected();
	}




	bool RTSIControlInterface::reconnect()
	{
		mp_db_client->connect();
		mp_script_client->connect();
		mp_rtsi->connect();
		mp_rtsi->negotiateProtocolVersion();
		auto controller_version = mp_rtsi->getControllerVersion();
		uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

		m_frequency = 125;
		// If e-Series Robot set frequency to 500Hz
		//if (major_version > CB3_MAJOR_VERSION)
		//	frequency_ = 500;

		// Set delta time to be used by receiveCallback
		m_delta_time = 1 / m_frequency;

		// Init Robot state
		mp_robot_state = std::make_shared<RobotState>();

		// Map the output registers to functions
		initOutputRegFuncMap();

		// If user want to use upper range of rtsi registers, add the register offset in control script
		if (m_use_upper_range_registers)
		{
			mp_script_client->setScriptInjection("# float register offset\n", "24");
			mp_script_client->setScriptInjection("# int register offset\n", "24");
			m_register_offset = 24;
		}
		else
		{
			mp_script_client->setScriptInjection("# float register offset\n", "0");
			mp_script_client->setScriptInjection("# int register offset\n", "0");
			m_register_offset = 0;
		}

		// Setup default recipes
		setupRecipes(m_frequency);

		// Wait until rtsi data synchronization has started.
		if (m_verbose)
			std::cout << "Waiting for rtsi data synchronization to start..." << std::endl;
		std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

		// Start rtsi data synchronization
		mp_rtsi->sendStart();

		while (!mp_rtsi->isStarted())
		{
			// Wait until rtsi data synchronization has started or timeout
			std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
			if (duration > RTSI_START_SYNCHRONIZATION_TIMEOUT)
			{
				break;
			}
		}

		if (!mp_rtsi->isStarted())
			throw std::logic_error("Failed to start rtsi data synchronization, before timeout");

		// Start executing receiveCallback
		m_stop_thread = false;
		mp_th = std::make_shared<boost::thread>(boost::bind(&RTSIControlInterface::receiveCallback, this));

		// Wait until the first robot state has been received
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// Clear command register
		sendClearCommand();

		if (m_upload_script)
		{
			if (!isProgramRunning())
			{
				// Send script to the UR Controller
				mp_script_client->sendScript();
				waitForProgramRunning();
			}
			else
			{
				if (m_verbose)
					std::cout << "A script was running on the controller, killing it!" << std::endl;
				// Stop the running script first
				stopScript();
				mp_db_client->stop();

				// Wait until terminated
				std::this_thread::sleep_for(std::chrono::milliseconds(100));

				// Send script to the UR Controller
				mp_script_client->sendScript();

				while (!isProgramRunning())
				{
					// Wait for program to be running
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
				}
			}
		}



		// When the user wants to a custom script / program on the controller interacting with ur_rtsi.
		if (!m_upload_script && !m_use_external_control_cs_cap)
		{
			if (!m_no_wait)
			{
				if (!isProgramRunning())
				{
					start_time = std::chrono::high_resolution_clock::now();
					std::cout << "Waiting for RTSI control program to be running on the controller" << std::endl;
					while (!isProgramRunning())
					{
						std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
						auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
						if (duration > WAIT_FOR_PROGRAM_RUNNING_TIMEOUT)
						{
							break;
						}
						// Wait for program to be running
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}

					if (!isProgramRunning())
					{
						disconnect();
						throw std::logic_error("rtsi control program is not running on controller, before timeout of " +
							std::to_string(WAIT_FOR_PROGRAM_RUNNING_TIMEOUT) + " seconds");
					}
				}
			}
		}

		return true;

	}
	

	bool RTSIControlInterface::reuploadScript()
	{
		if (isProgramRunning())
		{
			if (m_verbose)
				std::cout << "A script was running on the controller, killing it!" << std::endl;

			// Stop the running script first
			stopScript();
			mp_db_client->stop();

			// Wait until terminated
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// Re-upload rtsi script to the UR Controller
		if (mp_script_client->sendScript())
		{
			if (m_verbose)
				std::cout << "The rtsi Control script has been re-uploaded." << std::endl;
			return true;
		}
		else
		{
			return false;
		}
	}


	bool RTSIControlInterface::sendCustomScriptFunction(const std::string &function_name, const std::string &script)
	{
		std::string cmd_str;
		std::string line;
		std::stringstream ss(script);
		cmd_str += "def " + function_name + "():\n";
		cmd_str += "\twrite_output_integer_register(0 +" + std::to_string(m_register_offset) + ", 1)\n";

		while (std::getline(ss, line))
		{
			cmd_str += "\t" + line + "\n";
		}

		// Signal when motions are finished
		cmd_str += "\twrite_output_integer_register(0 +" + std::to_string(m_register_offset) + ", 2)\n";
		cmd_str += "end\n";

		return sendCustomScript(cmd_str);
	}

	bool RTSIControlInterface::sendCustomScript(const std::string &script)
	{
		m_custom_script_running = true;
		// First stop the running rtsi control script
		stopScript();

		auto start_time = std::chrono::high_resolution_clock::now();

		// Send custom script function
		mp_script_client->sendScriptCommand(script);

		while (getControlScriptState() != CS_CONTROLLER_DONE_WITH_CMD)
		{
			// Wait until the controller is done with command
			auto current_time = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
			if (duration > CS_PATH_EXECUTION_TIMEOUT)
				return false;
			// Sleep to avoid high CPU load
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		sendClearCommand();

		// Re-upload rtsi script to the UR Controller
		mp_script_client->sendScript();

		while (!isProgramRunning())
		{
			// Wait for program to be running
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}

		m_custom_script_running = false;
		return true;
	}

	bool RTSIControlInterface::sendCustomScriptFile(const std::string &file_path)
	{
		m_custom_script_running = true;
		// First stop the running rtsi control script
		stopScript();

		auto start_time = std::chrono::high_resolution_clock::now();

		// Send custom script file
		mp_script_client->sendScript(file_path);

		while (getControlScriptState() != CS_CONTROLLER_DONE_WITH_CMD)
		{
			// Wait until the controller is done with command
			auto current_time = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
			if (duration > CS_PATH_EXECUTION_TIMEOUT)
				return false;
			// Sleep to avoid high CPU load
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		sendClearCommand();

		// Re-upload rtsi script to the UR Controller
		mp_script_client->sendScript();

		while (!isProgramRunning())
		{
			// Wait for program to be running
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}

		m_custom_script_running = false;
		return true;
	}

	void RTSIControlInterface::setCustomScriptFile(const std::string &file_path)
	{
		mp_script_client->setScriptFile(file_path);
		reuploadScript();
	}


	void RTSIControlInterface::stopScript()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::STOP_SCRIPT;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::isProgramRunning()
	{
		if (mp_robot_state != nullptr)
		{
			// Read Bits 0-3: Is power on(1) | Is program running(2) | Is teach button pressed(4) |
			std::bitset<32> status_bits(mp_robot_state->getRobot_status_bits());
			return status_bits.test(RobotStatus::ROBOT_STATUS_PROGRAM_RUNNING);
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	bool RTSIControlInterface::setupRecipes(const double &frequency)
	{
		// Setup output
		std::vector<std::string> state_names = { "robot_status_bits", "safety_status_bits", "runtime_state", outIntReg(0),
										outIntReg(1),        outDoubleReg(0),      outDoubleReg(1), outDoubleReg(2),
										outDoubleReg(3),     outDoubleReg(4),      outDoubleReg(5),
			                            outDoubleReg(24),      outDoubleReg(25), outDoubleReg(26),
										outDoubleReg(27),     outDoubleReg(28),      outDoubleReg(29) };
		mp_rtsi->sendOutputSetup(state_names, frequency);

		// Setup input recipes
        // RECIPE_1   £¨9_input£©
		std::vector<std::string> async_setp_input = { inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
											 inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
											 inDoubleReg(7), inIntReg(1) };
		mp_rtsi->sendInputSetup(async_setp_input);

		// RECIPE_2¡¡£¨11_input£©
		std::vector<std::string> servoj_input = { inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
												 inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
												 inDoubleReg(7), inDoubleReg(8), inDoubleReg(9), inDoubleReg(10) };
		mp_rtsi->sendInputSetup(servoj_input);

		// RECIPE_3
		std::vector<std::string> force_mode_input = {
			inIntReg(0),     inIntReg(1),     inIntReg(2),     inIntReg(3),     inIntReg(4),     inIntReg(5),
			inIntReg(6),     inIntReg(7),     inDoubleReg(0),  inDoubleReg(1),  inDoubleReg(2),  inDoubleReg(3),
			inDoubleReg(4),  inDoubleReg(5),  inDoubleReg(6),  inDoubleReg(7),  inDoubleReg(8),  inDoubleReg(9),
			inDoubleReg(10), inDoubleReg(11), inDoubleReg(12), inDoubleReg(13), inDoubleReg(14), inDoubleReg(15),
			inDoubleReg(16), inDoubleReg(17) };
		mp_rtsi->sendInputSetup(force_mode_input);

		// RECIPE_4 (0_input)
		std::vector<std::string> no_cmd_input = { inIntReg(0) };
		mp_rtsi->sendInputSetup(no_cmd_input);

		// RECIPE_5
		std::vector<std::string> servoc_input = { inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
												 inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
												 inDoubleReg(7), inDoubleReg(8) };
		mp_rtsi->sendInputSetup(servoc_input);

		// RECIPE_6
		std::vector<std::string> wrench_input = { inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
												 inDoubleReg(3), inDoubleReg(4), inDoubleReg(5) };
		mp_rtsi->sendInputSetup(wrench_input);

		// RECIPE_7
		std::vector<std::string> set_payload_input = { inIntReg(0), inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
													  inDoubleReg(3) };
		mp_rtsi->sendInputSetup(set_payload_input);

		// RECIPE_8
		std::vector<std::string> force_mode_parameters_input = { inIntReg(0), inDoubleReg(0) };
		mp_rtsi->sendInputSetup(force_mode_parameters_input);

		// RECIPE_9
		std::vector<std::string> get_actual_joint_positions_history_input = { inIntReg(0), inIntReg(1) };
		mp_rtsi->sendInputSetup(get_actual_joint_positions_history_input);

		// RECIPE_10
		int offside = 24;
		std::vector<std::string> get_inverse_kin_input = { inIntReg(0),     inDoubleReg(0+ offside),  inDoubleReg(1 + offside), inDoubleReg(2 + offside),
														  inDoubleReg(3 + offside),  inDoubleReg(4 + offside),  inDoubleReg(5 + offside), inDoubleReg(6 + offside),
														  inDoubleReg(7 + offside),  inDoubleReg(8 + offside),  inDoubleReg(9 + offside), inDoubleReg(10 + offside),
														  inDoubleReg(11 + offside)};
		mp_rtsi->sendInputSetup(get_inverse_kin_input);

		// RECIPE_11
		std::vector<std::string> watchdog_input = { inIntReg(0) };
		mp_rtsi->sendInputSetup(watchdog_input);

		// RECIPE_12
		std::vector<std::string> pose_trans_input = {
			inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2), inDoubleReg(3),  inDoubleReg(4), inDoubleReg(5),
			inDoubleReg(6), inDoubleReg(7), inDoubleReg(8), inDoubleReg(9), inDoubleReg(10), inDoubleReg(11) };
		mp_rtsi->sendInputSetup(pose_trans_input);

		// RECIPE_13
		std::vector<std::string> setp_input = { inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2), inDoubleReg(3),
											   inDoubleReg(4), inDoubleReg(5), inDoubleReg(6), inDoubleReg(7) };
		mp_rtsi->sendInputSetup(setp_input);

		// RECIPE_14
		std::vector<std::string> jog_input = { inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
											  inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6) };
		mp_rtsi->sendInputSetup(jog_input);

		// RECIPE_15
		std::vector<std::string> async_path_input = { inIntReg(0), inIntReg(1) };
		mp_rtsi->sendInputSetup(async_path_input);

		return true;
	}


	void RTSIControlInterface::initOutputRegFuncMap()
	{
		m_output_reg_func_map["getOutput_int_register0"] = std::bind(&RobotState::getOutput_int_register_0, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register1"] = std::bind(&RobotState::getOutput_int_register_1, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register2"] = std::bind(&RobotState::getOutput_int_register_2, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register3"] = std::bind(&RobotState::getOutput_int_register_3, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register4"] = std::bind(&RobotState::getOutput_int_register_4, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register5"] = std::bind(&RobotState::getOutput_int_register_5, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register6"] = std::bind(&RobotState::getOutput_int_register_6, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register7"] = std::bind(&RobotState::getOutput_int_register_7, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register8"] = std::bind(&RobotState::getOutput_int_register_8, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register9"] = std::bind(&RobotState::getOutput_int_register_9, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register10"] = std::bind(&RobotState::getOutput_int_register_10, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register11"] = std::bind(&RobotState::getOutput_int_register_11, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register12"] = std::bind(&RobotState::getOutput_int_register_12, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register13"] = std::bind(&RobotState::getOutput_int_register_13, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register14"] = std::bind(&RobotState::getOutput_int_register_14, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register15"] = std::bind(&RobotState::getOutput_int_register_15, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register16"] = std::bind(&RobotState::getOutput_int_register_16, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register17"] = std::bind(&RobotState::getOutput_int_register_17, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register18"] = std::bind(&RobotState::getOutput_int_register_18, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register19"] = std::bind(&RobotState::getOutput_int_register_19, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register20"] = std::bind(&RobotState::getOutput_int_register_20, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register21"] = std::bind(&RobotState::getOutput_int_register_21, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register22"] = std::bind(&RobotState::getOutput_int_register_22, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register23"] = std::bind(&RobotState::getOutput_int_register_23, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register24"] = std::bind(&RobotState::getOutput_int_register_24, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register25"] = std::bind(&RobotState::getOutput_int_register_25, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register26"] = std::bind(&RobotState::getOutput_int_register_26, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register27"] = std::bind(&RobotState::getOutput_int_register_27, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register28"] = std::bind(&RobotState::getOutput_int_register_28, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register29"] = std::bind(&RobotState::getOutput_int_register_29, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register30"] = std::bind(&RobotState::getOutput_int_register_30, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register31"] = std::bind(&RobotState::getOutput_int_register_31, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register32"] = std::bind(&RobotState::getOutput_int_register_32, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register33"] = std::bind(&RobotState::getOutput_int_register_33, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register34"] = std::bind(&RobotState::getOutput_int_register_34, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register35"] = std::bind(&RobotState::getOutput_int_register_35, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register36"] = std::bind(&RobotState::getOutput_int_register_36, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register37"] = std::bind(&RobotState::getOutput_int_register_37, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register38"] = std::bind(&RobotState::getOutput_int_register_38, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register39"] = std::bind(&RobotState::getOutput_int_register_39, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register40"] = std::bind(&RobotState::getOutput_int_register_40, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register41"] = std::bind(&RobotState::getOutput_int_register_41, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register42"] = std::bind(&RobotState::getOutput_int_register_42, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register43"] = std::bind(&RobotState::getOutput_int_register_43, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register44"] = std::bind(&RobotState::getOutput_int_register_44, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register45"] = std::bind(&RobotState::getOutput_int_register_45, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register46"] = std::bind(&RobotState::getOutput_int_register_46, mp_robot_state);
		m_output_reg_func_map["getOutput_int_register47"] = std::bind(&RobotState::getOutput_int_register_47, mp_robot_state);

		m_output_reg_func_map["getOutput_double_register0"] =
			std::bind(&RobotState::getOutput_double_register_0, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register1"] =
			std::bind(&RobotState::getOutput_double_register_1, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register2"] =
			std::bind(&RobotState::getOutput_double_register_2, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register3"] =
			std::bind(&RobotState::getOutput_double_register_3, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register4"] =
			std::bind(&RobotState::getOutput_double_register_4, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register5"] =
			std::bind(&RobotState::getOutput_double_register_5, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register6"] =
			std::bind(&RobotState::getOutput_double_register_6, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register7"] =
			std::bind(&RobotState::getOutput_double_register_7, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register8"] =
			std::bind(&RobotState::getOutput_double_register_8, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register9"] =
			std::bind(&RobotState::getOutput_double_register_9, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register10"] =
			std::bind(&RobotState::getOutput_double_register_10, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register11"] =
			std::bind(&RobotState::getOutput_double_register_11, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register12"] =
			std::bind(&RobotState::getOutput_double_register_12, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register13"] =
			std::bind(&RobotState::getOutput_double_register_13, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register14"] =
			std::bind(&RobotState::getOutput_double_register_14, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register15"] =
			std::bind(&RobotState::getOutput_double_register_15, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register16"] =
			std::bind(&RobotState::getOutput_double_register_16, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register17"] =
			std::bind(&RobotState::getOutput_double_register_17, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register18"] =
			std::bind(&RobotState::getOutput_double_register_18, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register19"] =
			std::bind(&RobotState::getOutput_double_register_19, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register20"] =
			std::bind(&RobotState::getOutput_double_register_20, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register21"] =
			std::bind(&RobotState::getOutput_double_register_21, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register22"] =
			std::bind(&RobotState::getOutput_double_register_22, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register23"] =
			std::bind(&RobotState::getOutput_double_register_23, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register24"] =
			std::bind(&RobotState::getOutput_double_register_24, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register25"] =
			std::bind(&RobotState::getOutput_double_register_25, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register26"] =
			std::bind(&RobotState::getOutput_double_register_26, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register27"] =
			std::bind(&RobotState::getOutput_double_register_27, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register28"] =
			std::bind(&RobotState::getOutput_double_register_28, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register29"] =
			std::bind(&RobotState::getOutput_double_register_29, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register30"] =
			std::bind(&RobotState::getOutput_double_register_30, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register31"] =
			std::bind(&RobotState::getOutput_double_register_31, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register32"] =
			std::bind(&RobotState::getOutput_double_register_32, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register33"] =
			std::bind(&RobotState::getOutput_double_register_33, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register34"] =
			std::bind(&RobotState::getOutput_double_register_34, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register35"] =
			std::bind(&RobotState::getOutput_double_register_35, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register36"] =
			std::bind(&RobotState::getOutput_double_register_36, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register37"] =
			std::bind(&RobotState::getOutput_double_register_37, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register38"] =
			std::bind(&RobotState::getOutput_double_register_38, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register39"] =
			std::bind(&RobotState::getOutput_double_register_39, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register40"] =
			std::bind(&RobotState::getOutput_double_register_40, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register41"] =
			std::bind(&RobotState::getOutput_double_register_41, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register42"] =
			std::bind(&RobotState::getOutput_double_register_42, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register43"] =
			std::bind(&RobotState::getOutput_double_register_43, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register44"] =
			std::bind(&RobotState::getOutput_double_register_44, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register45"] =
			std::bind(&RobotState::getOutput_double_register_45, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register46"] =
			std::bind(&RobotState::getOutput_double_register_46, mp_robot_state);
		m_output_reg_func_map["getOutput_double_register47"] =
			std::bind(&RobotState::getOutput_double_register_47, mp_robot_state);
	}


	void RTSIControlInterface::sendClearCommand()
	{
		RTSI::RobotCommand clear_cmd;
		clear_cmd.m_type = RTSI::RobotCommand::Type::NO_CMD;
		clear_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		mp_rtsi->send(clear_cmd);
	}


	int RTSIControlInterface::getControlScriptState()
	{
		if (mp_robot_state != nullptr)
		{
			return getOutputIntReg(0);
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	bool RTSIControlInterface::isProtectiveStopped()
	{
		if (mp_robot_state != nullptr)
		{
			std::bitset<32> safety_status_bits(mp_robot_state->getSafety_status_bits());
			return safety_status_bits.test(SafetyStatus::IS_PROTECTIVE_STOPPED);
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	bool RTSIControlInterface::isEmergencyStopped()
	{
		if (mp_robot_state != nullptr)
		{
			std::bitset<32> safety_status_bits(mp_robot_state->getSafety_status_bits());
			return safety_status_bits.test(SafetyStatus::IS_EMERGENCY_STOPPED);
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}

	}

	double RTSIControlInterface::getStepTimeValue()
	{
		if (mp_robot_state != nullptr)
		{
			return getOutputDoubleReg(0);
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	int RTSIControlInterface::getToolContactValue()
	{
		if (mp_robot_state != nullptr)
		{
			return getOutputIntReg(1);
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	std::vector<double> RTSIControlInterface::getTargetWaypointValue()
	{
		if (mp_robot_state != nullptr)
		{
			std::vector<double> target_waypoint = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
												   getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5) };
			return target_waypoint;
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	std::vector<double> RTSIControlInterface::getActualJointPositionsHistoryValue()
	{
		if (mp_robot_state != nullptr)
		{
			std::vector<double> actual_joint_positions_history = { getOutputDoubleReg(0), getOutputDoubleReg(1),
																  getOutputDoubleReg(2), getOutputDoubleReg(3),
																  getOutputDoubleReg(4), getOutputDoubleReg(5) };
			return actual_joint_positions_history;
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	std::vector<double> RTSIControlInterface::getInverseKinematicsValue()
	{
		if (mp_robot_state != nullptr)
		{
			std::vector<double> q = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
									 getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5) };
			return q;
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}

	std::vector<double> RTSIControlInterface::poseTransValue()
	{
		if (mp_robot_state != nullptr)
		{
			std::vector<double> pose = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
										getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5) };
			return pose;
		}
		else
		{
			throw std::logic_error("Please initialize the RobotState, before using it!");
		}
	}


	void RTSIControlInterface::verifyValueIsWithin(const double &value, const double &min, const double &max)
	{
		if (std::isnan(min) || std::isnan(max))
		{
			throw std::invalid_argument("Make sure both min and max are not NaN's");
		}
		else if (std::isnan(value))
		{
			throw std::invalid_argument("The value is considered NaN");
		}
		else if (!(std::isgreaterequal(value, min) && std::islessequal(value, max)))
		{
			std::ostringstream oss;
			oss << "The value is not within [" << min << ";" << max << "]";
			throw std::range_error(oss.str());
		}
	}

	void RTSIControlInterface::receiveCallback()
	{
	  while (!m_stop_thread)
	  {
		// Receive and update the robot state
		try
		{
		  mp_rtsi->receiveData(mp_robot_state);
		  // temporary hack to fix synchronization problems on windows.
	#ifndef _WIN32
		  std::this_thread::sleep_for(std::chrono::microseconds(100));
	#endif
		}
		catch (std::exception &e)
		{
		  std::cerr << "rtsiControlInterface: Could not receive data from robot..." << std::endl;
		  std::cerr << e.what() << std::endl;
		  if (mp_rtsi != nullptr)
		  {
			if (mp_rtsi->isConnected())
				mp_rtsi->disconnect();

			if (!mp_rtsi->isConnected())
			{
			  std::cerr << "rtsiControlInterface: Robot is disconnected, reconnecting..." << std::endl;
			  reconnect();
			}

			if (mp_rtsi->isConnected())
			  std::cout << "rtsiControlInterface: Successfully reconnected!" << std::endl;
			else
			  throw std::runtime_error("Could not recover from losing connection to robot!");
		  }
		}
	  }
	}


	bool RTSIControlInterface::sendCommand(const RTSI::RobotCommand &cmd)
	{
		std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
		try
		{


		
			int runtime_state = mp_robot_state->getRuntime_state();
			if (runtime_state == RuntimeState::STOPPED)
			{
				if (!m_custom_script_running)
				{
					sendClearCommand();
					return false;
				}
			}

			if (isProgramRunning() || m_custom_script || m_custom_script_running || m_use_external_control_cs_cap)
			{
				while (getControlScriptState() != CS_CONTROLLER_READY_FOR_CMD)
				{
					// If robot is in an emergency or protective stop return false
					if (isProtectiveStopped() || isEmergencyStopped())
					{
						sendClearCommand();
						return false;
					}

					// Wait until the controller is ready for a command or timeout
					std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
					auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
					if (duration > CS_GET_READY_TIMEOUT)
					{
						sendClearCommand();
						return false;
					}
				}

				if (cmd.m_type == RTSI::RobotCommand::Type::SERVOJ || cmd.m_type == RTSI::RobotCommand::Type::SERVOL ||
					cmd.m_type == RTSI::RobotCommand::Type::SERVOC || cmd.m_type == RTSI::RobotCommand::Type::SPEEDJ ||
					cmd.m_type == RTSI::RobotCommand::Type::SPEEDL || cmd.m_type == RTSI::RobotCommand::Type::FORCE_MODE ||
					cmd.m_type == RTSI::RobotCommand::Type::WATCHDOG || cmd.m_type == RTSI::RobotCommand::Type::GET_JOINT_TORQUES ||
					cmd.m_type == RTSI::RobotCommand::Type::TOOL_CONTACT || cmd.m_type == RTSI::RobotCommand::Type::GET_STEPTIME ||
					cmd.m_type == RTSI::RobotCommand::Type::GET_ACTUAL_JOINT_POSITIONS_HISTORY)
				{
					// Send command to the controller
					mp_rtsi->send(cmd);

					// We do not wait for 'continuous' / RT commands to finish.

					return true;
				}
				else
				{
					// Send command to the controller
					mp_rtsi->send(cmd);

					if (cmd.m_type != RTSI::RobotCommand::Type::STOP_SCRIPT)
					{
						start_time = std::chrono::high_resolution_clock::now();
						while (getControlScriptState() != CS_CONTROLLER_DONE_WITH_CMD)
						{
							// if the script causes an error, for example because of inverse
							// kinematics calculation failed, then it may be that the script no
							// longer runs an we will never receive the UR_CONTROLLER_DONE_WITH_CMD
							// signal
							if (!isProgramRunning())
							{
								std::cerr << "rtsiControlInterface: rtsi control script is not running!" << std::endl;
								sendClearCommand();
								return false;
							}

							// If robot is in an emergency or protective stop return false
							if (isProtectiveStopped() || isEmergencyStopped())
							{
								sendClearCommand();
								return false;
							}

							// Wait until the controller has finished executing or timeout
							auto current_time = std::chrono::high_resolution_clock::now();
							auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
							if (duration > CS_EXECUTION_TIMEOUT)
							{
								sendClearCommand();
								return false;
							}
							std::this_thread::sleep_for(std::chrono::milliseconds(1));
						}
					}
					else
					{
						if (m_use_external_control_cs_cap)
						{
							// Program is allowed to still be running when using the ExternalControl UR Cap.
							// So we simply wait a bit for the stop script command to go through and clear the cmd register and return.
							std::this_thread::sleep_for(std::chrono::milliseconds(2));
							sendClearCommand();
							return true;
						}
						else
						{
							while (isProgramRunning())
							{
								// If robot is in an emergency or protective stop return false
								if (isProtectiveStopped() || isEmergencyStopped())
								{
									sendClearCommand();
									return false;
								}

								// Wait for program to stop running or timeout
								auto current_time = std::chrono::high_resolution_clock::now();
								auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
								if (duration > CS_EXECUTION_TIMEOUT)
								{
									sendClearCommand();
									return false;
								}
								std::this_thread::sleep_for(std::chrono::milliseconds(1));
							}
							std::this_thread::sleep_for(std::chrono::milliseconds(1));
						}
					}

					// Make controller ready for next command
					sendClearCommand();
					return true;
				}
			}
			else
			{
				std::cerr << "rtsiControlInterface: rtsi control script is not running!" << std::endl;
				sendClearCommand();
				return false;
			}



		}
		catch (const std::exception& e)
		{
			std::cerr << "rtsiControlInterface: Lost connection to robot..." << std::endl;
			std::cerr << e.what() << std::endl;
			if (mp_rtsi != nullptr)
			{
				if (mp_rtsi->isConnected())
					mp_rtsi->disconnect();
			}
		}

		if (!mp_rtsi->isConnected())
		{
			std::cerr << "rtsiControlInterface: Robot is disconnected, reconnecting..." << std::endl;
			reconnect();
			return sendCommand(cmd);
		}
		sendClearCommand();
		return false;

	}




	void RTSIControlInterface::waitForProgramRunning()
	{
		int ms_count = 0;
		int ms_retry_count = 0;
		while (!isProgramRunning())
		{
			// Wait for program to be running
			static const int sleep_ms = 10;
			std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
			ms_count += sleep_ms;
			ms_retry_count += sleep_ms;
			if (ms_retry_count >= 400)
			{
				ms_retry_count = 0;
				if (m_verbose)
					std::cout << "ur_rtsi: Program not running - resending script" << std::endl;
				mp_script_client->sendScript();
			}
			if (ms_count > 5000)
			{
				throw std::logic_error("ur_rtsi: Failed to start control script, before timeout");
			}
		}

	}



	void RTSIControlInterface::stopL(double a)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::STOPL;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_8;
		robot_cmd.m_val.push_back(a);
		sendCommand(robot_cmd);
	}


	void RTSIControlInterface::stopJ(double a)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::STOPJ;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_8;
		robot_cmd.m_val.push_back(a);
		sendCommand(robot_cmd);
	}


	bool RTSIControlInterface::moveJ(const std::vector<double> &q, double speed, double acceleration, bool async)
	{
		verifyValueIsWithin(speed, CS_JOINT_VELOCITY_MIN, CS_JOINT_VELOCITY_MAX);
		verifyValueIsWithin(acceleration, CS_JOINT_ACCELERATION_MIN, CS_JOINT_ACCELERATION_MAX);

		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::MOVEJ;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_1;
		if (async)
			robot_cmd.m_async = 1;
		else
			robot_cmd.m_async = 0;
		robot_cmd.m_val = q;
		robot_cmd.m_val.push_back(speed);
		robot_cmd.m_val.push_back(acceleration);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::moveL(const std::vector<double> &transform, double speed, double acceleration, bool async)
	{
		verifyValueIsWithin(speed, CS_TOOL_VELOCITY_MIN, CS_TOOL_VELOCITY_MAX);
		verifyValueIsWithin(acceleration, CS_TOOL_ACCELERATION_MIN, CS_TOOL_ACCELERATION_MAX);

		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::MOVEL;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_1;
		if (async)
			robot_cmd.m_async = 1;
		else
			robot_cmd.m_async = 0;
		robot_cmd.m_val = transform;
		robot_cmd.m_val.push_back(speed);
		robot_cmd.m_val.push_back(acceleration);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::speedJ(const std::vector<double> &qd, double acceleration, double time)
	{
		verifyValueIsWithin(acceleration, CS_JOINT_ACCELERATION_MIN, CS_JOINT_ACCELERATION_MAX);

		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SPEEDJ;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_13;
		robot_cmd.m_val = qd;
		robot_cmd.m_val.push_back(acceleration);
		robot_cmd.m_val.push_back(time);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::speedL(const std::vector<double> &xd, double acceleration, double time)
	{
		verifyValueIsWithin(acceleration, CS_TOOL_ACCELERATION_MIN, CS_TOOL_ACCELERATION_MAX);

		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SPEEDL;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_13;
		robot_cmd.m_val = xd;
		robot_cmd.m_val.push_back(acceleration);
		robot_cmd.m_val.push_back(time);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::servoJ(const std::vector<double> &q, double speed, double acceleration, double time,
		double lookahead_time, double gain)
	{
		verifyValueIsWithin(speed, CS_JOINT_VELOCITY_MIN, CS_JOINT_VELOCITY_MAX);
		verifyValueIsWithin(acceleration, CS_JOINT_ACCELERATION_MIN, CS_JOINT_ACCELERATION_MAX);
		verifyValueIsWithin(lookahead_time, CS_SERVO_LOOKAHEAD_TIME_MIN, CS_SERVO_LOOKAHEAD_TIME_MAX);
		verifyValueIsWithin(gain, CS_SERVO_GAIN_MIN, CS_SERVO_GAIN_MAX);

		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SERVOJ;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_2;
		robot_cmd.m_val = q;
		robot_cmd.m_val.push_back(speed);
		robot_cmd.m_val.push_back(acceleration);
		robot_cmd.m_val.push_back(time);
		robot_cmd.m_val.push_back(lookahead_time);
		robot_cmd.m_val.push_back(gain);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::servoL(const std::vector<double>& pose, double speed, double acceleration, double time,
		double lookahead_time, double gain)
	{
		verifyValueIsWithin(speed, CS_JOINT_VELOCITY_MIN, CS_JOINT_VELOCITY_MAX);
		verifyValueIsWithin(acceleration, CS_JOINT_ACCELERATION_MIN, CS_JOINT_ACCELERATION_MAX);
		verifyValueIsWithin(lookahead_time, CS_SERVO_LOOKAHEAD_TIME_MIN, CS_SERVO_LOOKAHEAD_TIME_MAX);
		verifyValueIsWithin(gain, CS_SERVO_GAIN_MIN, CS_SERVO_GAIN_MAX);

		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SERVOL;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_2;
		robot_cmd.m_val = pose;
		robot_cmd.m_val.push_back(speed);
		robot_cmd.m_val.push_back(acceleration);
		robot_cmd.m_val.push_back(time);
		robot_cmd.m_val.push_back(lookahead_time);
		robot_cmd.m_val.push_back(gain);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::jogStart(const std::vector<double> &speeds, int feature)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::JOG_START;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_14;
		robot_cmd.m_val = speeds;
		robot_cmd.m_val.push_back(feature);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::jogStop()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::JOG_STOP;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		return sendCommand(robot_cmd);
	}


	bool RTSIControlInterface::speedStop(double a)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SPEED_STOP;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_8;
		robot_cmd.m_val.push_back(a);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::servoStop(double a)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SERVO_STOP;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_8;
		robot_cmd.m_val.push_back(a);
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::forceMode(const std::vector<double> &task_frame, const std::vector<int> &selection_vector,
		const std::vector<double> &wrench, int type, const std::vector<double> &limits)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::FORCE_MODE;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_3;
		robot_cmd.m_val = task_frame;
		for (const auto &val : wrench)
			robot_cmd.m_val.push_back(val);

		for (const auto &val : limits)
			robot_cmd.m_val.push_back(val);

		robot_cmd.m_selection_vector = selection_vector;
		robot_cmd.m_force_mode_type = type;
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::forceModeStop()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::FORCE_MODE_STOP;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		return sendCommand(robot_cmd);
	}

	bool RTSIControlInterface::zeroFtSensor()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::ZERO_FT_SENSOR;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		return sendCommand(robot_cmd);
	}



	bool RTSIControlInterface::setPayload(double mass, const std::vector<double> &cog)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SET_PAYLOAD;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_7;
		robot_cmd.m_val.push_back(mass);
		if (!cog.empty())
		{
			for (const auto &val : cog)
				robot_cmd.m_val.push_back(val);
		}
		else
		{
			robot_cmd.m_val.push_back(0);
			robot_cmd.m_val.push_back(0);
			robot_cmd.m_val.push_back(0);
		}
		return sendCommand(robot_cmd);
	}

	double RTSIControlInterface::getStepTime()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_STEPTIME;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		if (sendCommand(robot_cmd))
		{
			return getStepTimeValue();
		}
		else
		{
			return 0;
		}
	}
	std::vector<double> RTSIControlInterface::getTargetWaypoint()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_TARGET_WAYPOINT;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		if (sendCommand(robot_cmd))
		{
			return getTargetWaypointValue();
		}
		else
		{
			return std::vector<double>();
		}
	}

	bool RTSIControlInterface::setTcp(const std::vector<double> &tcp_offset)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SET_TCP;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_6;
		robot_cmd.m_val = tcp_offset;
		return sendCommand(robot_cmd);
	}


	std::vector<double> RTSIControlInterface::getForwardKinematics(const std::vector<double> &q,
		const std::vector<double> &tcp_offset)
	{
		RTSI::RobotCommand robot_cmd;
		if (q.empty() && tcp_offset.empty())
		{
			robot_cmd.m_type = RTSI::RobotCommand::Type::GET_FORWARD_KINEMATICS_DEFAULT;
			robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;
		}
		else if (tcp_offset.empty() && !q.empty())
		{
			robot_cmd.m_type = RTSI::RobotCommand::Type::GET_FORWARD_KINEMATICS_ARGS;
			robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_6;
			robot_cmd.m_val = q;
		}
		else
		{
			robot_cmd.m_type = RTSI::RobotCommand::Type::GET_FORWARD_KINEMATICS_ARGS;
			robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_12;
			robot_cmd.m_val = q;
			robot_cmd.m_val.insert(robot_cmd.m_val.end(), tcp_offset.begin(), tcp_offset.end());
		}

		if (sendCommand(robot_cmd))
		{
			if (mp_robot_state != nullptr)
			{
				std::vector<double> forward_kin = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
												   getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5) };
				return forward_kin;
			}
			else
			{
				throw std::logic_error("Please initialize the RobotState, before using it!");
			}
		}
		else
		{
			return std::vector<double>();
		}
	}


	std::vector<double> RTSIControlInterface::getInverseKinematics(const std::vector<double> &x,
		const std::vector<double> &qnear,
		double max_position_error, double max_orientation_error)
	{
		RTSI::RobotCommand robot_cmd;
		if (!qnear.empty())
		{
			robot_cmd.m_type = RTSI::RobotCommand::Type::GET_INVERSE_KINEMATICS_ARGS;
			robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_10;
			robot_cmd.m_val = x;
			robot_cmd.m_val.insert(robot_cmd.m_val.end(), qnear.begin(), qnear.end());
			//robot_cmd.m_val.push_back(max_position_error);
			//robot_cmd.m_val.push_back(max_orientation_error);
		}
		else
		{
			robot_cmd.m_type = RTSI::RobotCommand::Type::GET_INVERSE_KINEMATICS_DEFAULT;
			robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_6;
			robot_cmd.m_val = x;
		}

		if (sendCommand(robot_cmd))
		{
			return getInverseKinematicsValue();
		}
		else
		{
			return std::vector<double>();
		}
	}

	std::vector<double> RTSIControlInterface::poseTrans(const std::vector<double> &p_from,
		const std::vector<double> &p_from_to)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::POSE_TRANS;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_12;
		robot_cmd.m_val = p_from;
		robot_cmd.m_val.insert(robot_cmd.m_val.end(), p_from_to.begin(), p_from_to.end());
		if (sendCommand(robot_cmd))
		{
			return poseTransValue();
		}
		else
		{
			return std::vector<double>();
		}
	}

	bool RTSIControlInterface::setWatchdog(double min_frequency)
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::SET_WATCHDOG;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_8;
		robot_cmd.m_val.push_back(min_frequency);
		return sendCommand(robot_cmd);
	}


	std::vector<double> RTSIControlInterface::getTCPOffset()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_TCP_OFFSET;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;

		if (sendCommand(robot_cmd))
		{
			if (mp_robot_state != nullptr)
			{
				std::vector<double> tcp_offset = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
												  getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5) };
				return tcp_offset;
			}
			else
			{
				throw std::logic_error("Please initialize the RobotState, before using it!");
			}
		}
		else
		{
			return std::vector<double>();
		}
	}

	std::vector<double> RTSIControlInterface::getJointTorques()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_JOINT_TORQUES;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;

		if (sendCommand(robot_cmd))
		{
			if (mp_robot_state != nullptr)
			{
				std::vector<double> torques = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
											   getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5) };
				return torques;
			}
			else
			{
				throw std::logic_error("Please initialize the RobotState, before using it!");
			}
		}
		else
		{
			return std::vector<double>();
		}
	}

	std::vector<double> RTSIControlInterface::getSensorForce()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_SENSOR_FORCE;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;

		if (sendCommand(robot_cmd))
		{
			if (mp_robot_state != nullptr)
			{
				std::vector<double> sensor_force = { getOutputDoubleReg(24), getOutputDoubleReg(25), getOutputDoubleReg(26),
												  getOutputDoubleReg(27), getOutputDoubleReg(28), getOutputDoubleReg(29) };
				return sensor_force;
			}
			else
			{
				throw std::logic_error("Please initialize the RobotState, before using it!");
			}
		}
		else
		{
			return std::vector<double>();
		}
	}

	std::vector<double> RTSIControlInterface::getTCPForce()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_TCP_FORCE;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;

		if (sendCommand(robot_cmd))
		{
			if (mp_robot_state != nullptr)
			{
				std::vector<double> tcp_force = { getOutputDoubleReg(24), getOutputDoubleReg(25), getOutputDoubleReg(26),
												  getOutputDoubleReg(27), getOutputDoubleReg(28), getOutputDoubleReg(29) };
				return tcp_force;
			}
			else
			{
				throw std::logic_error("Please initialize the RobotState, before using it!");
			}
		}
		else
		{
			return std::vector<double>();
		}
	}

	std::vector<double> RTSIControlInterface::getToolPayload()
	{
		RTSI::RobotCommand robot_cmd;
		robot_cmd.m_type = RTSI::RobotCommand::Type::GET_TOOL_PAYLOAD;
		robot_cmd.m_recipe_id = RTSI::RobotCommand::Recipe::RECIPE_4;

		if (sendCommand(robot_cmd))
		{
			if (mp_robot_state != nullptr)
			{
				std::vector<double> tcp_offset = { getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
												  getOutputDoubleReg(3) };
				return tcp_offset;
			}
			else
			{
				throw std::logic_error("Please initialize the RobotState, before using it!");
			}
		}
		else
		{
			return std::vector<double>();
		}
	}



}