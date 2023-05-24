#include "dashboard_client.h"
#include "robot_state.h"
#include "rtsi.h"
#include "rtsi_receive_interface.h"

#include <bitset>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <iostream>
#include <thread>


namespace cs_rtsi
{
	RTSIReceiveInterface::RTSIReceiveInterface(std::string hostname,
		std::vector<std::string> variables, bool verbose,
		bool use_upper_range_registers)
		: m_variables(std::move(variables)),
		m_hostname(std::move(hostname)),
		m_verbose(verbose),
		m_use_upper_range_registers(use_upper_range_registers)
	{
		m_port = 30004;
		mp_rtsi = std::make_shared<RTSI>(m_hostname, m_port, m_verbose);
		mp_rtsi->connect();
		mp_rtsi->negotiateProtocolVersion();
		auto controller_version = mp_rtsi->getControllerVersion();
		uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

		m_frequency = 250;

		// If e-Series Robot set frequency to 500Hz

		//if (major_version > CB3_MAJOR_VERSION)
		//	m_frequency = 500;

		// Set delta time to be used by receiveCallback
		m_delta_time = 1 / m_frequency;

		// Init Robot state
		mp_robot_state = std::make_shared<RobotState>();

		// Map the output registers to functions
		initOutputRegFuncMap();

		// Init pausing state
		m_pausing_state = PausingState::RUNNING;
		m_pausing_ramp_up_increment = 0.01;

		if (m_use_upper_range_registers)
			m_register_offset = 24;
		else
			m_register_offset = 0;

		// Setup recipes
		setupRecipes(m_frequency);

		// Start rtsi data synchronization
		mp_rtsi->sendStart();

		// Start executing receiveCallback
		mp_th = std::make_shared<boost::thread>(boost::bind(&RTSIReceiveInterface::receiveCallback, this));

		// Wait until the first robot state has been received
		std::this_thread::sleep_for(std::chrono::milliseconds(10));


	}

	RTSIReceiveInterface::~RTSIReceiveInterface()
	{
		disconnect();
	}

	void RTSIReceiveInterface::disconnect()
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

		// Wait until everything has disconnected
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	bool RTSIReceiveInterface::setupRecipes(const double& frequency)
	{
		if (m_variables.empty())
		{
			// Assume all variables
			m_variables = { "payload_cog",
							"payload_mass",
							//"script_control_line",
							"timestamp",
							"target_joint_positions",
							"target_joint_speeds",
							"actual_joint_torques",
							"actual_joint_positions",
							"actual_joint_speeds",
							"actual_joint_current",
							"actual_joint_positions",
							"actual_TCP_pose",
							"actual_TCP_speed",
							"target_TCP_pose",
							"target_TCP_speed",
			                "ft_raw_wrench",
				            "actual_TCP_force",
							"actual_digital_input_bits",
							"joint_temperatures",
							"robot_mode",
							"joint_mode",
							"safety_mode",
							"safety_status",
							"speed_scaling",
							"target_speed_fraction",
							"actual_robot_voltage",
							"actual_robot_current",
							//"actual_joint_voltage",
							"actual_digital_output_bits",
							"runtime_state",
							//"elbow_position",
							"robot_status_bits",
							"safety_status_bits",
							//"analog_io_types",
							"standard_analog_input0",
							"standard_analog_input1",
							"standard_analog_output0",
							"standard_analog_output1",
							//"io_current",
							//"tool_mode",
							//"tool_analog_input_types",
							//"tool_analog_output_types",
							//"tool_analog_input",
							//"tool_analog_output",
							//"tool_output_voltage",
							//"tool_output_current",
							//"tool_temperature",
							  outIntReg(2),
							  outIntReg(12),
							  outIntReg(13),
							  outIntReg(14),
							  outIntReg(15),
							  outIntReg(16),
							  outIntReg(17),
							  outIntReg(18),
							  outIntReg(19),
							  outDoubleReg(12),
							  outDoubleReg(13),
							  outDoubleReg(14),
							  outDoubleReg(15),
							  outDoubleReg(16),
							  outDoubleReg(17),
							  outDoubleReg(18),
							  outDoubleReg(19) };
		}

		// Setup output
		mp_rtsi->sendOutputSetup(m_variables, frequency);
		return true;
	}

	void RTSIReceiveInterface::initOutputRegFuncMap()
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
		m_output_reg_func_map["getOutput_double_register_15"] =
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

	void RTSIReceiveInterface::receiveCallback()
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
			catch (std::exception& e)
			{
				std::cerr << e.what() << std::endl;
				if (mp_rtsi->isConnected())
					mp_rtsi->disconnect();
				m_stop_thread = true;
			}
		}
	}


	bool RTSIReceiveInterface::reconnect()
	{
		if (mp_rtsi != nullptr)
		{
			mp_rtsi->connect();
			mp_rtsi->negotiateProtocolVersion();
			auto controller_version = mp_rtsi->getControllerVersion();
			uint32_t major_version = std::get<MAJOR_VERSION>(controller_version);

			m_frequency = 250;
			// If e-Series Robot set frequency to 500Hz
			if (major_version > CB3_MAJOR_VERSION)
				m_frequency = 500;

			// Set delta time to be used by receiveCallback
			m_delta_time = 1 / m_frequency;

			// Init Robot state
			mp_robot_state = std::make_shared<RobotState>();

			// Map the output registers to functions
			initOutputRegFuncMap();

			// Setup recipes
			setupRecipes(m_frequency);

			// Start rtsi data synchronization
			mp_rtsi->sendStart();

			m_stop_thread = false;

			// Start executing receiveCallback
			mp_th = std::make_shared<boost::thread>(boost::bind(&RTSIReceiveInterface::receiveCallback, this));

			// Wait until the first robot state has been received
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		return RTSIReceiveInterface::isConnected();
	}

	bool RTSIReceiveInterface::isConnected()
	{
		return mp_rtsi->isConnected();
	}


	int RTSIReceiveInterface::getConState()
	{
		return RTSI::sm_state;
	}

	double RTSIReceiveInterface::getPayloadMass()
	{
		return mp_robot_state->getPayload_mass();
	}

	std::vector<double> RTSIReceiveInterface::getPayloadCog()
	{
		return mp_robot_state->getPayload_cog();
	}

	double RTSIReceiveInterface::getTimestamp()
	{
		return mp_robot_state->getTimestamp();
	}

	std::vector<double> RTSIReceiveInterface::getTargetQ()
	{
		return mp_robot_state->getTarget_q();
	}

	std::vector<double> RTSIReceiveInterface::getTargetQd()
	{
		return mp_robot_state->getTarget_qd();
	}

	std::vector<double> RTSIReceiveInterface::getActualMoment()
	{
		return mp_robot_state->getActual_joint_torques();
	}

	std::vector<double> RTSIReceiveInterface::getActualQ()
	{

		return mp_robot_state->getActual_q();
	}

	std::vector<double> RTSIReceiveInterface::getActualQd()
	{

		return mp_robot_state->getActual_qd();
	}


	std::vector<double> RTSIReceiveInterface::getActualCurrent()
	{
		return mp_robot_state->getActual_current();
	}

	std::vector<double> RTSIReceiveInterface::getActualTCPPose()
	{
		return mp_robot_state->getActual_TCP_pose();
	}

	std::vector<double> RTSIReceiveInterface::getActualTCPSpeed()
	{
		return mp_robot_state->getActual_TCP_speed();
	}

	std::vector<double> RTSIReceiveInterface::getTargetTCPPose()
	{
		return mp_robot_state->getTarget_TCP_pose();
	}


	std::vector<double> RTSIReceiveInterface::getTargetTCPSpeed()
	{
		return mp_robot_state->getTarget_TCP_speed();
	}

	std::vector<double> RTSIReceiveInterface::getActualTCPForce()
	{
		return mp_robot_state->getActual_TCP_force();
	}

	std::vector<double> RTSIReceiveInterface::getActualSensorForce()
	{
		return mp_robot_state->getActual_sensor_force();
	}

	uint32_t RTSIReceiveInterface::getActualDigitalInputBits()
	{
		return mp_robot_state->getActual_digital_input_bits();
	}

	std::vector<double> RTSIReceiveInterface::getJointTemperatures()
	{
		return mp_robot_state->getJoint_temperatures();
	}

	int32_t RTSIReceiveInterface::getRobotMode()
	{
		return mp_robot_state->getRobot_mode();
	}

	std::vector<int32_t> RTSIReceiveInterface::getJointMode()
	{
		return mp_robot_state->getJoint_mode();
	}

	int32_t RTSIReceiveInterface::getSafetyMode()
	{
		return mp_robot_state->getSafety_mode();
	}

	int32_t RTSIReceiveInterface::getSafetyStatus()
	{
		return mp_robot_state->getSafety_status();
	}

	double RTSIReceiveInterface::getSpeedScaling()
	{
		return mp_robot_state->getSpeed_scaling();
	}

	double RTSIReceiveInterface::getTargetSpeedFraction()
	{
		return mp_robot_state->getTarget_speed_fraction();
	}

	double RTSIReceiveInterface::getActualRobotVoltage()
	{
		return mp_robot_state->getActual_robot_voltage();
	}

	double RTSIReceiveInterface::getActualRobotCurrent()
	{
		return mp_robot_state->getActual_robot_current();
	}

	uint32_t RTSIReceiveInterface::getActualDigitalOutputBits()
	{
		return mp_robot_state->getActual_digital_output_bits();
	}

	uint32_t RTSIReceiveInterface::getRuntimeState()
	{
		return mp_robot_state->getRuntime_state();
	}
	std::vector<double> RTSIReceiveInterface::getElbowPosition()
	{
		return mp_robot_state->getElbow_position();
	}

	uint32_t RTSIReceiveInterface::getRobotStatusBits()
	{
		return mp_robot_state->getRobot_status_bits();
	}

	std::uint32_t RTSIReceiveInterface::getSafetyStatusBits()
	{
		return mp_robot_state->getSafety_status_bits();
	}

	double RTSIReceiveInterface::getStandardAnalogInput0()
	{
		return mp_robot_state->getStandard_analog_input_0();
	}

	double RTSIReceiveInterface::getStandardAnalogInput1()
	{
		return mp_robot_state->getStandard_analog_input_1();
	}

	double RTSIReceiveInterface::getStandardAnalogOutput0()
	{
		return mp_robot_state->getStandard_analog_output_0();
	}

	double RTSIReceiveInterface::getStandardAnalogOutput1()
	{
		return mp_robot_state->getStandard_analog_output_1();
	}

	int RTSIReceiveInterface::getOutputIntRegister(int output_id)
	{
		if (m_use_upper_range_registers)
		{
			if (!isWithinBounds(output_id, 36, 43))
			{
				throw std::range_error(
					"The supported range of getOutputIntRegister() is [36-43], when using upper range, you specified: " +
					std::to_string(output_id));
			}
		}
		else
		{
			if (!isWithinBounds(output_id, 12, 19))
			{
				throw std::range_error(
					"The supported range of getOutputIntRegister() is [12-19], when using lower range you specified: " +
					std::to_string(output_id));
			}
		}

		return getOutputIntReg(output_id);
	}

	double RTSIReceiveInterface::getOutputDoubleRegister(int output_id)
	{
		if (m_use_upper_range_registers)
		{
			if (!isWithinBounds(output_id, 36, 43))
			{
				throw std::range_error(
					"The supported range of getOutputDoubleRegister() is [36-43], when using upper range, you specified: " +
					std::to_string(output_id));
			}
		}
		else
		{
			if (!isWithinBounds(output_id, 12, 19))
			{
				throw std::range_error(
					"The supported range of getOutputDoubleRegister() is [12-19], when using lower range you specified: " +
					std::to_string(output_id));
			}
		}

		return getOutputDoubleReg(output_id);
	}



}
