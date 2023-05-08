#include "robot_state.h"


namespace cs_rtsi
{
	RobotState::RobotState()
	{

	}

	RobotState::~RobotState()
	{
	}

	bool RobotState::lockUpdateStateMutex()
	{
		m_update_state_mutex.lock();
		return true;
	}

	bool RobotState::unlockUpdateStateMutex()
	{
		m_update_state_mutex.unlock();
		return true;
	}

	double RobotState::getPayload_mass()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_payload_mass;

	}
	void RobotState::setPayload_mass(double payload_mass)
	{
		RobotState::m_payload_mass = payload_mass;
	}

	const std::vector<double> RobotState::getPayload_cog()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_payload_cog;
	}
	void RobotState::setPayload_cog(const std::vector<double> &payload_cog)
	{
		RobotState::m_payload_cog = payload_cog;
	}

	double RobotState::getTimestamp()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_timestamp;
	}
	void RobotState::setTimestamp(double timestamp)
	{
		RobotState::m_timestamp = timestamp;
	}

	const std::vector<double> RobotState::getTarget_q()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_target_q;
	}
	void RobotState::setTarget_q(const std::vector<double> &target_q)
	{
		RobotState::m_target_q = target_q;
	}

	const std::vector<double> RobotState::getTarget_qd()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_target_qd;
	}
	void RobotState::setTarget_qd(const std::vector<double> &target_qd)
	{
		RobotState::m_target_qd = target_qd;
	}

	const std::vector<double> RobotState::getActual_q()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_q;
	}
	void RobotState::setActual_q(const std::vector<double> &actual_q)
	{
		RobotState::m_actual_q = actual_q;
	}

	const std::vector<double> RobotState::getActual_qd()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_qd;
	}
	void RobotState::setActual_qd(const std::vector<double> &actual_qd)
	{
		RobotState::m_actual_qd = actual_qd;
	}

	const std::vector<double> RobotState::getActual_moment()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_moment;
	}
	void RobotState::setActual_moment(const std::vector<double> &actual_moment)
	{
		RobotState::m_actual_moment = actual_moment;
	}


	const std::vector<double> RobotState::getActual_current()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_current;
	}

	void RobotState::setActual_current(const std::vector<double> &actual_current)
	{
		RobotState::m_actual_current = actual_current;
	}

	const std::vector<double> RobotState::getActual_TCP_pose()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_TCP_pose;
	}
	void RobotState::setActual_TCP_pose(const std::vector<double> &actual_TCP_pose)
	{
		RobotState::m_actual_TCP_pose = actual_TCP_pose;
	}

	const std::vector<double> RobotState::getActual_TCP_speed()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_TCP_speed;
	}
	void RobotState::setActual_TCP_speed(const std::vector<double> &actual_TCP_speed)
	{
		RobotState::m_actual_TCP_speed = actual_TCP_speed;
	}

	const std::vector<double> RobotState::getActual_TCP_force()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_TCP_force;

	}
	void RobotState::setActual_TCP_force(const std::vector<double> &actual_TCP_force)
	{
		RobotState::m_actual_TCP_force = actual_TCP_force;
	}

	const std::vector<double> RobotState::getActual_sensor_force()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_sensor_force;

	}
	void RobotState::setActual_sensor_force(const std::vector<double> &actual_sensor_force)
	{
		RobotState::m_actual_sensor_force = actual_sensor_force;
	}

	const std::vector<double> RobotState::getTarget_TCP_pose()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_target_TCP_pose;
	}
	void RobotState::setTarget_TCP_pose(const std::vector<double> &target_TCP_pose)
	{
		RobotState::m_target_TCP_pose = target_TCP_pose;
	}

	const std::vector<double> RobotState::getTarget_TCP_speed()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_target_TCP_speed;
	}
	void RobotState::setTarget_TCP_speed(const std::vector<double> &target_TCP_speed)
	{
		RobotState::m_target_TCP_speed = target_TCP_speed;
	}

	uint32_t RobotState::getActual_digital_input_bits()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_digital_input_bits;
	}
	void RobotState::setActual_digital_input_bits(uint32_t actual_digital_input_bits)
	{
		RobotState::m_actual_digital_input_bits = actual_digital_input_bits;
	}

	const std::vector<double> RobotState::getJoint_temperatures()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_joint_temperatures;
	}
	void RobotState::setJoint_temperatures(const std::vector<double> &joint_temperatures)
	{
		RobotState::m_joint_temperatures = joint_temperatures;
	}


	int32_t RobotState::getRobot_mode()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_robot_mode;
	}
	void RobotState::setRobot_mode(int32_t robot_mode)
	{
		RobotState::m_robot_mode = robot_mode;
	}

	uint32_t RobotState::getRobot_status_bits()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_robot_status_bits;
	}
	void RobotState::setRobot_status_bits(uint32_t robot_status)
	{
		RobotState::m_robot_status_bits = robot_status;
	}

	const std::vector<int32_t> RobotState::getJoint_mode()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_joint_mode;
	}
	void RobotState::setJoint_mode(const std::vector<int32_t> &joint_mode)
	{
		RobotState::m_joint_mode = joint_mode;
	}

	int32_t RobotState::getSafety_mode()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_safety_mode;
	}
	void RobotState::setSafety_mode(int32_t safety_mode)
	{
		RobotState::m_safety_mode = safety_mode;
	}

	int32_t RobotState::getSafety_status()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_safety_status;


	}
	void RobotState::setSafety_status(int32_t safety_status)
	{
		RobotState::m_safety_status = safety_status;

	}



	uint32_t RobotState::getSafety_status_bits()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_safety_status_bits;
	}
	void RobotState::setSafety_status_bits(uint32_t safety_status_bits)
	{
		RobotState::m_safety_status_bits = safety_status_bits;
	}

	double RobotState::getSpeed_scaling()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_speed_scaling;
	}
	void RobotState::setSpeed_scaling(double speed_scaling)
	{
		RobotState::m_speed_scaling = speed_scaling;
	}

	double RobotState::getTarget_speed_fraction()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_target_speed_fraction;
	}
	void RobotState::setTarget_speed_fraction(double target_speed_fraction)
	{
		RobotState::m_target_speed_fraction = target_speed_fraction;
	}

	double RobotState::getActual_robot_voltage()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_robot_voltage;
	}
	void RobotState::setActual_robot_voltage(double actual_robot_voltage)
	{
		RobotState::m_actual_robot_voltage = actual_robot_voltage;
	}

	double RobotState::getActual_robot_current()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_robot_current;
	}
	void RobotState::setActual_robot_current(double actual_robot_current)
	{
		RobotState::m_actual_robot_current = actual_robot_current;
	}

	const std::vector<double> RobotState::getActual_joint_current()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_joint_current;

	}
	void RobotState::setActual_joint_current(const std::vector<double> &actual_joint_current)
	{
		RobotState::m_actual_joint_current = actual_joint_current;

	}

	const std::vector<double> RobotState::getActual_joint_torques()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_joint_torques;
	}
	void RobotState::setActual_joint_torques(const std::vector<double> &actual_joint_torques)
	{
		RobotState::m_actual_joint_torques = actual_joint_torques;

	}





	uint32_t RobotState::getActual_digital_output_bits()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_actual_digital_output_bits;
	}
	void RobotState::setActual_digital_output_bits(uint32_t actual_digital_output_bits)
	{
		RobotState::m_actual_digital_output_bits = actual_digital_output_bits;
	}

	uint32_t RobotState::getRuntime_state()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_runtime_state;
	}
	void RobotState::setRuntime_state(uint32_t runtime_state)
	{
		RobotState::m_runtime_state = runtime_state;
	}

	const std::vector<double> RobotState::getElbow_position()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_elbow_position;
	}

	void RobotState::setElbow_position(const std::vector<double> &elbow_position)
	{
		RobotState::m_elbow_position = elbow_position;
	}


	double RobotState::getStandard_analog_input_0()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_standard_analog_input_0;
	}
	void RobotState::setStandard_analog_input_0(double standard_analog_input_0)
	{
		RobotState::m_standard_analog_input_0 = standard_analog_input_0;
	}
	double RobotState::getStandard_analog_input_1()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_standard_analog_input_1;
	}
	void RobotState::setStandard_analog_input_1(double standard_analog_input_1)
	{
		RobotState::m_standard_analog_input_1 = standard_analog_input_1;
	}
	double RobotState::getStandard_analog_output_0()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_standard_analog_output_0;
	}
	void RobotState::setStandard_analog_output_0(double standard_analog_output_0)
	{
		RobotState::m_standard_analog_output_0 = standard_analog_output_0;
	}
	double RobotState::getStandard_analog_output_1()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_standard_analog_output_1;
	}
	void RobotState::setStandard_analog_output_1(double standard_analog_output_1)
	{
		RobotState::m_standard_analog_output_1 = standard_analog_output_1;
	}


	uint32_t RobotState::getOutput_bit_registers0_to_31()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_bit_registers0_to_31;
	}
	void RobotState::setOutput_bit_registers0_to_31(uint32_t output_bit_registers0_to_31)
	{
		RobotState::m_output_bit_registers0_to_31 = output_bit_registers0_to_31;
	}
	uint32_t RobotState::getOutput_bit_registers32_to_63()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_bit_registers32_to_63;
	}
	void RobotState::setOutput_bit_registers32_to_63(uint32_t output_bit_registers32_to_63)
	{
		RobotState::m_output_bit_registers32_to_63 = output_bit_registers32_to_63;
	}


	int32_t RobotState::getOutput_int_register_0()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_0;
	}
	void RobotState::setOutput_int_register_0(int32_t output_int_register_0)
	{
		RobotState::m_output_int_register_0 = output_int_register_0;
	}
	int32_t RobotState::getOutput_int_register_1()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_1;
	}
	void RobotState::setOutput_int_register_1(int32_t output_int_register_1)
	{
		RobotState::m_output_int_register_1 = output_int_register_1;
	}
	int32_t RobotState::getOutput_int_register_2()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_2;
	}
	void RobotState::setOutput_int_register_2(int32_t output_int_register_2)
	{
		RobotState::m_output_int_register_2 = output_int_register_2;
	}
	int32_t RobotState::getOutput_int_register_3()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_3;
	}
	void RobotState::setOutput_int_register_3(int32_t output_int_register_3)
	{
		RobotState::m_output_int_register_3 = output_int_register_3;
	}
	int32_t RobotState::getOutput_int_register_4()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_4;
	}
	void RobotState::setOutput_int_register_4(int32_t output_int_register_4)
	{
		RobotState::m_output_int_register_4 = output_int_register_4;
	}
	int32_t RobotState::getOutput_int_register_5()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_5;
	}
	void RobotState::setOutput_int_register_5(int32_t output_int_register_5)
	{
		RobotState::m_output_int_register_5 = output_int_register_5;
	}
	int32_t RobotState::getOutput_int_register_6()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_6;
	}
	void RobotState::setOutput_int_register_6(int32_t output_int_register_6)
	{
		RobotState::m_output_int_register_6 = output_int_register_6;
	}
	int32_t RobotState::getOutput_int_register_7()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_7;
	}
	void RobotState::setOutput_int_register_7(int32_t output_int_register_7)
	{
		RobotState::m_output_int_register_7 = output_int_register_7;
	}
	int32_t RobotState::getOutput_int_register_8()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_8;
	}
	void RobotState::setOutput_int_register_8(int32_t output_int_register_8)
	{
		RobotState::m_output_int_register_8 = output_int_register_8;
	}
	int32_t RobotState::getOutput_int_register_9()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_9;
	}
	void RobotState::setOutput_int_register_9(int32_t output_int_register_9)
	{
		RobotState::m_output_int_register_9 = output_int_register_9;
	}
	int32_t RobotState::getOutput_int_register_10()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_10;
	}
	void RobotState::setOutput_int_register_10(int32_t output_int_register_10)
	{
		RobotState::m_output_int_register_10 = output_int_register_10;
	}
	int32_t RobotState::getOutput_int_register_11()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_11;
	}
	void RobotState::setOutput_int_register_11(int32_t output_int_register_11)
	{
		RobotState::m_output_int_register_11 = output_int_register_11;
	}
	int32_t RobotState::getOutput_int_register_12()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_12;
	}
	void RobotState::setOutput_int_register_12(int32_t output_int_register_12)
	{
		RobotState::m_output_int_register_12 = output_int_register_12;
	}
	int32_t RobotState::getOutput_int_register_13()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_13;
	}
	void RobotState::setOutput_int_register_13(int32_t output_int_register_13)
	{
		RobotState::m_output_int_register_13 = output_int_register_13;
	}
	int32_t RobotState::getOutput_int_register_14()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_14;
	}
	void RobotState::setOutput_int_register_14(int32_t output_int_register_14)
	{
		RobotState::m_output_int_register_14 = output_int_register_14;
	}
	int32_t RobotState::getOutput_int_register_15()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_15;
	}
	void RobotState::setOutput_int_register_15(int32_t output_int_register_15)
	{
		RobotState::m_output_int_register_15 = output_int_register_15;
	}
	int32_t RobotState::getOutput_int_register_16()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_16;
	}
	void RobotState::setOutput_int_register_16(int32_t output_int_register_16)
	{
		RobotState::m_output_int_register_16 = output_int_register_16;
	}
	int32_t RobotState::getOutput_int_register_17()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_17;
	}
	void RobotState::setOutput_int_register_17(int32_t output_int_register_17)
	{
		RobotState::m_output_int_register_17 = output_int_register_17;
	}
	int32_t RobotState::getOutput_int_register_18()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_18;
	}
	void RobotState::setOutput_int_register_18(int32_t output_int_register_18)
	{
		RobotState::m_output_int_register_18 = output_int_register_18;
	}
	int32_t RobotState::getOutput_int_register_19()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_19;
	}
	void RobotState::setOutput_int_register_19(int32_t output_int_register_19)
	{
		RobotState::m_output_int_register_19 = output_int_register_19;
	}
	int32_t RobotState::getOutput_int_register_20()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_20;
	}
	void RobotState::setOutput_int_register_20(int32_t output_int_register_20)
	{
		RobotState::m_output_int_register_20 = output_int_register_20;
	}
	int32_t RobotState::getOutput_int_register_21()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_21;
	}
	void RobotState::setOutput_int_register_21(int32_t output_int_register_21)
	{
		RobotState::m_output_int_register_21 = output_int_register_21;
	}
	int32_t RobotState::getOutput_int_register_22()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_22;
	}
	void RobotState::setOutput_int_register_22(int32_t output_int_register_22)
	{
		RobotState::m_output_int_register_22 = output_int_register_22;
	}
	int32_t RobotState::getOutput_int_register_23()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_23;
	}
	void RobotState::setOutput_int_register_23(int32_t output_int_register_23)
	{
		RobotState::m_output_int_register_23 = output_int_register_23;
	}
	int32_t RobotState::getOutput_int_register_24()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_24;
	}
	void RobotState::setOutput_int_register_24(int32_t output_int_register_24)
	{
		RobotState::m_output_int_register_24 = output_int_register_24;
	}
	int32_t RobotState::getOutput_int_register_25()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_25;
	}
	void RobotState::setOutput_int_register_25(int32_t output_int_register_25)
	{
		RobotState::m_output_int_register_25 = output_int_register_25;
	}
	int32_t RobotState::getOutput_int_register_26()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_26;
	}
	void RobotState::setOutput_int_register_26(int32_t output_int_register_26)
	{
		RobotState::m_output_int_register_26 = output_int_register_26;
	}
	int32_t RobotState::getOutput_int_register_27()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_27;
	}
	void RobotState::setOutput_int_register_27(int32_t output_int_register_27)
	{
		RobotState::m_output_int_register_27 = output_int_register_27;
	}
	int32_t RobotState::getOutput_int_register_28()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_28;
	}
	void RobotState::setOutput_int_register_28(int32_t output_int_register_28)
	{
		RobotState::m_output_int_register_28 = output_int_register_28;
	}
	int32_t RobotState::getOutput_int_register_29()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_29;
	}
	void RobotState::setOutput_int_register_29(int32_t output_int_register_29)
	{
		RobotState::m_output_int_register_29 = output_int_register_29;
	}
	int32_t RobotState::getOutput_int_register_30()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_30;
	}
	void RobotState::setOutput_int_register_30(int32_t output_int_register_30)
	{
		RobotState::m_output_int_register_30 = output_int_register_30;
	}
	int32_t RobotState::getOutput_int_register_31()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_31;
	}
	void RobotState::setOutput_int_register_31(int32_t output_int_register_31)
	{
		RobotState::m_output_int_register_31 = output_int_register_31;
	}
	int32_t RobotState::getOutput_int_register_32()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_32;
	}
	void RobotState::setOutput_int_register_32(int32_t output_int_register_32)
	{
		RobotState::m_output_int_register_32 = output_int_register_32;
	}
	int32_t RobotState::getOutput_int_register_33()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_33;
	}
	void RobotState::setOutput_int_register_33(int32_t output_int_register_33)
	{
		RobotState::m_output_int_register_33 = output_int_register_33;
	}
	int32_t RobotState::getOutput_int_register_34()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_34;
	}
	void RobotState::setOutput_int_register_34(int32_t output_int_register_34)
	{
		RobotState::m_output_int_register_34 = output_int_register_34;
	}
	int32_t RobotState::getOutput_int_register_35()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_35;
	}
	void RobotState::setOutput_int_register_35(int32_t output_int_register_35)
	{
		RobotState::m_output_int_register_35 = output_int_register_35;
	}
	int32_t RobotState::getOutput_int_register_36()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_36;
	}
	void RobotState::setOutput_int_register_36(int32_t output_int_register_36)
	{
		RobotState::m_output_int_register_36 = output_int_register_36;
	}
	int32_t RobotState::getOutput_int_register_37()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_37;
	}
	void RobotState::setOutput_int_register_37(int32_t output_int_register_37)
	{
		RobotState::m_output_int_register_37 = output_int_register_37;
	}
	int32_t RobotState::getOutput_int_register_38()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_38;
	}
	void RobotState::setOutput_int_register_38(int32_t output_int_register_38)
	{
		RobotState::m_output_int_register_38 = output_int_register_38;
	}
	int32_t RobotState::getOutput_int_register_39()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_39;
	}
	void RobotState::setOutput_int_register_39(int32_t output_int_register_39)
	{
		RobotState::m_output_int_register_39 = output_int_register_39;
	}
	int32_t RobotState::getOutput_int_register_40()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_40;
	}
	void RobotState::setOutput_int_register_40(int32_t output_int_register_40)
	{
		RobotState::m_output_int_register_40 = output_int_register_40;
	}
	int32_t RobotState::getOutput_int_register_41()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_41;
	}
	void RobotState::setOutput_int_register_41(int32_t output_int_register_41)
	{
		RobotState::m_output_int_register_41 = output_int_register_41;
	}
	int32_t RobotState::getOutput_int_register_42()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_42;
	}
	void RobotState::setOutput_int_register_42(int32_t output_int_register_42)
	{
		RobotState::m_output_int_register_42 = output_int_register_42;
	}
	int32_t RobotState::getOutput_int_register_43()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_43;
	}
	void RobotState::setOutput_int_register_43(int32_t output_int_register_43)
	{
		RobotState::m_output_int_register_43 = output_int_register_43;
	}
	int32_t RobotState::getOutput_int_register_44()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_44;
	}
	void RobotState::setOutput_int_register_44(int32_t output_int_register_44)
	{
		RobotState::m_output_int_register_44 = output_int_register_44;
	}
	int32_t RobotState::getOutput_int_register_45()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_45;
	}
	void RobotState::setOutput_int_register_45(int32_t output_int_register_45)
	{
		RobotState::m_output_int_register_45 = output_int_register_45;
	}
	int32_t RobotState::getOutput_int_register_46()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_46;
	}
	void RobotState::setOutput_int_register_46(int32_t output_int_register_46)
	{
		RobotState::m_output_int_register_46 = output_int_register_46;
	}
	int32_t RobotState::getOutput_int_register_47()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_int_register_47;
	}
	void RobotState::setOutput_int_register_47(int32_t output_int_register_47)
	{
		RobotState::m_output_int_register_47 = output_int_register_47;
	}
	double RobotState::getOutput_double_register_0()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_0;
	}
	void RobotState::setOutput_double_register_0(double output_double_register_0)
	{
		RobotState::m_output_double_register_0 = output_double_register_0;
	}
	double RobotState::getOutput_double_register_1()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_1;
	}
	void RobotState::setOutput_double_register_1(double output_double_register_1)
	{
		RobotState::m_output_double_register_1 = output_double_register_1;
	}
	double RobotState::getOutput_double_register_2()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_2;
	}
	void RobotState::setOutput_double_register_2(double output_double_register_2)
	{
		RobotState::m_output_double_register_2 = output_double_register_2;
	}
	double RobotState::getOutput_double_register_3()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_3;
	}
	void RobotState::setOutput_double_register_3(double output_double_register_3)
	{
		RobotState::m_output_double_register_3 = output_double_register_3;
	}
	double RobotState::getOutput_double_register_4()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_4;
	}
	void RobotState::setOutput_double_register_4(double output_double_register_4)
	{
		RobotState::m_output_double_register_4 = output_double_register_4;
	}
	double RobotState::getOutput_double_register_5()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_5;
	}
	void RobotState::setOutput_double_register_5(double output_double_register_5)
	{
		RobotState::m_output_double_register_5 = output_double_register_5;
	}
	double RobotState::getOutput_double_register_6()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_6;
	}
	void RobotState::setOutput_double_register_6(double output_double_register_6)
	{
		RobotState::m_output_double_register_6 = output_double_register_6;
	}
	double RobotState::getOutput_double_register_7()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_7;
	}
	void RobotState::setOutput_double_register_7(double output_double_register_7)
	{
		RobotState::m_output_double_register_7 = output_double_register_7;
	}
	double RobotState::getOutput_double_register_8()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_8;
	}
	void RobotState::setOutput_double_register_8(double output_double_register_8)
	{
		RobotState::m_output_double_register_8 = output_double_register_8;
	}
	double RobotState::getOutput_double_register_9()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_9;
	}
	void RobotState::setOutput_double_register_9(double output_double_register_9)
	{
		RobotState::m_output_double_register_9 = output_double_register_9;
	}
	double RobotState::getOutput_double_register_10()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_10;
	}
	void RobotState::setOutput_double_register_10(double output_double_register_10)
	{
		RobotState::m_output_double_register_10 = output_double_register_10;
	}
	double RobotState::getOutput_double_register_11()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_11;
	}
	void RobotState::setOutput_double_register_11(double output_double_register_11)
	{
		RobotState::m_output_double_register_11 = output_double_register_11;
	}
	double RobotState::getOutput_double_register_12()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_12;
	}
	void RobotState::setOutput_double_register_12(double output_double_register_12)
	{
		RobotState::m_output_double_register_12 = output_double_register_12;
	}
	double RobotState::getOutput_double_register_13()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_13;
	}
	void RobotState::setOutput_double_register_13(double output_double_register_13)
	{
		RobotState::m_output_double_register_13 = output_double_register_13;
	}
	double RobotState::getOutput_double_register_14()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_14;
	}
	void RobotState::setOutput_double_register_14(double output_double_register_14)
	{
		RobotState::m_output_double_register_14 = output_double_register_14;
	}
	double RobotState::getOutput_double_register_15()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_15;
	}
	void RobotState::setOutput_double_register_15(double output_double_register_15)
	{
		RobotState::m_output_double_register_15 = output_double_register_15;
	}
	double RobotState::getOutput_double_register_16()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_16;
	}
	void RobotState::setOutput_double_register_16(double output_double_register_16)
	{
		RobotState::m_output_double_register_16 = output_double_register_16;
	}
	double RobotState::getOutput_double_register_17()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_17;
	}
	void RobotState::setOutput_double_register_17(double output_double_register_17)
	{
		RobotState::m_output_double_register_17 = output_double_register_17;
	}
	double RobotState::getOutput_double_register_18()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_18;
	}
	void RobotState::setOutput_double_register_18(double output_double_register_18)
	{
		RobotState::m_output_double_register_18 = output_double_register_18;
	}
	double RobotState::getOutput_double_register_19()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_19;
	}
	void RobotState::setOutput_double_register_19(double output_double_register_19)
	{
		RobotState::m_output_double_register_19 = output_double_register_19;
	}
	double RobotState::getOutput_double_register_20()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_20;
	}
	void RobotState::setOutput_double_register_20(double output_double_register_20)
	{
		RobotState::m_output_double_register_20 = output_double_register_20;
	}
	double RobotState::getOutput_double_register_21()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_21;
	}
	void RobotState::setOutput_double_register_21(double output_double_register_21)
	{
		RobotState::m_output_double_register_21 = output_double_register_21;
	}
	double RobotState::getOutput_double_register_22()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_22;
	}
	void RobotState::setOutput_double_register_22(double output_double_register_22)
	{
		RobotState::m_output_double_register_22 = output_double_register_22;
	}
	double RobotState::getOutput_double_register_23()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_23;
	}
	void RobotState::setOutput_double_register_23(double output_double_register_23)
	{
		RobotState::m_output_double_register_23 = output_double_register_23;
	}
	double RobotState::getOutput_double_register_24()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_24;
	}
	void RobotState::setOutput_double_register_24(double output_double_register_24)
	{
		RobotState::m_output_double_register_24 = output_double_register_24;
	}
	double RobotState::getOutput_double_register_25()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_25;
	}
	void RobotState::setOutput_double_register_25(double output_double_register_25)
	{
		RobotState::m_output_double_register_25 = output_double_register_25;
	}
	double RobotState::getOutput_double_register_26()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_26;
	}
	void RobotState::setOutput_double_register_26(double output_double_register_26)
	{
		RobotState::m_output_double_register_26 = output_double_register_26;
	}
	double RobotState::getOutput_double_register_27()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_27;
	}
	void RobotState::setOutput_double_register_27(double output_double_register_27)
	{
		RobotState::m_output_double_register_27 = output_double_register_27;
	}
	double RobotState::getOutput_double_register_28()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_28;
	}
	void RobotState::setOutput_double_register_28(double output_double_register_28)
	{
		RobotState::m_output_double_register_28 = output_double_register_28;
	}
	double RobotState::getOutput_double_register_29()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_29;
	}
	void RobotState::setOutput_double_register_29(double output_double_register_29)
	{
		RobotState::m_output_double_register_29 = output_double_register_29;
	}
	double RobotState::getOutput_double_register_30()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_30;
	}
	void RobotState::setOutput_double_register_30(double output_double_register_30)
	{
		RobotState::m_output_double_register_30 = output_double_register_30;
	}
	double RobotState::getOutput_double_register_31()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_31;
	}
	void RobotState::setOutput_double_register_31(double output_double_register_31)
	{
		RobotState::m_output_double_register_31 = output_double_register_31;
	}
	double RobotState::getOutput_double_register_32()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_32;
	}
	void RobotState::setOutput_double_register_32(double output_double_register_32)
	{
		RobotState::m_output_double_register_32 = output_double_register_32;
	}
	double RobotState::getOutput_double_register_33()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_33;
	}
	void RobotState::setOutput_double_register_33(double output_double_register_33)
	{
		RobotState::m_output_double_register_33 = output_double_register_33;
	}
	double RobotState::getOutput_double_register_34()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_34;
	}
	void RobotState::setOutput_double_register_34(double output_double_register_34)
	{
		RobotState::m_output_double_register_34 = output_double_register_34;
	}
	double RobotState::getOutput_double_register_35()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_35;
	}
	void RobotState::setOutput_double_register_35(double output_double_register_35)
	{
		RobotState::m_output_double_register_35 = output_double_register_35;
	}
	double RobotState::getOutput_double_register_36()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_36;
	}
	void RobotState::setOutput_double_register_36(double output_double_register_36)
	{
		RobotState::m_output_double_register_36 = output_double_register_36;
	}
	double RobotState::getOutput_double_register_37()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_37;
	}
	void RobotState::setOutput_double_register_37(double output_double_register_37)
	{
		RobotState::m_output_double_register_37 = output_double_register_37;
	}
	double RobotState::getOutput_double_register_38()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_38;
	}
	void RobotState::setOutput_double_register_38(double output_double_register_38)
	{
		RobotState::m_output_double_register_38 = output_double_register_38;
	}
	double RobotState::getOutput_double_register_39()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_39;
	}
	void RobotState::setOutput_double_register_39(double output_double_register_39)
	{
		RobotState::m_output_double_register_39 = output_double_register_39;
	}
	double RobotState::getOutput_double_register_40()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_40;
	}
	void RobotState::setOutput_double_register_40(double output_double_register_40)
	{
		RobotState::m_output_double_register_40 = output_double_register_40;
	}
	double RobotState::getOutput_double_register_41()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_41;
	}
	void RobotState::setOutput_double_register_41(double output_double_register_41)
	{
		RobotState::m_output_double_register_41 = output_double_register_41;
	}
	double RobotState::getOutput_double_register_42()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_42;
	}
	void RobotState::setOutput_double_register_42(double output_double_register_42)
	{
		RobotState::m_output_double_register_42 = output_double_register_42;
	}
	double RobotState::getOutput_double_register_43()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_43;
	}
	void RobotState::setOutput_double_register_43(double output_double_register_43)
	{
		RobotState::m_output_double_register_43 = output_double_register_43;
	}
	double RobotState::getOutput_double_register_44()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_44;
	}
	void RobotState::setOutput_double_register_44(double output_double_register_44)
	{
		RobotState::m_output_double_register_44 = output_double_register_44;
	}
	double RobotState::getOutput_double_register_45()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_45;
	}
	void RobotState::setOutput_double_register_45(double output_double_register_45)
	{
		RobotState::m_output_double_register_45 = output_double_register_45;
	}
	double RobotState::getOutput_double_register_46()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_46;
	}
	void RobotState::setOutput_double_register_46(double output_double_register_46)
	{
		RobotState::m_output_double_register_46 = output_double_register_46;
	}
	double RobotState::getOutput_double_register_47()
	{
		std::lock_guard<std::mutex> lock(m_update_state_mutex);
		return m_output_double_register_47;
	}
	void RobotState::setOutput_double_register_47(double output_double_register_47)
	{
		RobotState::m_output_double_register_47 = output_double_register_47;
	}



}