#pragma once

#include <vector>
#include <cstdint>
#include <mutex>

namespace cs_rtsi
{
	class RobotState
	{
	public:
		explicit RobotState();
		virtual ~RobotState();

		bool lockUpdateStateMutex();

		bool unlockUpdateStateMutex();

	public:
		double getPayload_mass();
		void setPayload_mass(double payload_mass);

		const std::vector<double> getPayload_cog();
		void setPayload_cog(const std::vector<double> &payload_cog);


		double getTimestamp();
		void setTimestamp(double timestamp);

		const std::vector<double> getTarget_q();
		void setTarget_q(const std::vector<double> &target_q);

		const std::vector<double> getTarget_qd();
		void setTarget_qd(const std::vector<double> &target_qd);

		const std::vector<double> getTarget_qdd();
		void setTarget_qdd(const std::vector<double> &target_qdd);

		const std::vector<double> getTarget_current();
		void setTarget_current(const std::vector<double> &target_current);

		const std::vector<double> getTarget_moment();
		void setTarget_moment(const std::vector<double> &target_moment);

		const std::vector<double> getActual_q();
		void setActual_q(const std::vector<double> &actual_q);

		const std::vector<double> getActual_qd();
		void setActual_qd(const std::vector<double> &actual_qd);

		const std::vector<double> getActual_moment();
		void setActual_moment(const std::vector<double> &actual_moment);

		const std::vector<double> getActual_current();
		void setActual_current(const std::vector<double> &actual_current);

		const std::vector<double> getJoint_control_output();
		void setJoint_control_output(const std::vector<double> &joint_control_output);

		const std::vector<double> getActual_TCP_pose();
		void setActual_TCP_pose(const std::vector<double> &actual_TCP_pose);

		const std::vector<double> getActual_TCP_speed();
		void setActual_TCP_speed(const std::vector<double> &actual_TCP_speed);

		const std::vector<double> getActual_TCP_force();
		void setActual_TCP_force(const std::vector<double> &actual_TCP_force);

		const std::vector<double> getActual_sensor_force();
		void setActual_sensor_force(const std::vector<double> &actual_sensor_force);

		double getActual_TCP_force_scalar();
		void setActual_TCP_force_scalar(double &tcp_force_scalar);


		const std::vector<double> getTarget_TCP_pose();
		void setTarget_TCP_pose(const std::vector<double> &target_TCP_pose);

		const std::vector<double> getTarget_TCP_speed();
		void setTarget_TCP_speed(const std::vector<double> &target_TCP_speed);

		uint32_t getActual_digital_input_bits();
		void setActual_digital_input_bits(uint32_t actual_digital_input_bits);

		const std::vector<double> getJoint_temperatures();
		void setJoint_temperatures(const std::vector<double> &joint_temperatures);

		double getActual_execution_time();
		void setActual_execution_time(double actual_execution_time);

		int32_t getRobot_mode();
		void setRobot_mode(int32_t robot_mode);

		uint32_t getRobot_status_bits();
		void setRobot_status_bits(uint32_t robot_status);

		const std::vector<int32_t> getJoint_mode();
		void setJoint_mode(const std::vector<int32_t> &joint_mode);


		int32_t getSafety_mode();
		void setSafety_mode(int32_t safety_mode);


		int32_t getSafety_status();
		void setSafety_status(int32_t safety_status);


		uint32_t getSafety_status_bits();
		void setSafety_status_bits(uint32_t safety_status_bits);

		const std::vector<double> getActual_tool_accelerometer();
		void setActual_tool_accelerometer(const std::vector<double> &actual_tool_accelerometer);


		double getSpeed_scaling();
		void setSpeed_scaling(double speed_scaling);

		double getTarget_speed_fraction();
		void setTarget_speed_fraction(double target_speed_fraction);

		double getActual_momentum();
		void setActual_momentum(double actual_momentum);

		double getActual_robot_voltage();
		void setActual_robot_voltage(double actual_robot_voltage);


		double getActual_robot_current();
		void setActual_robot_current(double actual_robot_current);

		const std::vector<double> getActual_joint_voltage();
		void setActual_joint_voltage(const std::vector<double> &actual_joint_voltage);

		const std::vector<double> getActual_joint_current();
		void setActual_joint_current(const std::vector<double> &actual_joint_current);

		const std::vector<double> getActual_joint_torques();
		void setActual_joint_torques(const std::vector<double> &actual_joint_torques);



		uint32_t getActual_digital_output_bits();
		void setActual_digital_output_bits(uint32_t actual_digital_output_bits);

		uint32_t getRuntime_state();
		void setRuntime_state(uint32_t runtime_state);

		const std::vector<double> getElbow_position();
		void setElbow_position(const std::vector<double> &elbow_position);

		double getStandard_analog_input_0();
		void setStandard_analog_input_0(double standard_analog_input_0);
		double getStandard_analog_input_1();
		void setStandard_analog_input_1(double standard_analog_input_1);
		double getStandard_analog_output_0();
		void setStandard_analog_output_0(double standard_analog_output_0);
		double getStandard_analog_output_1();
		void setStandard_analog_output_1(double standard_analog_output_1);


		uint32_t getOutput_bit_registers0_to_31();
		void setOutput_bit_registers0_to_31(uint32_t output_bit_registers0_to_31);
		uint32_t getOutput_bit_registers32_to_63();
		void setOutput_bit_registers32_to_63(uint32_t output_bit_registers32_to_63);


		int32_t getOutput_int_register_0();
		void setOutput_int_register_0(int32_t output_int_register_0);
		int32_t getOutput_int_register_1();
		void setOutput_int_register_1(int32_t output_int_register_1);
		int32_t getOutput_int_register_2();
		void setOutput_int_register_2(int32_t output_int_register_2);
		int32_t getOutput_int_register_3();
		void setOutput_int_register_3(int32_t output_int_register_3);
		int32_t getOutput_int_register_4();
		void setOutput_int_register_4(int32_t output_int_register_4);
		int32_t getOutput_int_register_5();
		void setOutput_int_register_5(int32_t output_int_register_5);
		int32_t getOutput_int_register_6();
		void setOutput_int_register_6(int32_t output_int_register_6);
		int32_t getOutput_int_register_7();
		void setOutput_int_register_7(int32_t output_int_register_7);
		int32_t getOutput_int_register_8();
		void setOutput_int_register_8(int32_t output_int_register_8);
		int32_t getOutput_int_register_9();
		void setOutput_int_register_9(int32_t output_int_register_9);
		int32_t getOutput_int_register_10();
		void setOutput_int_register_10(int32_t output_int_register_10);
		int32_t getOutput_int_register_11();
		void setOutput_int_register_11(int32_t output_int_register_11);
		int32_t getOutput_int_register_12();
		void setOutput_int_register_12(int32_t output_int_register_12);
		int32_t getOutput_int_register_13();
		void setOutput_int_register_13(int32_t output_int_register_13);
		int32_t getOutput_int_register_14();
		void setOutput_int_register_14(int32_t output_int_register_14);
		int32_t getOutput_int_register_15();
		void setOutput_int_register_15(int32_t output_int_register_15);
		int32_t getOutput_int_register_16();
		void setOutput_int_register_16(int32_t output_int_register_16);
		int32_t getOutput_int_register_17();
		void setOutput_int_register_17(int32_t output_int_register_17);
		int32_t getOutput_int_register_18();
		void setOutput_int_register_18(int32_t output_int_register_18);
		int32_t getOutput_int_register_19();
		void setOutput_int_register_19(int32_t output_int_register_19);
		int32_t getOutput_int_register_20();
		void setOutput_int_register_20(int32_t output_int_register_20);
		int32_t getOutput_int_register_21();
		void setOutput_int_register_21(int32_t output_int_register_21);
		int32_t getOutput_int_register_22();
		void setOutput_int_register_22(int32_t output_int_register_22);
		int32_t getOutput_int_register_23();
		void setOutput_int_register_23(int32_t output_int_register_23);

		int32_t getOutput_int_register_24();
		void setOutput_int_register_24(int32_t output_int_register_24);
		int32_t getOutput_int_register_25();
		void setOutput_int_register_25(int32_t output_int_register_25);
		int32_t getOutput_int_register_26();
		void setOutput_int_register_26(int32_t output_int_register_26);
		int32_t getOutput_int_register_27();
		void setOutput_int_register_27(int32_t output_int_register_27);
		int32_t getOutput_int_register_28();
		void setOutput_int_register_28(int32_t output_int_register_28);
		int32_t getOutput_int_register_29();
		void setOutput_int_register_29(int32_t output_int_register_29);
		int32_t getOutput_int_register_30();
		void setOutput_int_register_30(int32_t output_int_register_30);
		int32_t getOutput_int_register_31();
		void setOutput_int_register_31(int32_t output_int_register_31);
		int32_t getOutput_int_register_32();
		void setOutput_int_register_32(int32_t output_int_register_32);
		int32_t getOutput_int_register_33();
		void setOutput_int_register_33(int32_t output_int_register_33);
		int32_t getOutput_int_register_34();
		void setOutput_int_register_34(int32_t output_int_register_24);
		int32_t getOutput_int_register_35();
		void setOutput_int_register_35(int32_t output_int_register_35);
		int32_t getOutput_int_register_36();
		void setOutput_int_register_36(int32_t output_int_register_36);
		int32_t getOutput_int_register_37();
		void setOutput_int_register_37(int32_t output_int_register_37);
		int32_t getOutput_int_register_38();
		void setOutput_int_register_38(int32_t output_int_register_38);
		int32_t getOutput_int_register_39();
		void setOutput_int_register_39(int32_t output_int_register_39);
		int32_t getOutput_int_register_40();
		void setOutput_int_register_40(int32_t output_int_register_40);
		int32_t getOutput_int_register_41();
		void setOutput_int_register_41(int32_t output_int_register_41);
		int32_t getOutput_int_register_42();
		void setOutput_int_register_42(int32_t output_int_register_42);
		int32_t getOutput_int_register_43();
		void setOutput_int_register_43(int32_t output_int_register_43);
		int32_t getOutput_int_register_44();
		void setOutput_int_register_44(int32_t output_int_register_44);
		int32_t getOutput_int_register_45();
		void setOutput_int_register_45(int32_t output_int_register_45);
		int32_t getOutput_int_register_46();
		void setOutput_int_register_46(int32_t output_int_register_46);
		int32_t getOutput_int_register_47();
		void setOutput_int_register_47(int32_t output_int_register_47);

		double getOutput_double_register_0();
		void setOutput_double_register_0(double output_double_register_0);
		double getOutput_double_register_1();
		void setOutput_double_register_1(double output_double_register_1);
		double getOutput_double_register_2();
		void setOutput_double_register_2(double output_double_register_2);
		double getOutput_double_register_3();
		void setOutput_double_register_3(double output_double_register_3);
		double getOutput_double_register_4();
		void setOutput_double_register_4(double output_double_register_4);
		double getOutput_double_register_5();
		void setOutput_double_register_5(double output_double_register_5);
		double getOutput_double_register_6();
		void setOutput_double_register_6(double output_double_register_6);
		double getOutput_double_register_7();
		void setOutput_double_register_7(double output_double_register_7);
		double getOutput_double_register_8();
		void setOutput_double_register_8(double output_double_register_8);
		double getOutput_double_register_9();
		void setOutput_double_register_9(double output_double_register_9);
		double getOutput_double_register_10();
		void setOutput_double_register_10(double output_double_register_10);
		double getOutput_double_register_11();
		void setOutput_double_register_11(double output_double_register_11);
		double getOutput_double_register_12();
		void setOutput_double_register_12(double output_double_register_12);
		double getOutput_double_register_13();
		void setOutput_double_register_13(double output_double_register_13);
		double getOutput_double_register_14();
		void setOutput_double_register_14(double output_double_register_14);
		double getOutput_double_register_15();
		void setOutput_double_register_15(double output_double_register_15);
		double getOutput_double_register_16();
		void setOutput_double_register_16(double output_double_register_16);
		double getOutput_double_register_17();
		void setOutput_double_register_17(double output_double_register_17);
		double getOutput_double_register_18();
		void setOutput_double_register_18(double output_double_register_18);
		double getOutput_double_register_19();
		void setOutput_double_register_19(double output_double_register_19);
		double getOutput_double_register_20();
		void setOutput_double_register_20(double output_double_register_20);
		double getOutput_double_register_21();
		void setOutput_double_register_21(double output_double_register_21);
		double getOutput_double_register_22();
		void setOutput_double_register_22(double output_double_register_22);
		double getOutput_double_register_23();
		void setOutput_double_register_23(double output_double_register_23);

		double getOutput_double_register_24();
		void setOutput_double_register_24(double output_double_register_24);
		double getOutput_double_register_25();
		void setOutput_double_register_25(double output_double_register_25);
		double getOutput_double_register_26();
		void setOutput_double_register_26(double output_double_register_26);
		double getOutput_double_register_27();
		void setOutput_double_register_27(double output_double_register_27);
		double getOutput_double_register_28();
		void setOutput_double_register_28(double output_double_register_28);
		double getOutput_double_register_29();
		void setOutput_double_register_29(double output_double_register_29);
		double getOutput_double_register_30();
		void setOutput_double_register_30(double output_double_register_30);
		double getOutput_double_register_31();
		void setOutput_double_register_31(double output_double_register_31);
		double getOutput_double_register_32();
		void setOutput_double_register_32(double output_double_register_32);
		double getOutput_double_register_33();
		void setOutput_double_register_33(double output_double_register_33);
		double getOutput_double_register_34();
		void setOutput_double_register_34(double output_double_register_34);
		double getOutput_double_register_35();
		void setOutput_double_register_35(double output_double_register_35);
		double getOutput_double_register_36();
		void setOutput_double_register_36(double output_double_register_36);
		double getOutput_double_register_37();
		void setOutput_double_register_37(double output_double_register_37);
		double getOutput_double_register_38();
		void setOutput_double_register_38(double output_double_register_38);
		double getOutput_double_register_39();
		void setOutput_double_register_39(double output_double_register_39);
		double getOutput_double_register_40();
		void setOutput_double_register_40(double output_double_register_40);
		double getOutput_double_register_41();
		void setOutput_double_register_41(double output_double_register_41);
		double getOutput_double_register_42();
		void setOutput_double_register_42(double output_double_register_42);
		double getOutput_double_register_43();
		void setOutput_double_register_43(double output_double_register_43);
		double getOutput_double_register_44();
		void setOutput_double_register_44(double output_double_register_44);
		double getOutput_double_register_45();
		void setOutput_double_register_45(double output_double_register_45);
		double getOutput_double_register_46();
		void setOutput_double_register_46(double output_double_register_46);
		double getOutput_double_register_47();
		void setOutput_double_register_47(double output_double_register_47);

     
	private:

		double m_payload_mass;
		std::vector<double> m_payload_cog;
		double m_timestamp;
		std::vector<double> m_target_q;
		std::vector<double> m_target_qd;
		std::vector<double> m_target_qdd;
		std::vector<double> m_target_current;
		std::vector<double> m_target_moment;
		std::vector<double> m_actual_q;
		std::vector<double> m_actual_qd;
		std::vector<double> m_actual_moment;   //Elite special
		std::vector<double> m_actual_current;
		std::vector<double> m_joint_control_output;
		std::vector<double> m_actual_TCP_pose;
		std::vector<double> m_actual_TCP_speed;
		double m_tcp_force_scalar;
		std::vector<double> m_target_TCP_pose;
		std::vector<double> m_target_TCP_speed;
		std::vector<double> m_actual_TCP_force;
		std::vector<double> m_actual_sensor_force;

		uint32_t m_actual_digital_input_bits; //elite special 
		std::vector<double> m_joint_temperatures;
		double m_actual_execution_time;

		int32_t m_robot_mode;
		uint32_t m_robot_status_bits;
		uint32_t m_safety_status_bits;
		std::vector<int32_t> m_joint_mode;
		int32_t m_safety_mode;
		int32_t m_safety_status;
		std::vector<double> m_actual_tool_accelerometer;
		double m_speed_scaling;
		double m_target_speed_fraction;
		double m_actual_momentum;
		double m_actual_main_voltage;
		double m_actual_robot_voltage;
		double m_actual_robot_current;
		std::vector<double> m_actual_joint_voltage;
		std::vector<double> m_actual_joint_current;
		std::vector<double> m_actual_joint_torques;


		uint32_t m_actual_digital_output_bits; //elite special
		uint32_t m_runtime_state;
		std::vector<double> m_elbow_position;

		double m_standard_analog_input_0;
		double m_standard_analog_input_1;
		double m_standard_analog_output_0;
		double m_standard_analog_output_1;

		uint32_t m_output_bit_registers0_to_31;
		uint32_t m_output_bit_registers32_to_63;

		int32_t m_output_int_register_0;
		int32_t m_output_int_register_1;
		int32_t m_output_int_register_2;
		int32_t m_output_int_register_3;
		int32_t m_output_int_register_4;
		int32_t m_output_int_register_5;
		int32_t m_output_int_register_6;
		int32_t m_output_int_register_7;
		int32_t m_output_int_register_8;
		int32_t m_output_int_register_9;
		int32_t m_output_int_register_10;
		int32_t m_output_int_register_11;
		int32_t m_output_int_register_12;
		int32_t m_output_int_register_13;
		int32_t m_output_int_register_14;
		int32_t m_output_int_register_15;
		int32_t m_output_int_register_16;
		int32_t m_output_int_register_17;
		int32_t m_output_int_register_18;
		int32_t m_output_int_register_19;
		int32_t m_output_int_register_20;
		int32_t m_output_int_register_21;
		int32_t m_output_int_register_22;
		int32_t m_output_int_register_23;

		int32_t m_output_int_register_24;
		int32_t m_output_int_register_25;
		int32_t m_output_int_register_26;
		int32_t m_output_int_register_27;
		int32_t m_output_int_register_28;
		int32_t m_output_int_register_29;
		int32_t m_output_int_register_30;
		int32_t m_output_int_register_31;
		int32_t m_output_int_register_32;
		int32_t m_output_int_register_33;
		int32_t m_output_int_register_34;
		int32_t m_output_int_register_35;
		int32_t m_output_int_register_36;
		int32_t m_output_int_register_37;
		int32_t m_output_int_register_38;
		int32_t m_output_int_register_39;
		int32_t m_output_int_register_40;
		int32_t m_output_int_register_41;
		int32_t m_output_int_register_42;
		int32_t m_output_int_register_43;
		int32_t m_output_int_register_44;
		int32_t m_output_int_register_45;
		int32_t m_output_int_register_46;
		int32_t m_output_int_register_47;

		double m_output_double_register_0;
		double m_output_double_register_1;
		double m_output_double_register_2;
		double m_output_double_register_3;
		double m_output_double_register_4;
		double m_output_double_register_5;
		double m_output_double_register_6;
		double m_output_double_register_7;
		double m_output_double_register_8;
		double m_output_double_register_9;
		double m_output_double_register_10;
		double m_output_double_register_11;
		double m_output_double_register_12;
		double m_output_double_register_13;
		double m_output_double_register_14;
		double m_output_double_register_15;
		double m_output_double_register_16;
		double m_output_double_register_17;
		double m_output_double_register_18;
		double m_output_double_register_19;
		double m_output_double_register_20;
		double m_output_double_register_21;
		double m_output_double_register_22;
		double m_output_double_register_23;

		double m_output_double_register_24;
		double m_output_double_register_25;
		double m_output_double_register_26;
		double m_output_double_register_27;
		double m_output_double_register_28;
		double m_output_double_register_29;
		double m_output_double_register_30;
		double m_output_double_register_31;
		double m_output_double_register_32;
		double m_output_double_register_33;
		double m_output_double_register_34;
		double m_output_double_register_35;
		double m_output_double_register_36;
		double m_output_double_register_37;
		double m_output_double_register_38;
		double m_output_double_register_39;
		double m_output_double_register_40;
		double m_output_double_register_41;
		double m_output_double_register_42;
		double m_output_double_register_43;
		double m_output_double_register_44;
		double m_output_double_register_45;
		double m_output_double_register_46;
		double m_output_double_register_47;

		std::mutex m_update_state_mutex;



	};
}