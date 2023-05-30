#pragma once

#include "rtsi_export.h"
#include "rtsi.h"
#include <memory>
#include <string>
#include <map>
#include <atomic>
#include <functional>

#define MAJOR_VERSION 2
#define MINOR_VERSION 1

#define CS_CONTROLLER_READY_FOR_CMD 1
#define CS_CONTROLLER_DONE_WITH_CMD 2
#define CS_EXECUTION_TIMEOUT 300
#define CS_PATH_EXECUTION_TIMEOUT 600
#define CS_GET_READY_TIMEOUT 3
#define RTSI_START_SYNCHRONIZATION_TIMEOUT 50
#define WAIT_FOR_PROGRAM_RUNNING_TIMEOUT 60


#define CS_JOINT_VELOCITY_MAX 3.14      // rad/s
#define CS_JOINT_VELOCITY_MIN 0         // rad/s
#define CS_JOINT_ACCELERATION_MAX 40.0  // rad/s^2
#define CS_JOINT_ACCELERATION_MIN 0     // rad/s^2
#define CS_TOOL_VELOCITY_MAX 3.0        // m/s
#define CS_TOOL_VELOCITY_MIN 0          // m/s
#define CS_TOOL_ACCELERATION_MAX 150.0  // m/s^2
#define CS_TOOL_ACCELERATION_MIN 0      // m/s^2
#define CS_SERVO_LOOKAHEAD_TIME_MAX 0.2
#define CS_SERVO_LOOKAHEAD_TIME_MIN 0.03
#define CS_SERVO_GAIN_MAX 2000
#define CS_SERVO_GAIN_MIN 100
#define CS_BLEND_MAX 2.0
#define CS_BLEND_MIN 0.0

// forward declarations
namespace boost
{
	class thread;
}
namespace cs_rtsi
{
	class DashboardClient;
}
namespace cs_rtsi
{
	class ScriptClient;
}
namespace cs_rtsi
{
	class RobotState;
}
namespace cs_rtsi
{
	class RTSI;
}

namespace cs_rtsi
{

	class Path;



	//这个类提供了控制机器人和执行机器人的接口运动。注意目前RTSIControlInterface，不应该被认为是线程
    //安全，因为没有采取任何措施(互斥)来确保函数是安全的，在执行另一个之前。这取决于调用者是否使用互斥锁提供保护。
	class RTSIControlInterface
	{
	public:

		enum Flags
		{
			FLAG_UPLOAD_SCRIPT = 0x01,
			FLAG_USE_EXT_CS_CAP = 0x02,
			FLAG_VERBOSE = 0x04,
			FLAG_UPPER_RANGE_REGISTERS = 0x08,
			FLAG_NO_WAIT = 0x10,
			FLAG_CUSTOM_SCRIPT = 0x20,
			FLAGS_DEFAULT = FLAG_UPLOAD_SCRIPT
		};

		enum RobotStatus
		{
			ROBOT_STATUS_POWER_ON = 0,
			ROBOT_STATUS_PROGRAM_RUNNING = 1,
			ROBOT_STATUS_TEACH_BUTTON_PRESSED = 2,
		};

		enum SafetyStatus
		{
			IS_NORMAL_MODE = 0,
			IS_REDUCED_MODE = 1,
			IS_PROTECTIVE_STOPPED = 2,
			IS_RECOVERY_MODE = 3,
			IS_SAFEGUARD_STOPPED = 4,
			IS_SYSTEM_EMERGENCY_STOPPED = 5,
			IS_ROBOT_EMERGENCY_STOPPED = 6,
			IS_EMERGENCY_STOPPED = 7
		};

		enum RuntimeState
		{
			UNKNOWN = 0,
			PLAYING = 1,
			PAUSED = 2,
			STOPPED = 3
		};

		enum Feature
		{
			FEATURE_BASE,
			FEATURE_TOOL,
			FEATURE_CUSTOM  // not supported yet - reserved for future
		};

		RTSI_EXPORT explicit RTSIControlInterface(std::string hostname, uint16_t flags = FLAGS_DEFAULT,
			int cs_cap_port = 50002);

		RTSI_EXPORT virtual ~RTSIControlInterface();


		/**
		* @brief Can be used to disconnect from the robot. To reconnect you have to call the reconnect() function.
		*/
		RTSI_EXPORT void disconnect();

		/**
		* @brief Can be used to reconnect to the robot after a lost connection.
		*/
		RTSI_EXPORT bool reconnect();

		/**
		* @brief Connection status for RTSI, useful for checking for lost connection.
		*/
		RTSI_EXPORT bool isConnected();

		/**
		 * @brief In the event of an error, this function can be used to resume operation by reuploading the RTSI control
		 * script. This will only happen if a script is not already running on the controller.
		 */
		RTSI_EXPORT bool reuploadScript();


		/***************************
		@brief      : 向控制器发送一个自定义的脚本
		@param[in]  :　指定的脚本函数
		@param[in]  :　这个自定义的脚本被指定为一个字符串
		@return     :
		@time       : 2023/04/13
		****************************/
		RTSI_EXPORT bool sendCustomScriptFunction(const std::string &function_name, const std::string &script);


		/**
		  * Send a custom ur script to the controller
		  * The function enables sending of short scripts which was defined inline
		  * within source code. So you can write code like this:
		  * \code
		  * const std::string inline_script =
					   "def script_test():\n"
							   "\tdef test():\n"
									   "textmsg(\"test1\")\n"
									   "textmsg(\"test2\")\n"
							   "\tend\n"
							   "\twrite_output_integer_register(0, 1)\n"
							   "\ttest()\n"
							   "\ttest()\n"
							   "\twrite_output_integer_register(0, 2)\n"
					   "end\n"
					   "run program\n";
				 bool result = rtsi_c.sendCustomScript(inline_script);
		  * \endcode
		  * @return Returns true if the script has been executed successfully and false
		  * on timeout
		  */
		RTSI_EXPORT bool sendCustomScript(const std::string &script);

		/**
		 * @brief Send a custom ur script file to the controller
		 * @param file_path the file path to the custom ur script file
		 */
		RTSI_EXPORT bool sendCustomScriptFile(const std::string &file_path);


		/**
		 * Assign a custom script file that will be sent to device as the main
		 * control script.
		 * Setting an empty file_name will disable the custom script loading
		 * This eases debugging when modifying the control
		 * script because it does not require to recompile the whole library
		 */
		RTSI_EXPORT void setCustomScriptFile(const std::string &file_path);

		/**
		 * @brief This function will terminate the script on controller.
		 */
		RTSI_EXPORT void stopScript();

		/**
		* @brief Returns true if a program is running on the controller, otherwise it returns false
		*/
		bool isProgramRunning();


	public:

		/***************************
		@brief      : Stop (linear in tool space) - decelerate tool speed to zero
		@param[in]  : a tool acceleration [m/s^2] (rate of deceleration of the tool)
		@time       : 2023/04/17
		****************************/
		RTSI_EXPORT void stopL(double a = 10.0);


		/***************************
		@brief      : Stop (linear in joint space) - decelerate joint speeds to zero
		@param[in]  : a joint acceleration [rad/s^2] (rate of deceleration of the leading axis).
		@time       : 2023/04/17
		****************************/
		RTSI_EXPORT void stopJ(double a = 2.0);



		/***************************
		@brief      : Move to joint position (linear in joint-space)
		@param[in]  : joint positions
		@param[in]  : speed joint speed of leading axis [rad/s]
		@param[in]  : acceleration joint acceleration of leading axis [rad/s^2]
		@param[in]  : async a bool specifying if the move command should be asynchronous. If async is true it is possible to
        stop a move command using either the stopJ or stopL function. Default is false, this means the function will
        block until the movement has completed.
		@time       : 2023/04/17
		****************************/
		RTSI_EXPORT bool moveJ(const std::vector<double> &q, double speed = 1.05, double acceleration = 1.4,
			bool async = false);

		/***************************
		@brief      : 　在笛卡尔空间移动一个位置
		@param[in]  :　　目标位姿
		@param[in]  :　　工具速度[m/s]
		@param[in]  :　　工具加速度[m/s^2]
		@param[in]  :　　a bool specifying if the move command should be asynchronous. If async is true it is possible to
		stop a move command using either the stopJ or stopL function. Default is false, this means the function will
		block until the movement has completed.
		@time       : 2023/04/17
		****************************/
		RTSI_EXPORT bool moveL(const std::vector<double> &pose, double speed = 0.25, double acceleration = 1.2,
			bool async = false);

		/**
		* @brief Joint speed - Accelerate linearly in joint space and continue with constant joint speed
		* @param qd joint speeds [rad/s]
		* @param acceleration joint acceleration [rad/s^2] (of leading axis)
		* @param time time [s] before the function returns (optional)
		*/
		RTSI_EXPORT bool speedJ(const std::vector<double> &qd, double acceleration = 0.5, double time = 0.0);


		/**
		* @brief Tool speed - Accelerate linearly in Cartesian space and continue with constant tool speed. The time t is
		* optional;
		* @param xd tool speed [m/s] (spatial vector)
		* @param acceleration tool position acceleration [m/s^2]
		* @param time time [s] before the function returns (optional)
		*/
		RTSI_EXPORT bool speedL(const std::vector<double> &xd, double acceleration = 0.25, double time = 0.0);


		/**
		* @brief Servo to position (linear in joint-space)
		* @param q joint positions [rad]
		* @param speed NOT used in current version
		* @param acceleration NOT used in current version
		* @param time time where the command is controlling the robot. The function is blocking for time t [S]
		* @param lookahead_time time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
		* @param gain proportional gain for following target position, range [100,2000]
		*/
		RTSI_EXPORT bool servoJ(const std::vector<double> &q, double speed, double acceleration, double time,
			double lookahead_time, double gain);


		/**
		* @brief Servo to position (linear in tool-space)
		* @param pose target pose
		* @param speed NOT used in current version
		* @param acceleration NOT used in current version
		* @param time time where the command is controlling the robot. The function is blocking for time t [S]
		* @param lookahead_time time [S], range [0.03,0.2] smoothens the trajectory with this lookahead time
		* @param gain proportional gain for following target position, range [100,2000]
		*/
		RTSI_EXPORT bool servoL(const std::vector<double> &pose, double speed, double acceleration, double time,
			double lookahead_time, double gain);


		/**
		* @brief Stop servo mode and decelerate the robot.
		* @param a rate of deceleration of the tool [m/s^2]
		*/
		RTSI_EXPORT bool servoStop(double a = 10.0);

		/**
		* @brief Stop speed mode and decelerate the robot.
		* @param a rate of deceleration of the tool [m/s^2] if using speedL, for speedJ its [rad/s^2]
		* and rate of deceleration of leading axis.
		*/
		RTSI_EXPORT bool speedStop(double a = 10.0);


		RTSI_EXPORT bool forceMode(const std::vector<double> &task_frame, const std::vector<int> &selection_vector,
			const std::vector<double> &wrench, int type, const std::vector<double> &limits);

		/**
		* @brief Resets the robot mode from force mode to normal operation.
		*/
		RTSI_EXPORT bool forceModeStop();

		/**
		* @brief Starts jogging with the given speed vector with respect to the given
		* feature.
		* When jogging has started, it is possible to provide new speed vectors by
		* calling the jogStart() function over and over again. This makes it
		* possible to use a joystick or a 3D Space Navigator to provide new speed
		* vectors if the user moves the joystick or the Space Navigator cap.
		* Switching the feature (base or tool) is only possible, if the jogging has
		* been stopped before the jogStart() function is called. That means, with
		* the first call of jogStart() the speed vector and feature parameter is
		* evaluated. With all following calls of the function only the speed vector
		* will be evaluated.
		* @param speed Speed vector for translation and rotation. Translation values
		* are given in mm / s and rotation values in rad / s.
		* @param feature Configures to move to move with respect to base frame
		* (FEATURE_BASE) or with respect to tcp frame (FEATURE_TOOL)
		*/
		RTSI_EXPORT bool jogStart(const std::vector<double> &speeds, int feature = FEATURE_BASE);


		/**
		* Stops jogging that has been started start_jog
		*/
		RTSI_EXPORT bool jogStop();


		/**
		* @brief Zeroes the TCP force/torque measurement from the builtin force/torque sensor by subtracting the current
		* measurement from the subsequent.
		*/
		RTSI_EXPORT bool zeroFtSensor();


		/**
		* @brief Set payload
		* @param mass Mass in kilograms
		* @param cog Center of Gravity, a vector [CoGx, CoGy, CoGz] specifying the displacement (in meters) from the
		* toolmount. If not specified the current CoG will be used.
		*/
		RTSI_EXPORT bool setPayload(double mass, const std::vector<double> &cog = {});


		/**
		* @brief Set robot in freedrive mode. In this mode the robot can be moved around by hand in the same way as
		* by pressing the "freedrive" button. The robot will not be able to follow a trajectory (eg. a movej) in this mode.
		*/
		bool teachMode();


		/**
		* @brief Set robot back in normal position control mode after freedrive mode.
		*/
		bool endTeachMode();


		/**
		* @brief Sets the damping parameter in force mode.
		* @param damping Between 0 and 1, default value is 0.005
		*
		* A value of 1 is full damping, so the robot will decellerate quickly if no force is present.
		* A value of 0 is no damping, here the robot will maintain the speed.
		*
		* The value is stored until this function is called again. Call this function
		* before force mode is entered (otherwise default value will be used).
		*/
		bool forceModeSetDamping(double damping);

		/**
		* @brief Scales the gain in force mode.
		* @param scaling scaling parameter between 0 and 2, default is 1.
		*
		* A value larger than 1 can make force mode unstable, e.g. in case of collisions or pushing against hard surfaces.
		*
		* The value is stored until this function is called again. Call this function before force mode is entered
		* (otherwise default value will be used)
		*/
		bool forceModeSetGainScaling(double scaling);


		/**
		* @brief Returns the duration of the robot time step in seconds.
		*
		* In every time step, the robot controller will receive measured joint positions and velocities from the robot, and
		* send desired joint positions and velocities back to the robot. This happens with a predetermined frequency, in
		* regular intervals. This interval length is the robot time step.
		*
		* @returns Duration of the robot step in seconds or 0 in case of an error
		*/
		RTSI_EXPORT double getStepTime();


		/**
		* @brief Returns the target waypoint of the active move
		*
		* This is different from the target tcp pose which returns the target pose for each time step.
		* The get_target_waypoint() returns the same target pose for movel, movej, movep or movec during the motion. It
		* returns the target tcp pose, if none of the mentioned move functions are running.
		*
		* This method is useful for calculating relative movements where the previous move command uses blends.
		*
		* @returns The desired waypoint TCP vector [X, Y, Z, Rx, Ry, Rz] or and empty
		*          vector in case of an error.
		*/
		RTSI_EXPORT std::vector<double> getTargetWaypoint();

		/**
		* @brief Sets the active tcp offset, i.e. the transformation from the output flange coordinate system to the
		* TCP as a pose.
		* @param tcp_offset A pose describing the transformation of the tcp offset.
		*/
		RTSI_EXPORT bool setTcp(const std::vector<double> &tcp_offset);


		/**
		* @brief Calculate the forward kinematic  (joint space -> tool
		* space) using the calibrated robot kinematics. If no joint position vector
		* is provided the current joint angles of the robot arm will be used. If no
		* tcp is provided the currently active tcp of thtransformatione controller will be used.
		*
		* NOTICE! If you specify the tcp_offset you must also specify the q.
		*
		* @param q joint position vector (Optional)
		* @param tcp_offset tcp offset pose (Optional)
		* @returns the forward kinematic transformation as a pose
		*/
		RTSI_EXPORT std::vector<double> getForwardKinematics(const std::vector<double> &q = {},
			const std::vector<double> &tcp_offset = {});

		/**
		* @brief Calculate the inverse kinematic transformation (tool space -> jointspace). If qnear is defined, the
		* solution closest to qnear is returned.Otherwise, the solution closest to the current joint positions is returned.
		* If no tcp is provided the currently active tcp of the controller will be used.
		* @param x tool pose
		* @param qnear list of joint positions (Optional)
		* @param maxPositionError the maximum allowed positionerror (Optional)
		* @param maxOrientationError the maximum allowed orientationerror (Optional)
		* @returns joint positions
		*/
		RTSI_EXPORT std::vector<double> getInverseKinematics(const std::vector<double> &x,
			const std::vector<double> &qnear = {},
			double max_position_error = 1e-10,
			double max_orientation_error = 1e-10);


		/**
		* @brief Pose transformation to move with respect to a tool or w.r.t. a custom feature/frame
		* The first argument, p_from, is used to transform the second argument,
		* p_from_to, and the result is then returned. This means that the result is the resulting pose, when starting at
		* the coordinate system of p_from, and then in that coordinate system moving p_from_to.
		* This function can be seen in two different views. Either the function transforms, that is translates and rotates,
		* p_from_to by the parameters of p_from. Or the function is used to get the resulting pose, when first
		* making a move of p_from and then from there, a move of p_from_to.
		* If the poses were regarded as transformation matrices, it would look like:
		* @verbatim
		* T_world->to = T_world->from * T_from->to
		* T_x->to = T_x->from * T_from->to
		* @endverbatim
		* @param p_from starting pose (spatial vector)
		* @param p_from_to pose change relative to starting pose (spatial vector)
		* @returns resulting pose (spatial vector)
		*/
		RTSI_EXPORT std::vector<double> poseTrans(const std::vector<double> &p_from, const std::vector<double> &p_from_to);


		/**
		* @brief Triggers a protective stop on the robot. Can be used for testing and debugging.
		*/
		bool triggerProtectiveStop();

		/**
		* @brief Enable a watchdog for the communication with a specified minimum frequency for which an input update is
		* expected to arrive. The watchdog is useful for safety critical realtime applications eg. servoing. The default
		* action taken is to shutdown the control, if the watchdog is not kicked with the minimum frequency.
		*
		* Preferably you would call this function right after the rtsiControlInterface has been constructed.
		*
		* @param min_frequency The minimum frequency an input update is expected to arrive defaults to 10Hz.
		*/
		RTSI_EXPORT bool setWatchdog(double min_frequency = 10.0);


		/**
		* @brief Kicks the watchdog safeguarding the communication. Normally you would kick the watchdog in your control
		* loop. Be sure to kick it as often as specified by the minimum frequency of the watchdog.
		*/
		bool kickWatchdog();


		/**
		* @brief Returns the torques of all joints
		*
		* The torque on the joints, corrected by the torque needed to move the
		* robot itself (gravity, friction, etc.), returned as a vector of length 6.
		*
		* @returns The joint torque vector in Nm: [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
		*/
		RTSI_EXPORT std::vector<double> getJointTorques();

		/**
		* @brief Gets the active tcp offset, i.e. the transformation from the output flange coordinate system to the TCP
		* as a pose.
		*
		* @returns the TCP offset as a pose
		*/
		RTSI_EXPORT std::vector<double> getTCPOffset();

		/***************************
		@brief      : 得到传感器的原始力
		@param[out] :　原始力
		@time       : 2023/04/20
		****************************/
		RTSI_EXPORT std::vector<double> getSensorForce();

		/***************************
		@brief      : 获取TCP坐标系下的六维力
		@return     :
		@time       : 2023/04/20
		****************************/
		RTSI_EXPORT std::vector<double> getTCPForce();

		/***************************
		@brief      : 获取末端工具的质量、质心
		@return     :｛cog,cogX,cogY,cogZ｝
		@time       : 2023/04/20
		****************************/
		RTSI_EXPORT std::vector<double> getToolPayload();



	private:
		bool setupRecipes(const double &frequency);

		void initOutputRegFuncMap();

		bool sendCommand(const RTSI::RobotCommand &cmd);

		void sendClearCommand();

		int getControlScriptState();

		bool isProtectiveStopped();

		bool isEmergencyStopped();

		int getToolContactValue();

		double getStepTimeValue();

		std::vector<double> getTargetWaypointValue();

		std::vector<double> getActualJointPositionsHistoryValue();

		std::vector<double> getInverseKinematicsValue();

		std::vector<double> poseTransValue();

		void verifyValueIsWithin(const double &value, const double &min, const double &max);


		void receiveCallback();

		/***************************
		@brief      : 此函数等待脚本程序运行。如果程序在一段时间后没有运行，则函数尝试重新发送脚本。如果超时后脚本未运行
		时间，抛出异常
		@time       : 2023/04/12
		****************************/
		void waitForProgramRunning();

		std::string outDoubleReg(int reg) const
		{
			return "output_double_register" + std::to_string(m_register_offset + reg);
		};

		std::string outIntReg(int reg) const
		{
			return "output_int_register" + std::to_string(m_register_offset + reg);
		};

		std::string inDoubleReg(int reg) const
		{
			return "input_double_register" + std::to_string(m_register_offset + reg);
		};

		std::string inIntReg(int reg) const
		{
			return "input_int_register" + std::to_string(m_register_offset + reg);
		};

		double getOutputDoubleReg(int reg)
		{
			std::string func_name = "getOutput_double_register" + std::to_string(m_register_offset + reg);
			return m_output_reg_func_map[func_name]();
		};

		int getOutputIntReg(int reg)
		{
			std::string func_name = "getOutput_int_register" + std::to_string(m_register_offset + reg);
			return m_output_reg_func_map[func_name]();
		};
	private:
		std::string m_hostname;
		int m_port;
		bool m_upload_script;
		bool m_use_external_control_cs_cap;
		bool m_verbose;
		bool m_use_upper_range_registers;
		bool m_no_wait;
		bool m_custom_script;
		bool m_custom_script_running;
		int m_cs_cap_port;
		double m_frequency;
		double m_delta_time;
		int m_register_offset;
		std::shared_ptr<RTSI> mp_rtsi;
		std::atomic<bool> m_stop_thread{ false };
		std::shared_ptr<boost::thread> mp_th;
		std::shared_ptr<DashboardClient> mp_db_client;
		std::shared_ptr<ScriptClient> mp_script_client;
		std::shared_ptr<RobotState> mp_robot_state;
		std::map<std::string, std::function<double()>> m_output_reg_func_map;

	};

}