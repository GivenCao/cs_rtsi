#include "dashboard_client.h"
#include "rtsi.h"
#include "rtsi_receive_interface.h"
#include "rtsi_control_interface.h"
#include "script_client.h"



int main()
{
	//GLogger::init("E:\\mylog.txt", 1, false);

	//cs_rtsi::RTSIReceiveInterface recv("192.168.40.27");
	//cs_rtsi::RTSIControlInterface control("192.168.40.27");
	cs_rtsi::RTSIControlInterface control("192.168.40.27");
	std::vector<double> speed = { 0, 0, 0,0.05, 0, 0 };
	std::vector<double> speed_vector = { 0.0, 0.0, 0.1, 0, 0.0, 0.0 };
	control.speedJ({ 0.5,0.1,0.1,0.1,0.1,0.1 }, 0.1);
	cs_rtsi::DashboardClient dash("192.168.40.27");
	while (true)
	{
		if (!dash.isConnected())
		{
			if (dash.connect())
			{
				std::string str = dash.robotMode();

				std::cout << str << std::endl;
			}
			else
			{
				continue;
			}

		}
		else
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			std::string str = dash.robotMode();

			std::cout << str << std::endl;

		}


	}


	cs_rtsi::RTSIReceiveInterface recv("192.168.40.27");





	int count = 0;

	while (true)
	{


		//rtsi_control.setTcp({ -0.06932, 0.0000, 0.1485, 0, 0, 0 });
		std::vector<double> direction = { 1,0.0,0.0,0.0,0.0,0.0 };
		// std::vector<double> direction = rtsiStade.getActualTCPSpeed();
		// std::cout << direction[0] << " " << direction[1] << " " << direction[2] << " " << direction[3] << " "
		// << direction[4] << " " << direction[5] << std::endl;
		//rtsi_control.moveUntilContact(speed, direction);
		//double speed_magnitude = 0.15;

		control.jogStart(speed, cs_rtsi::RTSIControlInterface::FEATURE_TOOL);
		//rtsi_control.jogStart(speed, rtsiControlInterface::FEATURE_TOOL);
		//
		//std::this_thread::sleep_for(std::chrono::milliseconds(2));
		count++;
		std::cout << count << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		if (count == 10)
		{
			break;
		}
	}
	control.jogStop();




	std::vector<double>force1 = recv.getActualTCPForce();
	std::vector<double>force2 = recv.getActualSensorForce();
	std::vector<double>force = control.getTCPForce();
	//std::vector<double>force2 = control.getSensorForce();
	std::cout << "¶©ÔÄ×Ö¶Î¶Ë£º"
		<< force1[0] << ","
		<< force1[1] << ","
		<< force1[2] << ","
		<< force1[3] << ","
		<< force1[4] << ","
		<< force1[5] << ","
		<< std::endl;
	std::cout << "¼Ä´æÆ÷½Ó¿Ú¶Ë£º"
		<< force2[0] << ","
		<< force2[1] << ","
		<< force2[2] << ","
		<< force2[3] << ","
		<< force2[4] << ","
		<< force2[5] << ","
		<< std::endl;
	//std::cout << "¼Ä´æÆ÷½Ó¿Ú¶Ëget_tcp_force()£º"
	//	<< force2[0] << ","
	//	<< force2[1] << ","
	//	<< force2[2] << ","
	//	<< force2[3] << ","
	//	<< force2[4] << ","
	//	<< force2[5] << ","
	//	<< std::endl;
	std::vector<double>qq0 = recv.getActualQ();
	int res = recv.getConState();
	std::vector<double>pose1 = recv.getActualTCPPose();



	//std::vector<double>force2 = control.getSensorForce();
	control.moveL({ -0.5569,-0.1366,0.8168,-0.684,-0.0631,-0.1521 },0.2,0.1);
	//cs_rtsi::DashboardClient dash("192.168.40.27", 29999, true);
	dash.connect();
	//dash.powerOn();
	dash.brakeRelease();
	std::string str=dash.robotMode();
	//cs_rtsi::RTSI rtsi("192.168.40.111");
	//cs_rtsi::ScriptClient script("192.168.40.111",2,4);
	//script.connect();
	//script.sendScript("E:\\Given\\RobotControl\\cs_robot_script.script");

	std::vector<double> qq = control.getInverseKinematics({ -0.11039,-0.53123,0.30594,1.855,-0.352,0.528 });
	std::vector<double> pose = control.getForwardKinematics({ -1.6,-2.03,-2.01,0.57,2.21,-2.98 });
	res = recv.getConState();
	control.disconnect();
	res = recv.getConState();
	control.zeroFtSensor();
	std::vector<double> mas_cog = control.getToolPayload();
	std::vector<double>q = { 2.02,-2.30,1.72,-1.44,-0.38,0.28 };
	control.moveJ(q, 0.5, 0.1);

	
	uint32_t status = recv.getRobotStatusBits();
	int32_t mode=recv.getSafetyMode();
	std::vector<double> actualQ = recv.getActualQ();
	std::vector<double> actualP = recv.getActualTCPPose();

	//rtsi.connect();
	//rtsi.negotiateProtocolVersion();
	//auto controller_version = rtsi.getControllerVersion();
	//dash.connect();
	//dash.powerOn();
	//dash.brakeRelease();
	//dash.clear();
	////dash.play();
	//dash.unlockProtectiveStop();
	//dash.stop();
	//std::string str2=dash.polyscopeVersion();
	//dash.speed(100);
	//std::string str = dash.robotMode();
	//std::string str1 = dash.safetystatus();
	//dash.powerOff();
	//dash.shutdown();
	return 0;
}