#include "dashboard_client.h"
#include "rtsi.h"
#include "rtsi_receive_interface.h"
#include "rtsi_control_interface.h"
#include "script_client.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>

using namespace std;

int main()
{
	//GLogger::init("E:\\mylog.txt", 1, false);

	//cs_rtsi::RTSIReceiveInterface recv("192.168.40.27");
	//cs_rtsi::RTSIControlInterface control("192.168.40.27");
	std::string host = "172.21.113.154";
	std::string dash_host = "127.0.0.1";
	cs_rtsi::RTSIControlInterface control(host);
	std::vector<double> speed = { 0, 0, 0,0.05, 0, 0 };
	std::vector<double> speed_vector = { 0.0, 0.0, 0.1, 0, 0.0, 0.0 };
	//control.speedJ({ 0.5,0.1,0.1,0.1,0.1,0.1 }, 0.1);

	//ofstream outfile;
	//outfile.open(R"(D:\Work\CS\Code\RTSI\cs_rtsi\test\test_data\tcp_path_2.txt)", ios::out | ios::app);  // 需要目录下有该文件
	//if (!outfile.is_open())
	//{
	//	cout << "read outfile failed" << endl;
	//	return -1;
	//}

	while (true) {

		ifstream readFile;
		readFile.open(R"(D:\Work\CS\Code\RTSI\cs_rtsi\test\test_data\tcp_path_2.txt)");
		//readFile.open(R"(D:\Work\CS\Code\RTSI\cs_rtsi\test\test_data\joint_path.txt)");
	
		vector<string> file_in;
		if (readFile) {
			cout << "文件打开成功！" << endl;
			string buf;
			char sep = ' ';
			while (getline(readFile, buf)) {
				int index1 = buf.find_first_of(sep);     //查找逗号分隔符        
				string str1 = buf.substr(0, index1);
				string leftstring = buf.substr(index1 + 1, buf.length());

				int index2 = leftstring.find_first_of(sep);
				string str2 = leftstring.substr(0, index2);
				leftstring = leftstring.substr(index2 + 1, leftstring.length());

				int index3 = leftstring.find_first_of(sep);
				string str3 = leftstring.substr(0, index3);
				leftstring = leftstring.substr(index3 + 1, leftstring.length());

				int index4 = leftstring.find_first_of(sep);
				string str4 = leftstring.substr(0, index4);
				leftstring = leftstring.substr(index4 + 1, leftstring.length());

				int index5 = leftstring.find_first_of(sep);
				string str5 = leftstring.substr(0, index5);
				leftstring = leftstring.substr(index5 + 1, leftstring.length());

				int index6 = leftstring.find_first_of(sep);
				string str6 = leftstring.substr(0, index6);
				leftstring = leftstring.substr(index6 + 1, leftstring.length());


				//float x = std::stof(str1);  // 字符串转浮点型
				//float y = std::stof(str2);
				//float z = std::stof(str3);
				vector<double> data;
				double j0 = std::stod(str1);// 字符串转双精度
				double j1 = std::stod(str2);
				double j2 = std::stod(str3);
				double j3 = std::stod(str4);// 字符串转双精度
				double j4 = std::stod(str5);
				double j5 = std::stod(str6);
				data.push_back(j0);
				data.push_back(j1);
				data.push_back(j2);
				data.push_back(j3);
				data.push_back(j4);
				data.push_back(j5);
				//vector<double> joints = data;
				//vector<double> joints = control.getInverseKinematics(data);
				//std:vector<double> tcp = control.getForwardKinematics(joints);
				//joints = control->getInverseKinematics(joints);
				//point3 p{ xx,yy,zz };
				//vec3.push_back(p);
				//outfile << tcp[0] << sep << tcp[1] << sep << tcp[2] << sep << tcp[3] << sep << tcp[4] << sep << tcp[5] << endl;  //输出到文件
				control.servoL(data, 0, 0, 0.08, 0.2, 500);
				Sleep(70);
				//time_t timep;
				//time(&timep);
				//char tmp[64];
				//strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&timep));
				//cout << "currentTime is " << tmp << "." << endl;
				// file_in.push_back(temp);
			}
		}
		else {
			cerr << "文件打开失败!" << endl;
		}
		readFile.close();
		//打印读取的输出
		for (const auto& i : file_in)
			cout << i << endl;
	}


	cs_rtsi::DashboardClient dash(dash_host);
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
			if (str.find("RUNNING")!=-1 ) {
				break;
			}

		}


	}


	cs_rtsi::RTSIReceiveInterface recv(host);





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
	std::cout << "订阅字段端："
		<< force1[0] << ","
		<< force1[1] << ","
		<< force1[2] << ","
		<< force1[3] << ","
		<< force1[4] << ","
		<< force1[5] << ","
		<< std::endl;
	std::cout << "寄存器接口端："
		<< force2[0] << ","
		<< force2[1] << ","
		<< force2[2] << ","
		<< force2[3] << ","
		<< force2[4] << ","
		<< force2[5] << ","
		<< std::endl;
	//std::cout << "寄存器接口端get_tcp_force()："
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