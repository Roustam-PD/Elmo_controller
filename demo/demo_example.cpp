#include <iostream>
#include <mqueue.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "../shared_interface.h"

mqd_t   cmdQueue, 
		statQueue;

		EngineInfo info;

// Send a command to the queue
bool sendCommand(int eng_id, Command type, int value) {
	EngineCommand cmd;
	cmd.eng_id = eng_id;
	cmd.cmd_id = type;
	cmd.param = value;
	if (mq_send(cmdQueue, (const char*)&cmd, sizeof(EngineCommand), 0) == -1) {
		perror("mq_send");
		return false;
	}
	return true;
}

// Try to read status if available
bool readStatus(EngineInfo& info, bool blocking) {
	struct mq_attr attr;

	if (blocking) {

		ssize_t bytes = mq_receive(statQueue, (char *)&info, sizeof(EngineInfo), nullptr);
		if (bytes == -1) {
			if (errno == EAGAIN) {
				std::cout << "[Status] No status message available." << std::endl;
				return false; 
			} else {
				perror("mq_receive");
				return false;
			}
		}
	} else {

		if (mq_getattr(statQueue, &attr) == -1) {
			perror("mq_getattr");
			return false;
		}

		if (attr.mq_curmsgs == 0) {
			std::cout << "[Status] No messages in queue." << std::endl;
			return false; 
		}

		ssize_t bytes = mq_receive(statQueue, (char *)&info, sizeof(EngineInfo), nullptr);
		if (bytes == -1) {
			perror("mq_receive");
			return false; 
		}
	}

	return true; 
}



int main() {

	int eng_id = 1; // Use motor with ID: #1 (e.t: 0,1,2,3 .. by order, not a node!)

	cmdQueue = mq_open(CMD_QUEUE_NAME, O_WRONLY);
	if (cmdQueue == -1) {
		perror("mq_open cmdQueue");
		return 1;
	}
	
	statQueue = mq_open(DATA_QUEUE_NAME, O_RDONLY);
	if (statQueue == -1) {
		perror("mq_open statQueue");
		return 1;
	}


	// Send START
	std::cout << "Sending START for engine ID: " << eng_id << "..." << std::endl;
		sendCommand(eng_id, Command::Init, 0);
			sleep(1);
				readStatus(info,false);

	// Send SET_SPEED_MODE
	std::cout << "Sending Mode by SPEED for engine ID: " << eng_id << "..." << std::endl;
		sendCommand(eng_id, Command::ModeBySpeed, 0);
			sleep(1);
				readStatus(info,false);

	// Send SET_SPEED_VALUE
	std::cout << "Sending SPEED 30 RPM for engine ID: " << eng_id << "..." << std::endl;
		sendCommand(eng_id, Command::SetSpeed, 30);
			sleep(1);
				readStatus(info,false);

	// Send SET_ANGLE_MODE
	std::cout << "Sending Mode by Angle for engine ID: " << eng_id << "..." << std::endl;
		sendCommand(eng_id, Command::ModeByAngle, 0);
			sleep(1);
				readStatus(info,false);

	// Send SET_ANGLE_VALUE
	std::cout << "Sending Angle 45 DEGREE for engine ID: " << eng_id << "..." << std::endl;
		sendCommand(eng_id, Command::SetAngle, 45);
			sleep(1);
				readStatus(info,false);

	// Send STOP
	std::cout << "Sending STOP to  engine ID: " << eng_id << "..." << std::endl;
		sendCommand(eng_id, Command::DeInit, 0);
			sleep(1);
				readStatus(info,false);

	// Cleanup
	mq_close(cmdQueue);
	mq_close(statQueue);

	return 0;
}
