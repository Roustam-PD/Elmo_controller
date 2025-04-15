/*
 * motor_control.cpp
 *
 * This program provides a control interface for managing motors over a CAN (Controller Area Network) bus.
 * It initializes communication with multiple motor nodes, sends control commands, and receives motor status updates.
 *
 * Key components:
 * - Initialization of CAN interface and validation of its availability.
 * - Parsing command-line arguments for setting interface name and motor node addresses.
 * - Sending control commands (e.g., Init, SetMode, SetSpeed) to motors via a dedicated thread.
 * - Receiving motor status (e.g., speed, angle) from the CAN bus and forwarding it via POSIX message queues.
 * - Uses POSIX threads for concurrent read/write access to CAN and thread-safe operations.
 *
 * Message queues are used for inter-thread communication:
 * - /motorcmd: queue for sending commands to motors.
 * - /motordata: queue for receiving motor data.
 *
 * The program supports graceful shutdown via a special shutdown command.
 *
 * Dependencies:
 * - Linux CAN utilities (can.h, can/raw.h)
 * - POSIX threads and message queues
 * - C++ STL (string, vector, unordered_map, etc.)
 *
 * Usage example:
 * ./motor_interface -i can0 -a 0x01,0x02
 *
 * Author: Roustam R. Akhmetzyanov
 * Company: Pass-Dynamics.com
 */

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <getopt.h>
#include <time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

#include <pthread.h>
#include <sys/syscall.h>
#include <mqueue.h>
#include <cstring>
#include <unordered_map>
#include <fcntl.h>
#include <errno.h> 

#include "shared_interface.h"

/* Global vars */

std::string interface;
std::vector<int> node_id;
std::vector<EngineInfo> engine_info;
std::unordered_map<int, int> nodeid_idx;

int can_socket = -1;
pthread_mutex_t can_mutex  = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t stop_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t stop_cond   = PTHREAD_COND_INITIALIZER;
bool stop_requested = false;



void print_help() {
	std::cout << "Usage: ./motor_interface -i <interface> -a <addr1,addr2,...>\n";
	std::cout << "   or: ./motor_interface if=<interface> addr=<addr1,addr2,...>\n";
	std::cout << "Options:\n";
	std::cout << "  -i, --interface   Set CAN interface (e.g. can0)\n";
	std::cout << "  -a, --addresses   Set comma-separated CAN addresses\n";
	std::cout << "  -h, --help        Show this help message\n";
}

int parse_address(const std::string& token) {
	int val;
	std::stringstream ss(token);
	if (token.find("0x") == 0 || token.find("0X") == 0) {
		ss >> std::hex >> val;
	} else {
		ss >> val;
	}
	return val;
}


// Function to set CAN bitrate
// bool set_can_bitrate(const std::string& interface, int bitrate) {
	// Создание сокета для CAN
	// int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	// if (sock < 0) {
	// 	std::cerr << "Error: Unable to create socket.\n";
	// 	return false;
	// }

	// struct ifreq ifr;
	// memset(&ifr, 0, sizeof(struct ifreq));
	// strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);

	// // Get interface index
	// if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
	// 	std::cerr << "Error: Unable to get interface index.\n";
	// 	close(sock);
	// 	return false;
	// }

	// // Get current bitrate settings of the interface
	// struct can_bittiming bittiming;
	// if (can_get_bittiming(sock, ifr.ifr_ifindex, &bittiming) < 0) {
	// 	std::cerr << "Error: Unable to get current bitrate.\n";
	// 	close(sock);
	// 	return false;
	// }

	// Set new bitrate
	// bittiming.bitrate = bitrate;
	// if (can_set_bittiming(sock, ifr.ifr_ifindex, &bittiming) < 0) {
	// 	std::cerr << "Error: Unable to set bitrate for CAN interface " << interface << ".\n";
	// 	close(sock);
	// 	return false;
	// }

	// std::cout << "CAN interface " << interface << " bitrate set to " << bitrate << " bps.\n";
	// close(sock);
// 	return true;
// }


// Function to check and configure CAN interface

bool check_can_interface(const std::string& interface) {
    // Create socket
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        std::cerr << "Error: Unable to create socket.\n";
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);

    // Check if interface is available
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0) {
        std::cerr << "Error: Unable to get flags for CAN interface " << interface << ".\n";
        close(sock);
        return false;
    }

    // // Check if interface is UP
    if (!(ifr.ifr_flags & IFF_UP)) {
        std::cerr << "Error: CAN interface " << interface << " is DOWN.\n";
		std::cerr << "Exitting :: Activate it like this: sudo ip link set " << interface << " up type can bitrate 500000" << ".\n";
        close(sock);
        return false;
    }

	// Get interface index
    int if_index = if_nametoindex(interface.c_str());
    if (if_index == 0) {
        std::cerr << "Exitting :: Error : Unable to get interface index for " << interface << ".\n";
        close(sock);
        return false;
    }

    // Bind socket to interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_index;

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Exitting :: Error : Unable to bind to CAN interface " << interface << ".\n";
        close(sock);
        return false;
    }

    std::cout << "CAN interface [" << interface << "] check passed: available and operational.\n";
    close(sock);
    return true;
}

// Implementation of function
void send_CAN_msg(int can_id, std::initializer_list<uint8_t> data_list) {
// void send_CAN_msg(int can_id, uint8_t* data_list, size_t data_size) {

	// CAN message structure
    struct can_frame frame = {};
    frame.can_id = can_id;
    // frame.can_dlc = data_size;
    // memcpy(&frame.data[0], data, data_size );

    frame.can_dlc = std::min(data_list.size(), size_t(8));
    std::copy(data_list.begin(), data_list.begin() + frame.can_dlc, frame.data);

    pthread_mutex_lock(&can_mutex);

    ssize_t bytes_written = write(can_socket, &frame, sizeof(frame));
    if (bytes_written == -1) {
        perror("ERROR :: CAN write :: send_CAN_msg");
    }

    pthread_mutex_unlock(&can_mutex);

}


// === Command sending thread ===
void* can_write_thread(void*) {

	// Queue attributes
	struct mq_attr attr;
	attr.mq_flags   =  0;						// Queue flags (default)
	attr.mq_maxmsg  = 10;						// Max number of messages
	attr.mq_msgsize = sizeof(EngineCommand);	// Max message size
	attr.mq_curmsgs =  0;						// Number of messages in queue (default)

	mqd_t cmd_queue = mq_open(CMD_QUEUE_NAME, O_RDWR | O_CREAT, 0644, &attr);
	if (cmd_queue == -1) {
		perror("mq_open write");
		stop_requested = true;
		return nullptr;
	}
	
	printf("Outgoing thread was started, TID: %lu\n", syscall(SYS_gettid) );
	EngineCommand cmd;
	while (true) {

		// Set timeout: 1 second
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec += 1;  // When timeout reached, print dbg message  and continue

		ssize_t bytes = mq_timedreceive(cmd_queue, (char*)&cmd, sizeof(EngineCommand), nullptr, &ts);

		if (bytes == -1) {
			if (errno == ETIMEDOUT) {
				// If timeout reached, print message and continue
				// printf("Timeout reached, queue is empty.\n");
				continue;
			} else {
				// If error not related to timeout, log and break
				perror("mq_timedreceive failed");
				break;  // Quit the cycle
			}
		}

			if (bytes > 0) {
			// Special command — shutdown
			if (cmd.eng_id  ==  SHUTDOWN_ID) {
				pthread_mutex_lock(&stop_mutex);
				stop_requested = true;
				pthread_cond_signal(&stop_cond);
				pthread_mutex_unlock(&stop_mutex);
				break;
			}

		switch (cmd.cmd_id)
		{
			case Command::Init:
				send_CAN_msg(TPDO1 | nodeid_idx[cmd.eng_id], {1,2,3,4});
				break;
		
			case Command::DeInit:
				send_CAN_msg(TPDO1 | nodeid_idx[cmd.eng_id], {5,6,7,8,9});
				break;
		
			case Command::ModeByAngle :
				send_CAN_msg(SDO_REQ | nodeid_idx[cmd.eng_id], {11,12,13,14,cmd.param});
				break;
			
			case Command::ModeBySpeed:
				send_CAN_msg(SDO_REQ | nodeid_idx[cmd.eng_id], {16,17,cmd.param});
				break;
		
			case Command::SetMode:
				send_CAN_msg(SDO_REQ | nodeid_idx[cmd.eng_id], {19,cmd.param});
				break;
		
			case Command::SetSpeed:
				send_CAN_msg(SDO_REQ | nodeid_idx[cmd.eng_id], {21,cmd.param});
				break;

			case Command::SetAngle:
				send_CAN_msg(SDO_REQ | nodeid_idx[cmd.eng_id], {23,cmd.param});
				break;

			default:
				break;

		}


		}
		usleep(10000);
	}

	stop_requested = true;
	mq_close(cmd_queue);
	return nullptr;
}


// === Receiving thread – motor state – non-blocking read mode ===
void* can_read_thread(void*) {

    // Queue attributes
    struct mq_attr attr;
    attr.mq_flags  =  0;					// Queue flags (default)
    attr.mq_maxmsg = 10;					// Max number of incoming messages
    attr.mq_msgsize = sizeof(EngineInfo);	// Max message size
    attr.mq_curmsgs = 0;					// Number of messages in queue (default)
	
	
	mqd_t data_queue = mq_open(DATA_QUEUE_NAME, O_WRONLY | O_CREAT, 0644, &attr);
   if (data_queue == -1) {
        perror("mq_open read");
        return nullptr;
    }

    // Set CAN socket to non-blocking mode
    int flags = fcntl(can_socket, F_GETFL, 0);
    if (flags == -1) {
        perror("fcntl F_GETFL failed");
        mq_close(data_queue);
        return nullptr;
    }

    if (fcntl(can_socket, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl F_SETFL failed");
        mq_close(data_queue);
        return nullptr;
    }

	printf("Incoming thread was started, TID: %lu\n", syscall(SYS_gettid) );

    can_frame frame;
	
    while (true) {
		
        // Check shutdown request
        pthread_mutex_lock(&stop_mutex);
	    bool should_stop = stop_requested;
        pthread_mutex_unlock(&stop_mutex);
        if (should_stop) break;

        // Non-blocking read from CAN with shared access
        pthread_mutex_lock(&can_mutex);
        ssize_t bytes = read(can_socket, &frame, sizeof(frame));
        pthread_mutex_unlock(&can_mutex);

        if (bytes > 0) {
			
			if (nodeid_idx.find(frame.can_id) != nodeid_idx.end()) {

				int can_idx = nodeid_idx[frame.can_id];

				EngineInfo *m_state = &engine_info[can_idx];

				m_state->speed  = (frame.data[0] << 8) | frame.data[1];	// Fake data

				clock_gettime(CLOCK_REALTIME, &m_state->last);			// send real time stamp

				// Try to send message to queue, but do not wait if queue is full
				int ret;
				while ((ret = mq_send(data_queue, (char*)m_state, sizeof(EngineInfo), 0)) == -1 && errno == EAGAIN) {
					usleep(100000); // Wait a bit (100 ms) before retry
				}

				if (ret == -1) {
					perror("mq_send failed");
					// just log the error
				}
		}
	}

        usleep(10000); // Small delay to avoid CPU overload
    }

    mq_close(data_queue);
    return nullptr;
}


int main(int argc, char* argv[]) {

	struct option long_options[] = {
		{"help", no_argument, nullptr, 'h'},
		{"interface", required_argument, nullptr, 'i'},
		{"addresses", required_argument, nullptr, 'a'},
		{nullptr, 0, nullptr, 0}
	};

	int opt;
	optind = 1;

	while ((opt = getopt_long(argc, argv, "hi:a:", long_options, nullptr)) != -1) {
		switch (opt) {
			case 'h':
				print_help();
				return 0;
			case 'i':
				if (!interface.empty()) {
					std::cerr << "Error: Only one interface allowed.\n";
					return 1;
				}
				interface = optarg;
				break;
			case 'a': {
				std::stringstream ss(optarg);
				std::string token;
				while (std::getline(ss, token, ',')) {
					int val = parse_address(token);
					node_id.push_back(val);
				}
				break;
			}
			default:
				print_help();
				return 1;
		}
	}

	for (int i = optind; i < argc; ++i) {
		std::string arg = argv[i];
		if (arg.find("if=") == 0 && interface.empty()) {
			interface = arg.substr(3);
		} else if (arg.find("addr=") == 0) {
			std::string addr_str = arg.substr(5);
			std::stringstream ss(addr_str);
			std::string token;
			while (std::getline(ss, token, ',')) {
				int val = parse_address(token);
				node_id.push_back(val);
			}
		} else {
			std::cerr << "Unknown or malformed argument: " << arg << "\n";
			print_help();
			return 1;
		}
	}

	if (interface.empty()) {
		std::cerr << "Error: At least one CAN interface is required\n";
		return 1;
	}

	if (node_id.empty()) {
		std::cerr << "Error: At least one CAN address is required\n";
		return 1;
	}

	std::cout << "Using Interface: " << interface << "\nOpenCAN Nodes ID: ";
	for (int addr : node_id) {
		std::cout << "0x" << std::hex << addr << " ";
	}
	std::cout << std::dec << "\n";


	if (!check_can_interface(interface /*, BITRATE */)) {
		// Errir while checking
		std::cerr << "Failed to use  interface \'" << interface << "\'.\n";
		return -1;
	} else {
		// Interface exist and functonal
		std::cout << "Interface " << interface << " was initilized and ready for use.\n";
	}

	/* ----------------------------------------------------------------------*/
	/*                    -=|[ Operational part ]|=-                         */
	/* ----------------------------------------------------------------------*/

	// CaN interface setup
	can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (can_socket < 0) {
		perror("socket");
		return 1;
	}

	engine_info.resize(node_id.size());
	for (int i = 0; i < node_id.size(); ++i){
        nodeid_idx[node_id[i]] = i;
		engine_info[i].nod_id = node_id[i];
		engine_info[i].eng_id = i;
	}

	ifreq ifr;
	strcpy(ifr.ifr_name, interface.c_str());  // should be like "can0"
	ioctl(can_socket, SIOCGIFINDEX, &ifr);

	sockaddr_can addr = {};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(can_socket, (sockaddr*)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	// Threads
	pthread_t writer_thread, reader_thread;
	pthread_create(&writer_thread, nullptr, can_write_thread, nullptr);
	pthread_create(&reader_thread, nullptr, can_read_thread, nullptr);
	printf("Main thread suspended now, TID: %lu\n", syscall(SYS_gettid) );

	// Multithread lock
	pthread_mutex_lock(&stop_mutex);
	while (!stop_requested)
		pthread_cond_wait(&stop_cond, &stop_mutex);
	pthread_mutex_unlock(&stop_mutex);

	// Termitation sync
	pthread_join(writer_thread, nullptr);
	pthread_join(reader_thread, nullptr);
	
	mq_unlink(CMD_QUEUE_NAME);
	mq_unlink(DATA_QUEUE_NAME);
	
	close(can_socket);

	return 0;
}
