
#define CMD_QUEUE_NAME  "/motorcmd"
#define DATA_QUEUE_NAME "/motordata"
#define MAX_MSG_SIZE 256
#define SHUTDOWN_ID -1
#define BITRATE 500000

// OpenCAN messages types
#define TPDO1		0x180
#define RPDO1		0x200
#define SDO_ANS		0x580
#define SDO_REQ		0x600

enum Command : int {
	Init,
	SetMode,
	SetSpeed,
	SetAngle,
	DeInit,
	ModeBySpeed,
	ModeByAngle,
	ModeByTorque
};

struct EngineCommand {
	uint8_t eng_id;
	Command cmd_id;
	uint8_t param;
};

struct EngineInfo {
	int eng_id;
	int	nod_id;
	uint8_t eng_sn[8];
	int angle;
	int speed;
	int current;
	int voltage;
	struct timespec last;
};
