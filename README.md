# Motor Control Interface for CAN Bus

This repository provides a C++ program for managing motors over a CAN (Controller Area Network) bus. The program facilitates communication with multiple motor nodes, sends control commands, and receives status updates in a multithreaded environment using POSIX message queues for inter-thread communication.

## Key Features

- **CAN Interface Initialization**: Verifies the availability of the specified CAN interface.
- **Command Parsing**: Accepts command-line arguments to set the CAN interface and motor node addresses.
- **Motor Control**: Sends commands to motors via CAN messages, such as initializing, setting modes, and adjusting speeds.
- **Motor Status Monitoring**: Receives motor status updates like speed and angle, forwarding them through message queues.
- **Multithreading**: Utilizes POSIX threads for concurrent reading and writing to CAN, with thread-safe operations protected by mutexes.
- **Graceful Shutdown**: Supports a shutdown command to cleanly terminate the program.

## Dependencies

- **Linux CAN Utilities**: `can.h`, `can/raw.h`
- **POSIX Threads** and **Message Queues** (`pthread.h`, `mqueue.h`)
- **C++ STL**: `string`, `vector`, `unordered_map`, etc.

## Usage

### Command-Line Options

```
Usage: ./motor_interface -i <interface> -a <addr1,addr2,...>
   or: ./motor_interface if=<interface> addr=<addr1,addr2,...>
Options:
  -i, --interface   Set CAN interface (e.g. can0)
  -a, --addresses   Set comma-separated node addresses
  -h, --help        Show this help message
```

### Example

To run the program with the `can0` interface and motor nodes at addresses `0x01` and `0x02`, use the following command:

```
./motor_interface -i can0 -a 0x01,0x02
```

## Program Workflow

### Command Sending Thread

The command sending thread listens for messages from a queue (`/motorcmd`) and sends corresponding CAN commands. These commands can be for initializing, de-initializing, or adjusting the mode and speed of motors.

### Motor State Receiving Thread

The receiving thread listens for motor status messages on the CAN bus and forwards the data (e.g., speed, angle) to a message queue (`/motordata`). It operates in a non-blocking mode to avoid stalling the program.

### CAN Message Handling

The program uses a CAN socket to send and receive messages. It supports standard CAN frames and allows specifying CAN IDs, as well as the data to be sent. The CAN interface is checked for availability before establishing communication.

### Graceful Shutdown

The program supports a shutdown procedure through a special command, ensuring that all threads stop correctly and resources are released.

## Author

- **Name**: Roustam R. Akhmetzyanov
- **Company**: Pass-Dynamics.com

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
