#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <thread>
#include <atomic>

// 데이터 읽을 때 여기서부터 읽어라
#define START_FRAME 0xABCD

// 프로토콜 ( 수신 )
struct SerialFeedback {
    uint16_t start;
    uint16_t cmd1;
    uint16_t cmd2;
    uint16_t wheelR_cnt;
    uint16_t wheelL_cnt;
    uint16_t batVoltage;
    uint16_t boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
};

// 프로토콜 ( 발신 )
struct SerialCommand {
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint16_t checksum;
};

// class 정의
class HoverboardDriver {
// 이 class에서 쓸 변수(값)
private:
    int port_fd;
    SerialFeedback msg;
    char *p;
    uint16_t start_frame;
    char prev_byte;
    int msg_len;
    std::thread serial_thread;
    std::atomic<bool> running;

public:
   // HoverboardDriver 생성자 ( 이 객체를 생성하면 최초 실행되는 것들, 여기선 포트 설정)
    HoverboardDriver(const std::string &port) : running(true) {
        if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
            std::cerr << "Cannot open serial port to hoverboard" << std::endl;
            exit(EXIT_FAILURE);
        }
	
	// 시리얼 포트 설정
        struct termios options;
        tcgetattr(port_fd, &options);
        options.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcflush(port_fd, TCIFLUSH);
        tcsetattr(port_fd, TCSANOW, &options);
	
	// 값들 초기화
        msg_len = 0;
        prev_byte = 0;
        
	// read_from_serial 함수를 쓰레드 돌림 ( 연결이 끊어지지 않게 하기 위해서 )
        serial_thread = std::thread(&HoverboardDriver::read_from_serial, this);
    }

    // 소멸자 (객체가 삭제될때 실행) / 여기선 쓰레드 종료 하고 포트 닫음
    ~HoverboardDriver() {
        running = false;
        if (serial_thread.joinable()) {
            serial_thread.join();
        }
        close(port_fd);
    }

    // 프로토콜에 맞는 값 받는 받고 checksum 계산까지
    // 값들이 cd ab 이렇게 와서 비트 스위치를 통해 ab cd로 만들어서 읽음
    void protocol_recv(char byte) {
        start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

	// start frame을 읽으면 그 다음것들 쭉 읽음
        if (start_frame == START_FRAME) {
            p = (char *)&msg;
            *p++ = prev_byte;
            *p++ = byte;
            msg_len = 2;
        } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
            *p++ = byte;
            msg_len++;
        }
	
	// 프로토콜 사이즈(하나당 2바이트 총 18 바이트)랑 들어온 값들 바이트 수랑 맞으면 checksum 계산 실행 
        if (msg_len == sizeof(SerialFeedback)) {
            uint16_t checksum = msg.start ^ msg.cmd1 ^ msg.cmd2 ^ msg.batVoltage ^
                                msg.cmdLed ^ msg.wheelR_cnt ^ msg.wheelL_cnt ^ msg.boardTemp;
	    
	    // start frame 과 checksum도 다 맞으면 아래 값들을 출력
            if (msg.start == START_FRAME && msg.checksum == checksum) {
                std::cout << "message received: "
                          << " WheelR: " << -msg.wheelR_cnt
                          << " WheelL: " << msg.wheelL_cnt << std::endl;
            }
            // msg 길이 다시 초기화
            msg_len = 0;
        }

        prev_byte = byte;
    }

    // 보낼 값들 정의 및 checksum까지 계산
    void send_command(int16_t speed, int16_t steer) {
        SerialCommand command;
        command.start = START_FRAME;
        command.steer = steer;
        command.speed = speed;
        command.checksum = command.start ^ command.steer ^ command.speed;
	
        int rc = write(port_fd, &command, sizeof(command));
        //if (rc < 0) {
        //    std::cerr << "Error writing to serial port" << std::endl;
        //}
    }

    // serial 연결 함수 ( pretocol_recv를 포함하고있음)
    void read_from_serial() {
        char byte;
        while (running) {
            if (read(port_fd, &byte, 1) > 0) {
                protocol_recv(byte);
            } else {
                usleep(1000);
            }
        }
    }
};

int main() {
    HoverboardDriver driver("/dev/ttyUSB0");
    
    while (true) {
        driver.send_command(150, 0);
        //sleep(1);
    }

    return 0;
}

