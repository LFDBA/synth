#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sstream>

int main() {
    const char* port = "/dev/ttyACM0";  // Change if needed
    int fd = open(port, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open serial port\n";
        return 1;
    }

    // Serial config (same as before)
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error: tcgetattr failed\n";
        return 1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &tty);

    std::cout << "Reading labeled pot values...\n";

    char buf[64];
    std::string line;

    while (true) {
        int n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                char c = buf[i];

                if (c == '\n') {
                    if (!line.empty()) {
                        std::stringstream ss(line);
                        std::string label;
                        int value;

                        ss >> label >> value;

                        if (label == "p1") {
                            std::cout << "[P1] " << value << std::endl;
                        } 
                        else if (label == "p2") {
                            std::cout << "[P2] " << value << std::endl;
                        } 
                        else {
                            std::cout << "Unknown label: " << label 
                                      << " value=" << value << std::endl;
                        }
                    }
                    line.clear();
                }
                else if (c != '\r') {
                    line += c;
                }
            }
        }
    }

    close(fd);
    return 0;
}
