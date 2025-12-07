#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

int main() {
    const char* device = "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00";

    int fd = open(device, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    struct input_event ev;

    std::cout << "Reading PowerMate events...\n";

    while (true) {
        ssize_t n = read(fd, &ev, sizeof(ev));
        if (n != sizeof(ev)) continue;

        if (ev.type == EV_REL && ev.code == REL_DIAL) {
            // PowerMate rotation
            std::cout << "Rotated: " << ev.value << "\n";
        }

        if (ev.type == EV_KEY && ev.code == BTN_MISC) {
            if (ev.value == 1)
                std::cout << "Button pressed\n";
            else if (ev.value == 0)
                std::cout << "Button released\n";
        }
    }

    close(fd);
    return 0;
}
