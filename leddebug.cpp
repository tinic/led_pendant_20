#include <sys/ioctl.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <termios.h>

static int setRTS(int fd, int level)
{
    int status;

    if (ioctl(fd, TIOCMGET, &status) == -1) {
        perror("setRTS(): TIOCMGET");
        return 0;
    }
    if (level)
        status |= TIOCM_RTS;
    else
        status &= ~TIOCM_RTS;
    if (ioctl(fd, TIOCMSET, &status) == -1) {
        perror("setRTS(): TIOCMSET");
        return 0;
    }
    return 1;
}

static int setDTR(int fd, int level)
{
    int status;

    if (ioctl(fd, TIOCMGET, &status) == -1) {
        perror("setRTS(): TIOCMGET");
        return 0;
    }
    if (level)
        status |= TIOCM_DTR;
    else
        status &= ~TIOCM_DTR;
    if (ioctl(fd, TIOCMSET, &status) == -1) {
        perror("setRTS(): TIOCMSET");
        return 0;
    }
    return 1;
}

int main()
{
        int fd;
        fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY );
        if (fd) {
				setRTS(fd, 0);
				setDTR(fd, 0);

				struct termios settings;
				tcgetattr(fd, &settings);

				cfsetospeed(&settings, B115200); 
				settings.c_cflag &= ~PARENB;
				settings.c_cflag &= ~CSTOPB;
				settings.c_cflag &= ~CSIZE;
				settings.c_cflag |= CS8 | CLOCAL;
				settings.c_lflag = ICANON;
				settings.c_oflag &= ~OPOST;

				tcsetattr(fd, TCSANOW, &settings);
				tcflush(fd, TCOFLUSH);

                char c = 0;
                while(read(fd, &c, 1) == 1) {
                        putchar(c);
                }
                close(fd);
        }
        return 0;
}

