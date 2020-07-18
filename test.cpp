#include <chrono>
#include <iostream>
#include <thread>

#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int serial_fd;
char flag = 0;
double tx_time;

bool SerialInit(const char *port)
{
    int error_flag = 0;
    struct termios tio;
    speed_t baud = B115200;

    bzero(&tio, sizeof(tio));

    serial_fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(serial_fd < 0){
        std::cout << "Could not Open Port" << std::endl;
        return -1;
    }

    tio.c_cflag += CREAD;       //enable receive
    tio.c_cflag += CLOCAL;      //local line 
    tio.c_cflag += CS8;         // data bit 8bit
    tio.c_cflag += 0;           // stop bit 1bit
    tio.c_cflag += PARENB;      // parity even

    cfsetispeed(&tio, baud);
    cfsetospeed(&tio, baud);

    tio.c_oflag      = 0;
	tio.c_lflag      = 0;
	tio.c_cc[VTIME]  = 0;
	tio.c_cc[VMIN]   = 0;

    error_flag = tcsetattr(serial_fd, TCSANOW, &tio ); // デバイスに設定を行う (成功したら0を返す)
     if (error_flag){ 
        std::cout << "Serial Device Setting error" << std::endl;
 		return -1;
 	}

     error_flag = ioctl(serial_fd, TCSETS, &tio);            // ポートの設定を有効にする
     if (error_flag== -1 ){ 
 		std::cout << "Serial Device Setting error" << std::endl;
 		return -1;
 	}

 	error_flag = tcflush(serial_fd,TCIFLUSH);				// 入力バッファクリア（追加）
    if (error_flag == -1 ){ 
 	 	std::cout << "Serial Device Setting error" << std::endl;
 		return -1;
 	}

    flag = 1;
    tx_time = (1000.0 / (double)baud) * 10.0;

    return 0;

}

bool closePort()
{
    return close(serial_fd);
}

int writePort(uint8_t *data, int size)
{
    return write(serial_fd, data, size);
}

int readPort(uint8_t *data, int size)
{
    return read(serial_fd, data, size);
}

int readNum()
{
    int availables;
    ioctl(serial_fd, FIONREAD, &availables);
    return availables;
}

int ICSwrite(unsigned char *data, unsigned char id, int pos)
{
    int rsize;
    uint8_t tdata[6] = {0};

    tdata[0] = 0x80 | (0x0f & id);
    tdata[1] = (unsigned char)(pos >> 7 & 0x7f);
    tdata[2] = (unsigned char)(pos & 0x7f);
    //std::cout << std::hex << tdata[0] << std::hex << tdata[1] << std::hex << tdata[2] << std::endl;
    writePort(tdata, 3);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    rsize = readNum();

    if(rsize >= 3){
        return (readPort(data, 6));
    }

    return -1;

}

int getID(unsigned char *data)
{
    int rsize;
    uint8_t tdata[12] = {0};

    tdata[0] = 0xFF;
    tdata[1] = 0x00;
    tdata[2] = 0x00;
    tdata[3] = 0x00;

    writePort(tdata, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rsize = readNum();
    return (readPort(data, 5));
}


int main(int argc, char *argv[])
{
    using namespace std;
    uint8_t rdata[64];
    int rsize;
    int available = 0;
    if(0 != SerialInit("/dev/ttyUSB0")){
        return -1;
    }
    int setpos = 5000;
    /*
    rsize = getID(rdata);
    std::cout << rsize << std::endl;

    for(int i = 0; i < 6; i++){
        cout << hex << (int)(rdata[i])  << " ";
    }
    cout << endl;

    */
    while(setpos < 10000){
    for(int j = 0; j < 2; j++){
        uint8_t data[12] = {0};
        rsize  = ICSwrite(data, j, 0);
        if(rsize != -1){
            std::cout << "Connection : " << rsize << std::endl;
            
            /*
            for(int i = 0; i < 6; i++){
                std::cout << std::hex << (int)(data[i]) << " ";
            }
            std::cout << std::endl;
            */
            int Num = (int)(data[3] & 0x7F);
            int Npos = (int)(data[4]);
            Npos = (Npos << 7) + (int)data[5];
            std::cout << Num << " " << Npos << std::endl;
            
        }
        else{
            std::cout << "Could not Connection" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    setpos += 100;
    }
    

    closePort();

    return 0;


}