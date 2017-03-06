#include <raspi_gpio/MX28.h>
using namespace std;

MX28::MX28(){
	fd = -1;
	baudrate = 57600;
}

void MX28::portOpen()
{
	fd = serialOpen("/dev/ttyAMA0", baudrate);
	if(fd == -1) {
		cout<<"Failed to open serial port."<<endl;
	}
	else {
		cout<<"Serial port opened."<<endl;
	}

}

