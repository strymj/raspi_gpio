#ifndef OMNI_H_
#define OMNI_H_

#include <iostream>
#include <wiringSerial.h>

class MX28{
	public:
		MX28();
		void portOpen();
	private:
		int fd;
		int baudrate;
};

#endif
