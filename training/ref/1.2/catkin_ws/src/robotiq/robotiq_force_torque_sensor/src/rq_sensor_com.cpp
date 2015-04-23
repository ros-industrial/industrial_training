/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/**
 * \file rq_sensor_com.c
 * \date June 18, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

//////////
//Includes

//Platform specific
#ifdef __unix__ /*For Unix*/
#define _BSD_SOURCE
#include <termios.h>
#include <unistd.h>
#elif defined(_WIN32)||defined(WIN32) /*For Windows*/
#include <windows.h>
#endif

#include <dirent.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

//application specific
#include "robotiq_force_torque_sensor/rq_sensor_com.h"

// Definitions
#define REGISTER_SELECT_OUTPUT 410
#define REGISTER_SERIAL_NUMBER 510
#define REGISTER_PRODUCTION_YEAR 514
#define REGISTER_FIRMWARE_VERSION 500
#define RQ_COM_MAX_STR_LENGTH 20
#define RQ_COM_JAM_SIGNAL_CHAR      0xff
#define RQ_COM_JAM_SIGNAL_LENGTH    50
#define RQ_COM_TIMER_FOR_STREAM_DETECTION_MAX_VALUE 20 //20 * 4ms =  80ms without bytes means that the stream is stopped
#define RQ_COM_TIMER_FOR_VALID_STREAM_MAX_VALUE     40 //40 * 4ms = 160ms without any valid message means that the communication is not working well

////////////////////
// Private variables
static float rq_com_received_data[6] = {0.0};
static float rq_com_received_data_offset[6] = {0.0};

static UINT_16 rq_com_computed_crc = 0;
static UINT_16 rq_com_crc = 0;

static INT_8 rq_com_str_sensor_serial_number[RQ_COM_MAX_STR_LENGTH];
static INT_8 rq_com_str_sensor_production_year[RQ_COM_MAX_STR_LENGTH];
static INT_8 rq_com_str_sensor_firmware_version[RQ_COM_MAX_STR_LENGTH];

static UINT_32 rq_com_msg_received = 0;

static UINT_8 rq_com_rcv_buff[MP_BUFF_SIZE];
static UINT_8 rq_com_snd_buff[MP_BUFF_SIZE];
static UINT_8 rq_com_rcv_buff2[MP_BUFF_SIZE];
static INT_32 rq_com_rcv_len;
static INT_32 rq_com_rcv_len2 = 0;

static INT_32 rq_com_zero_force_flag = 0;
static INT_32 rq_state_sensor_watchdog = 0;

//variables related to the communication status
static bool rq_com_stream_detected = false;
static bool rq_com_valid_stream = false;
static bool rq_com_new_message = false;
static INT_32  rq_com_timer_for_stream_detection = 0;
static INT_32  rq_com_timer_for_valid_stream = 0;

///////////////////////////////
//Private functions declaration
static INT_8 rq_com_tentative_connexion(void);
static void rq_com_send_jam_signal(void);
static void rq_com_stop_stream_after_boot(void);

static INT_32 rq_com_read_port(UINT_8 * const buf, UINT_32 buf_len);
static INT_32 rq_com_write_port(UINT_8 const * const buf, UINT_32 buf_len);

//Modbus functions
static UINT_16 rq_com_compute_crc(UINT_8 const * adr, INT_32 length );
static void rq_com_send_fc_03_request(UINT_16 base, UINT_16 n);
static void rq_com_send_fc_16_request(INT_32 base, INT_32 n, UINT_8 const * const data);
static INT_32 rq_com_wait_for_fc_03_echo(UINT_8 * const data);
static INT_32 rq_com_wait_for_fc_16_echo(void);
static INT_8 rq_com_send_fc_03(UINT_16 base, UINT_16 n, UINT_16 * const data);
static INT_8 rq_com_send_fc_16(INT_32 base, INT_32 n, UINT_16 const * const data);

static UINT_8 rq_com_identify_device(INT_8 const * const d_name);

#ifdef __unix__ //For Unix
static INT_32 fd_connexion = -1;
static INT_8 set_com_attribs (INT_32 fd, speed_t speed);
#elif defined(_WIN32)||defined(WIN32) //For Windows
HANDLE hSerial;
#endif

//////////////////////
//Function definitions

/**
 * \fn void rq_sensor_com(void)
 * \brief Discovers and initialize the communication with the sensor
 * \details The functions loops  through all the serial com ports 
 *          and polls them to discover the sensor
 */
INT_8 rq_sensor_com()
{
#ifdef __unix__ //For Unix
	UINT_8 device_found = 0;
	DIR *dir = NULL;
	struct dirent *entrydirectory = NULL;

	//Close a previously opened connection to a device
	close(fd_connexion);
	if ((dir = opendir("/sys/class/tty/")) == NULL)
	{
		return -1;
	}

	//Loops through the files in the /sys/class/tty/ directory
	while ((entrydirectory = readdir(dir)) != NULL && device_found == 0)
	{
		//Look for a serial device
		if (strstr(entrydirectory->d_name, "ttyS") || 
		    strstr(entrydirectory->d_name, "ttyUSB"))
		{
			device_found = rq_com_identify_device(entrydirectory->d_name);
		}
	}

	closedir(dir);

	if (device_found == 0)
	{
		return -1;
	}
#elif defined(_WIN32)||defined(WIN32) //For Windows
	DCB dcb;
	INT_32 i;
	INT_8 port[13];
	for(i = 0;i < 256; i++){
		sprintf(port,"\\\\.\\COM%d",i);
		hSerial = CreateFileA(port, GENERIC_READ | GENERIC_WRITE,
				0,NULL,OPEN_EXISTING,0,NULL);
		if(hSerial != INVALID_HANDLE_VALUE){
			dcb.DCBlength = sizeof(DCB);
			if (!GetCommState(hSerial, &dcb)){
				CloseHandle(hSerial);
				hSerial = INVALID_HANDLE_VALUE;
				continue;//Permet de recommencer la boucle
			}
			dcb.BaudRate = CBR_19200;
			dcb.ByteSize = 8;
			dcb.StopBits = ONESTOPBIT;
			dcb.Parity = NOPARITY;
			dcb.fParity = FALSE;

			/* No software handshaking */
			dcb.fTXContinueOnXoff = TRUE;
			dcb.fOutX = FALSE;
			dcb.fInX = FALSE;
			//dcb.fNull = FALSE;

			/* Binary mode (it's the only supported on Windows anyway) */
			dcb.fBinary = TRUE;

			/* Don't want errors to be blocking */
			dcb.fAbortOnError = FALSE;

			/* Setup port */
			if(!SetCommState(hSerial, &dcb)){
				CloseHandle(hSerial);
				continue;//Permet de recommencer la boucle
			}
			COMMTIMEOUTS timeouts={0};
			timeouts.ReadIntervalTimeout=0;
			timeouts.ReadTotalTimeoutConstant=1;
			timeouts.ReadTotalTimeoutMultiplier=0;
			timeouts.WriteTotalTimeoutConstant=1;
			timeouts.WriteTotalTimeoutMultiplier=0;
			if(!SetCommTimeouts(hSerial, &timeouts)){
				CloseHandle(hSerial);
				hSerial = INVALID_HANDLE_VALUE;
				continue;//Permet de recommencer la boucle
			}
			if (rq_com_tentative_connexion() == 1){
				return 0;
			}
			CloseHandle(hSerial);
		}
	}
	return -1;
#else
#endif
	return 0;
}

/**
 * \fn static INT_8 rq_com_tentative_connexion()
 * \brief Tries connecting to the sensor
 * \returns 1 if the connection attempt succeeds, -1 otherwise
 */
static INT_8 rq_com_tentative_connexion()
{
	INT_32 rc = 0;
	UINT_16 firmware_version [1];
	UINT_8 retries = 0;

	while(retries < 5 && rq_com_stream_detected == false)
	{
		rq_com_listen_stream();
		retries++;
	}

	rq_com_listen_stream();
	if(rq_com_stream_detected)
	{
		rq_com_stop_stream_after_boot();
	}

	//Give some time to the sensor to switch to modbus
	usleep(100000);

	//If the device returns an F as the first character of the fw version,
	//we consider its a sensor
	rc = rq_com_send_fc_03(REGISTER_FIRMWARE_VERSION, 2, firmware_version);
	if (rc != -1)
	{
		if (firmware_version[0] >> 8 == 'F')
		{
			return 1;
		}
	}

	return -1;
}

/**
 * \fn static INT_8 set_com_attribs()
 * \brief Sets the com port parameters to match those of the sensor
 * \param fd, file descriptor that points to the serial port
 * \param speed, baudrate
 * \return 0 in case of a success, -1 otherwise
 */
#ifdef __unix__ //For Unix
static INT_8 set_com_attribs (INT_32 fd, speed_t speed)
{
	struct termios tty;
	memset (&tty, 0, sizeof (struct termios));

	if (tcgetattr (fd, &tty) != 0)
	{
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~PARENB;
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tty.c_iflag &= ~INPCK;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_cc[VMIN]  = 0;			// read doesn't block
	tty.c_cc[VTIME] = 1;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf("error %d from tcsetattr", errno);
		return -1;
	}

	return 0;
}
#endif

/**
 * \fn void rq_com_listen_stream()
 * \brief Listens and decode a valid stream input
 */
void rq_com_listen_stream(void)
{
	static INT_32 last_byte = 0;
	static INT_32 new_message = 0;
	INT_32 i = 0;
	INT_32 j = 0;    

	//Capture and store the current value of the sensor and use it to
	//zero subsequent sensor values
	if(rq_com_zero_force_flag == 1)
	{
		rq_com_zero_force_flag = 0;

		for(i = 0; i < 6; i++)
		{
			rq_com_received_data_offset[i] = rq_com_received_data[i];
		}

	}

	usleep(4000);

	//Increment communication state counters
	if(rq_com_timer_for_stream_detection++ > RQ_COM_TIMER_FOR_STREAM_DETECTION_MAX_VALUE)
	{
		rq_com_timer_for_stream_detection = RQ_COM_TIMER_FOR_STREAM_DETECTION_MAX_VALUE;
		rq_com_stream_detected = false;
	}

	if(rq_com_timer_for_valid_stream++ > RQ_COM_TIMER_FOR_VALID_STREAM_MAX_VALUE)
	{
		rq_com_timer_for_valid_stream = RQ_COM_TIMER_FOR_VALID_STREAM_MAX_VALUE;
		rq_com_valid_stream = false;
	}

	last_byte = 0;
	new_message = 0;

	//Data reception
	rq_com_rcv_len = rq_com_read_port(rq_com_rcv_buff, MP_BUFF_SIZE);

	//If at least a byte is received, we consider the sensor to stream
	if(rq_com_rcv_len > 0)
	{
		rq_com_stream_detected = true;
		rq_com_timer_for_stream_detection = 0;
	}

	//Copie les données au bout du buffer 2
	for(i = 0; i < rq_com_rcv_len; i++)
	{
		//If the buffer overflows, set the index to the beginning
		if(rq_com_rcv_len2 == MP_BUFF_SIZE)
		{
			//Next bytes will overwrite the begining of the buffer
			rq_com_rcv_len2 = 0;
			break;
		}

		rq_com_rcv_buff2[rq_com_rcv_len2++] = rq_com_rcv_buff[i];
	}

	//empty the buffers
	memset( rq_com_rcv_buff, 0, sizeof(rq_com_rcv_buff));
	memset( rq_com_snd_buff, 0, sizeof(rq_com_snd_buff));

	//If there is enough characters,...
	if(rq_com_rcv_len2 >= 16 && rq_com_rcv_len >= 1) 
	{        
		//Search for a valid stream message in the buffer
		for(i = rq_com_rcv_len2 - 16; i >= 0; i--)
		{
			//Header
			if(rq_com_rcv_buff2[i] == 0x20 && rq_com_rcv_buff2[i+1] == 0x4E && new_message == 0)
			{

				last_byte = i - 1;

				rq_com_computed_crc = rq_com_compute_crc(rq_com_rcv_buff2 + i * sizeof(*rq_com_rcv_buff2), 14);

				rq_com_crc 			= (rq_com_rcv_buff2[i+14] + rq_com_rcv_buff2[i+15] * 256);

				rq_com_msg_received++;


				//The crc is valid.. the message can be used
				if(rq_com_computed_crc == rq_com_crc)
				{
					last_byte = i + 15; //will erase the message

					//convert the efforts to floating point numbers
					for(j = 0; j < 3; j++)
					{
						rq_com_received_data[j] = ((float)((INT_16)((UINT_8)rq_com_rcv_buff2[i+2+2*j] + ((INT_16)(UINT_8)rq_com_rcv_buff2[i+3+2*j]) * 256))) / 100 ;
					}

					for(j = 3; j < 6; j++)
					{
						rq_com_received_data[j] = ((float)((INT_16)((UINT_8)rq_com_rcv_buff2[i+2+2*j] + ((INT_16)(UINT_8)rq_com_rcv_buff2[i+3+2*j]) * 256))) / 1000;
					}

					//Signal the stream as valid
					rq_com_valid_stream = true;
					rq_com_new_message = true;
					rq_com_timer_for_valid_stream = 0;

					new_message = 1;
					rq_state_sensor_watchdog = 1;

				}
				else
				{
					last_byte = i + 15;
					new_message = 1;
				}
			}
		}

		if(last_byte > 0)
		{
			//On shift le buffer 2 afin de ne garder que ce qui dépasse le dernier caractère du dernier message complet
			for(i = 0; i < (rq_com_rcv_len2 - last_byte - 1); i++)
			{
				rq_com_rcv_buff2[i] = rq_com_rcv_buff2[last_byte + 1 + i];
			}
			rq_com_rcv_len2 = rq_com_rcv_len2 - last_byte - 1;
		}
	}
}

/**
 * \fn static void rq_com_send_jam_signal(void)
 * \brief Send a signal that interrupts the streaming
 */
static void rq_com_send_jam_signal(void)
{
	//Build the jam signal
	memset(rq_com_snd_buff, RQ_COM_JAM_SIGNAL_CHAR, RQ_COM_JAM_SIGNAL_LENGTH);

	//Send the jam signal
	rq_com_write_port(rq_com_snd_buff,RQ_COM_JAM_SIGNAL_LENGTH);
}


/**
 * \fn static void rq_com_stop_stream_after_boot(void)
 * \brief Send a jam signal to stop the sensor stream
 *        and retry until the stream stops
 */
static void rq_com_stop_stream_after_boot(void)
{
	static INT_32 counter = 0;
	
	while(rq_com_stream_detected)
	{
		counter++;

		if(counter == 1)
		{
			rq_com_send_jam_signal();
		}
		else
		{
			rq_com_listen_stream();

			if(rq_com_stream_detected)
			{
				counter = 0;
			}
			else
			{
				counter = 0;
			}
		}
	}
}

/**
 * \fn INT_8 rq_com_start_stream(void)
 * \brief Starts the sensor streaming mode
 * \return 0 if the stream started, -1 otherwise
 */
INT_8 rq_com_start_stream(void)
{
	UINT_16 data[1] = {0x0200}; //0x0200 selects the stream output
	UINT_8 retries = 0;
	INT_8 rc = rq_com_send_fc_16(REGISTER_SELECT_OUTPUT, 2, data);

	if (rc == -1)
	{
		while(retries < 5)
		{
			rq_com_listen_stream();

			if(rq_com_stream_detected)
			{
				return 0;
			}

			usleep(50000);
		}

		return -1;
	}

	return 0;
}

/**
 * \brief Sends a read request
 * \param base Address of the first register to read
 * \param n Number of bytes to read
 * \param data table into which data will be written
 */
static INT_8 rq_com_send_fc_03(UINT_16 base, UINT_16 n, UINT_16 * const data)
{
	UINT_8 bytes_read = 0;
	INT_32 i = 0;
	INT_32 cpt = 0;
	UINT_8 data_request[n]; 
	UINT_16 retries = 0;

	//precondition, null pointer
	if (data == NULL)
	{
		return -1;
	}

	//Send the read request
	rq_com_send_fc_03_request(base,n);

	//Read registers
	while (retries < 100 && bytes_read == 0)
	{
		usleep(4000);
		bytes_read = rq_com_wait_for_fc_03_echo(data_request);
		retries++;
	}

	if (bytes_read <= 0)
	{
		return -1;
	}

	for (i = 0; i < n/2; i++ )
	{
		data[i] = (data_request[cpt++] * 256);
		data[i] = data[i] + data_request[cpt++];
	}

	return 0;
}

/**
 * \fn static INT_8 rq_com_send_fc_16(INT_32 base, INT_32 n, UINT_16 * data)
 * \brief Sends a write request to a number of registers
 * \param base Address of the first register to write to
 * \param n Number of registers to write
 * \param data buffer that contains data to write
 */
static INT_8 rq_com_send_fc_16(INT_32 base, INT_32 n, UINT_16 const * const data)
{
	INT_8 valid_answer = 0;
	UINT_8 data_request[n];
	UINT_16 retries = 0;
	UINT_32 i;

	//precondition, null pointer
	if (data == NULL)
	{
		return -1;
	}

	for (i = 0; i < n; i++ )
	{
		if(i % 2 == 0)
		{
			data_request[i] = (data[(i/2)] >> 8);
		}
		else
		{
			data_request[i] = (data[(i/2)] & 0xFF);
		}
	}

	rq_com_send_fc_16_request(base, n, data_request);

	while (retries < 100 && valid_answer == 0)
	{
		usleep(10000);
		valid_answer = rq_com_wait_for_fc_16_echo();
		retries++;
	}

	if(valid_answer == 1)
	{
		return 0;
	}

	return -1;
}

//////////////////
//Public functions

/**
 * \fn void rq_sensor_com_read_info_high_lvl(void)
 * \brief Reads and stores high level information from
 *        the sensor. These include the firmware version,
 *        the serial number and the production year.
 */
void rq_sensor_com_read_info_high_lvl(void)
{
	UINT_16 registers[4] = {0};//table de registre
	UINT_64 serial_number = 0;
	INT_32 result = 0;

	//Firmware Version
	result = rq_com_send_fc_03(REGISTER_FIRMWARE_VERSION, 6, registers);
	if (result != -1)
	{
		sprintf(rq_com_str_sensor_firmware_version, "%c%c%c-%hhu.%hhu.%hhu",
				registers[0] >> 8, registers[0] & 0xFF, registers[1] >> 8,
				registers[1] & 0xFF, registers[2] >> 8, registers[2] & 0xFF);
	}

	//Production Year
	result = rq_com_send_fc_03(REGISTER_PRODUCTION_YEAR, 2, registers);
	if (result != -1)
	{
		sprintf(rq_com_str_sensor_production_year, "%u", registers[0]);
	}

	//Serial Number
	result = rq_com_send_fc_03(REGISTER_SERIAL_NUMBER, 8, registers);
	if (result != -1)
	{
		serial_number = (UINT_64) ((registers[3] >> 8) * 256 * 256 * 256
				+ (registers[3] & 0xFF) * 256 * 256 + (registers[2] >> 8) * 256
				+ (registers[2] & 0xFF));

		if (serial_number == 0)
		{
			sprintf(rq_com_str_sensor_serial_number, "UNKNOWN");
		}
		else
		{
			sprintf(rq_com_str_sensor_serial_number, "%c%c%c-%.4lu", registers[0] >> 8,
					registers[0] & 0xFF, registers[1] >> 8, serial_number);
		}
	}
}

/**
 * \fn static INT_32 rq_com_read_port(UINT_8 * buf, UINT_32 buf_len)
 * \brief Reads incomming data on the com port
 * \param buf, contains the incomming data
 * \param buf_len maximum number of data to read
 * \return The number of character read
 */
static INT_32 rq_com_read_port(UINT_8 * const buf, UINT_32 buf_len)
{
#ifdef __unix__ //For Unix
	return read(fd_connexion, buf, buf_len);
#elif defined(_WIN32)||defined(WIN32) //For Windows
	DWORD myBytesRead = 0;
	ReadFile(hSerial,buf,buf_len,&myBytesRead,NULL);
	return myBytesRead;
#endif
}

/**
 * \fn static INT_32 rq_com_write_port(UINT_8 * buf, UINT_32 buf_len)
 * \brief Writes on the com port
 * \param buf data to write
 * \param buf_len numer of bytes to write
 * \return The number of characters written or -1 in case of an error
 */
static INT_32 rq_com_write_port(UINT_8 const * const buf, UINT_32 buf_len)
{
	//precondition
	if (buf == NULL)
	{
		return -1;
	}

#ifdef __unix__ //For Unix
    return write(fd_connexion, buf, buf_len);
#elif defined(_WIN32)||defined(WIN32) //For Windows
    DWORD n_bytes = 0;
    return (WriteFile(hSerial, buf, buf_len, &n_bytes, NULL)) ? n_bytes: -1;
#endif
}

/**
 * \fn static UINT_16 rq_com_compute_crc(UINT_8 * adr, INT_32 length )
 * \param adr, Address of the first byte 
 * \param length Length of the buffer on which the crc is computed
 * \return Value of the crc
 */
static UINT_16 rq_com_compute_crc(UINT_8 const * adr, INT_32 length )
{
        UINT_16 CRC_calc = 0xFFFF;
        INT_32 j=0;
        INT_32 k=0;

	//precondition, null pointer
	if (adr == NULL)
	{
		return 0;
	}

	//Tant qu'il reste des bytes dans le message
	while (j < length)
	{
		//Si c'est le premier byte
		if (j==0)
		{
			CRC_calc ^= *adr & 0xFF;
		}
		//Sinon on utilisera un XOR sur le word
		else
		{
			CRC_calc ^= *adr;
		}

		k=0;
		
		//Tant que le byte n'est pas complété
		while (k < 8)
		{
			//Si le dernier bit est un 1
			if (CRC_calc & 0x0001)
			{
				CRC_calc =  (CRC_calc >> 1)^ 0xA001;	//Shift de 1 bit vers la droite et XOR avec le facteur polynomial
			}
	    	else
			{
				CRC_calc >>= 1;			//Shift de 1 bit vers la droite
			}
	
    		k++;
	    }
	
		//Incrémente l'adresse et le compteur d'adresse
		adr++;
		j++;
	}
	
	return CRC_calc;
}

/**
 * \fn static void rq_com_send_fc_03_request(UINT_16 base, UINT_16 n)
 * \brief Compute and send the fc03 request on the com port
 * \param base Address of the first register to read
 * \param n Number of bytes to read
 */
static void rq_com_send_fc_03_request(UINT_16 base, UINT_16 n)
{
	static UINT_8 buf[MP_BUFF_SIZE];
	INT_32 length = 0;
	UINT_8 reg[2];
	UINT_8 words[2];
	UINT_16 CRC;
	
	//Si le nombre de registre est impair
	if(n % 2 != 0)
	{
		n += 1;
	}
	
	//Scinder en LSB et MSB
	reg[0]   = (UINT_8)(base >> 8); //MSB to the left
	reg[1]   = (UINT_8)(base & 0x00FF); //LSB to the right
	words[0] = (UINT_8)((n/2) >> 8); //MSB to the left
	words[1] = (UINT_8)((n/2) & 0x00FF); //LSB to the right
	
	//Build the request
	buf[length++] = 9; //slave address
	buf[length++] = 3;
	buf[length++] = reg[0];
	buf[length++] = reg[1];
	buf[length++] = words[0];
	buf[length++] = words[1];
	
	//CRC computation
	CRC = rq_com_compute_crc(buf, length);
	
	//Append the crc
	buf[length++] = (UINT_8)(CRC & 0x00FF);
	buf[length++] = (UINT_8)(CRC >> 8);
	
	//Send the request
	rq_com_write_port(buf, length);
}

/**
 * \fn static INT_32 rq_com_wait_for_fc_03_echo(UINT_8 data[])
 * \brief Reads the reply to a fc03 request
 * \param base Address of the buffer that will store the reply
 * \return The number of character read
 */
static INT_32 rq_com_wait_for_fc_03_echo(UINT_8 * const data)
{
	static UINT_8 buf[MP_BUFF_SIZE];
	static INT_32 length = 0;
	static INT_32 old_length = 0;
	static INT_32 counter_no_new_data = 0;
	UINT_8 n = 0;
	UINT_16 CRC = 0;
	INT_32 j = 0;
	INT_32 ret = rq_com_read_port(&buf[length], MP_BUFF_SIZE - length);
	
	if(ret != -1)
	{
		length = length + ret;
	}

	//If there is no new data, the buffer is cleared
	if(length == old_length)
	{
		if(counter_no_new_data < 5)
		{
			counter_no_new_data++;
		}
		else
		{
			length = 0;
		}
	}
	else
	{
		counter_no_new_data = 0;
	}
	
	old_length = length;
	
	if(length > 0)
	{
		//If there is not enough data, return
		if(length <= 5)
		{
			return 0;
		}
		else
		{
			if(buf[1] == 3) //3 indicates the response to a fc03 query
			{
				n = buf[2];
				if(length < 5 + n)
				{
					return 0;
				}
			}
			else //unknown fc code
			{
				length = 0;
				return 0;
			}
		}
		CRC = rq_com_compute_crc(buf, length - 2);
		
		//Verifies the crc and the slave ID
		if(CRC != (UINT_16)((buf[length - 1] * 256) + (buf[length - 2])))
		{
			//On clear le buffer
			buf[0] = 0;
			length = 0;
			return 0;
		}
		else
		{
			n = buf[2];
			
			//Writes the bytes to the return buffer
			for(j = 0; j < n; j++)
			{
				data[j] = buf[j + 3];
			}
			
			//Clears the buffer
			buf[0] = 0;
			length = 0;
			return n;
		}
	}
	
	return 0;
}


/**
 * \fn void modbus_send_fc_16_request(INT_32 base, INT_32 n, UINT_8 * data)
 * \brief Sends a fc16 write request
 * \param base Address of the first register to write to
 * \param n Number of bytes to write
 */
static void rq_com_send_fc_16_request(INT_32 base, INT_32 n, UINT_8 const * const data)
{
	static UINT_8 buf[MP_BUFF_SIZE];
	INT_32 length = 0;

	//Byte of the query
	UINT_8 reg[2];
	UINT_8 words[2];
	UINT_16 CRC;
	INT_32 n2 = 0;
	INT_32 i = 0;

	if (data == NULL)
	{
		return;
	}
	
	//Manage if the number of bytes to write is odd or even
	if(n %2 != 0)
	{
		n2 = n+1;
	}
	else
	{
		n2 = n;
	}
	
	//Split the address and the number of bytes between MSB ans LSB
	reg[0]   = (UINT_8)(base >> 8);       //MSB to the left
	reg[1]   = (UINT_8)(base & 0x00FF);   //LSB to the right
	words[0] = (UINT_8)((n2/2) >> 8);     //MSB to the left
	words[1] = (UINT_8)((n2/2) & 0x00FF); //LSB to the right
	
	
	//Build the query
	buf[length++] = 9; //slave address
	buf[length++] = 16;
	buf[length++] = reg[0];
	buf[length++] = reg[1];
	buf[length++] = words[0];
	buf[length++] = words[1];
	buf[length++] = n2;
	
	//Copy data to the send buffer
	for(i = 0; i < n; i++)
	{
		buf[length++] = data[i];
	}
	
	if(n != n2)
	{
		buf[length++] = 0;
	}
	
	CRC = rq_com_compute_crc(buf, length);
	
	//Append the crc to the query
	buf[length++] = (UINT_8)(CRC & 0x00FF);
	buf[length++] = (UINT_8)(CRC >> 8);
	
	//Send the query
	rq_com_write_port(buf, length);
}

/**
 * \fn static INT_32 rq_com_wait_for_fc_16_echo(void)
 * \brief Reads the response to a fc16 write query
 * \return 0 for an invalid response, 1 otherwise
 */
static INT_32 rq_com_wait_for_fc_16_echo(void)
{
	static UINT_8 buf[MP_BUFF_SIZE];
	static INT_32 length = 0;
	static INT_32 old_length = 0;
	static INT_32 counter_no_new_data = 0;
	UINT_16 CRC = 0;

	length = length + rq_com_read_port(&buf[length], MP_BUFF_SIZE - length);

	//Clear the buffer if no new data
	if(length == old_length)
	{
		if(counter_no_new_data < 5)
		{
			counter_no_new_data++;
		}
		else
		{
			length = 0;
		}
	}
	else
	{
		counter_no_new_data = 0;
	}

	old_length = length;

	if(length > 0)
	{
		//If not enough data, return
		if(length < 8)
		{
			return 0;
		}
		else
		{
			//if it's a reply to a fc16 query then proceed
			if(buf[1] == 16)
			{
					length = 8;

					CRC = rq_com_compute_crc(buf, length - 2);

					//Check the crc an the slave ID
					if(CRC != (UINT_16)((buf[length - 1] * 256) + (buf[length - 2])))
					{
						//Clear the buffer
						length = 0;

						return 0;
					}
					else
					{
						//Clear the buffer
						length = 0;

						return 1;
					}

			}
			else //Clear the buffer
			{
					length = 0;
					return 0;
			}
		}

	}

	return 0;
}


/**
 * \brief Retrieves the sensor serial number
 * \param serial_number address of the return buffer
 */
void rq_com_get_str_serial_number(INT_8 * serial_number)
{
	strcpy(serial_number, rq_com_str_sensor_serial_number);
}

/**
 * \brief Retrieves the sensor firmware version
 * \param firmware_version Address of the return buffer
 */
void rq_com_get_str_firmware_version(INT_8 * firmware_version)
{
	strcpy(firmware_version, rq_com_str_sensor_firmware_version);
}

/**
 * \brief Retrieves the sensor firmware version
 * \param production_year Address of the return buffer
 */
void rq_com_get_str_production_year(INT_8 * production_year)
{
	strcpy(production_year,rq_com_str_sensor_production_year);
}

/**
 * \brief retrieves the sensor firmware version
 * \param production_year Address of the return buffer
 */
bool rq_com_get_stream_detected()
{
	return rq_com_stream_detected;
}

/**
 * \brief returns if the stream message is valid
 */
bool rq_com_get_valid_stream()
{
	return rq_com_valid_stream;
}

/**
 * \brief Return an effort component
 * \param i Index of the component. 0 to 2 for Fx, Fy and Fz.
 *          3 to 5 for Mx, My and Mz.
 */
float rq_com_get_received_data(UINT_8 i)
{
	if(i >= 0 && i <= 5)
	{
		return rq_com_received_data[i] - rq_com_received_data_offset[i];
	}

	return 0.0;
}

/**
 * \brief Returns true if a new valid stream message has been decoded and
 *        is available.
 * \details When this function is called, the variable that indicates if a 
            new message is available is set to false even if the message
            hasn't beed read.
 */
bool rq_com_got_new_message()
{
	bool tmp = rq_com_new_message;
	rq_com_new_message = false;
	return tmp;
}

/**
 * \brief Set the "zero sensor" flag to 1. When the next stream message will
          be decoded, the effort values will be stored as offsets a
          substracted from the next values
 */
void rq_com_do_zero_force_flag()
{
	rq_com_zero_force_flag = 1;
}

/**
 * \brief close the serial port. 
 * \warning Only valid under Windows.
 */
void stop_connection()
{
#if defined(_WIN32)||defined(WIN32) //For Windows
	CloseHandle(hSerial);
	hSerial = INVALID_HANDLE_VALUE;
#endif
}


/**
 * \author Pierre-Olivier Proulx
 * \brief try to discover a com port by polling each serial port
 * \return 1 if a device is found, 0 otherwise
 */
static UINT_8 rq_com_identify_device(INT_8 const * const d_name)
{
	INT_8 dirParent[20] = {0};
	INT_8 port_com[15] = {0};

	strcpy(dirParent, "/dev/");
	strcat(dirParent, d_name);
	strcpy(port_com, dirParent);
	fd_connexion = open(port_com, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);

	//The serial port is open
	if(fd_connexion != -1)
	{
		if(set_com_attribs(fd_connexion,B19200) != -1)
		{
			//Try connecting to the sensor
			if (rq_com_tentative_connexion() == 1)
			{
				return 1;
			}
		}
		
		//The device is identified, close the connection
		close(fd_connexion);
	}

	return 0;
}
