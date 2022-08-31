/* This code comes from CMOOSLinuxSerialPort.cpp inside MOOS */
#include <ros/ros.h>
#include "SerialPort.h"

/** constructor. */
SerialPort::SerialPort()
{
    m_nPortFD = -1;
    m_cTermCharacter = '\r';
}

/** Destructor.Reset the port option to what every they were before and close
port*/
SerialPort::~SerialPort()
{
    Close();
}

/** Create and set up the port */
bool SerialPort::Create(const char * sPort, int nBaudRate)
{
    if (m_nPortFD >= 0)
    {
        ROS_ERROR("Serial Port already open.\n");
        return false;
    }

    int nLinuxBaudRate = B9600;
    switch(nBaudRate)
    {
    case 500000:    nLinuxBaudRate = B500000; break;
    case 115200:    nLinuxBaudRate = B115200; break;
    case 38400:     nLinuxBaudRate = B38400;  break;
    case 19200:     nLinuxBaudRate = B19200;  break;
    case 57600:     nLinuxBaudRate = B57600;  break;
    case 9600:      nLinuxBaudRate = B9600;   break;
    case 4800:      nLinuxBaudRate = B4800;   break;
    case 2400:      nLinuxBaudRate = B2400;   break;
    case 1200:      nLinuxBaudRate = B1200;   break;
    case 600:       nLinuxBaudRate = B600;    break;
    case 300:       nLinuxBaudRate = B300;    break;
    default :
        return false;
        break;
    }

    // open and configure the serial port
    m_nPortFD = open(sPort, O_RDWR | O_NOCTTY | O_NONBLOCK); //| O_NDELAY);

    if (m_nPortFD <0)
    {
        perror(sPort);
        //exit(-1);
        return  0;
    }

    //save the current configuration
    tcgetattr(m_nPortFD,&m_OldPortOptions);

    //zero the buffers
    //bzero(&m_PortOptions, sizeof(m_PortOptions));
    memset(&m_PortOptions,0,sizeof(m_PortOptions));
    m_PortOptions.c_cflag = nLinuxBaudRate | CS8 | CLOCAL | CREAD ;
    m_PortOptions.c_iflag = IGNPAR;
    m_PortOptions.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    m_PortOptions.c_lflag = 0;

    // inter-character timer unused
    m_PortOptions.c_cc[VTIME]    = 0;
    // blocking read until 0 chars received, i.e. don't block
    m_PortOptions.c_cc[VMIN]     = 1; //Check THIS

    //save the new settings
    tcflush(m_nPortFD, TCIFLUSH);
    tcsetattr(m_nPortFD,TCSANOW,&m_PortOptions);
    
    return  m_nPortFD!=0;
}



/** returns the port file descriptor */
int SerialPort::GetFD()
{
    return m_nPortFD;
}


int SerialPort::GrabN(char * pBuffer,int nRequired)
{
    if (m_nPortFD < 0)
    {
        ROS_ERROR("Can't GrabN because port is not open.");
        return 0;
    }

	int nRead = read(m_nPortFD, pBuffer, nRequired);
	return nRead;
}

/** Read a string out of a port */
//TODO: Get a NMEA message within the buffer
char *buff =NULL;
int old_nRead=0;
int SerialPort::GetTelegram(char **pData,double dfTimeOut,double *pTime)
{  
    //char pData[TELEGRAM_LEN];
    double dfTimeWaited   = 0.0;              //haven't waited any tiome yet
    double dfInterval =        0.01;             //10ms
    int nRead =            0;              //total number of chars read
    static fd_set readfds;
    int rv, n;
    struct timeval tvselect;  			//Time to wait
	
	tvselect.tv_sec=0;
	tvselect.tv_usec=dfInterval*1000000; 

	// Get the old buffer
	if(old_nRead>=TELEGRAM_LEN){
		free(buff);
		buff=NULL;
		old_nRead=0;
	}else{
		if(old_nRead>0){
			nRead = old_nRead;
			*pData = (char*) realloc (*pData, (nRead) * sizeof(char));
			memcpy(*pData, buff, nRead);
		}
	}
	//ROS_ERROR("TESTE = %d\n", old_nRead);
    while ((dfTimeWaited<dfTimeOut) && nRead<TELEGRAM_LEN)
    {
        int nGrabbed = 0;
        
        //try the read
		*pData = (char*) realloc (*pData, (nRead+1) * sizeof(char));
		
//        nGrabbed = GrabN((*pData)+nRead,1);

       	/* Perform read operation to outputs,*/
		FD_ZERO(&readfds);
		FD_SET(m_nPortFD, &readfds);
		n = m_nPortFD + 1;   
        rv = select(n, &readfds, NULL, NULL, &tvselect);
		if (rv == -1){
			// Select Error
            nGrabbed = 0;
        }else if (rv == 0){
			// Timeout occurred
			nGrabbed = 0;
        }else{
            if((nGrabbed = read(m_nPortFD, (*pData)+nRead, 1)) == 0)
                return -2; // port closed
        }
		
        if (nGrabbed == 0)
        {
            //OK wait a while...maybe it is on its way!
            dfTimeWaited+=dfInterval;
            
            ros::Duration(dfInterval).sleep();
        }
        else
        {
			//ROS_INFO("[%c]", *((*pData)+nRead));
            if(nRead==0 && pTime!=NULL)
            {
                //grab the time..                        
                *pTime = ros::Time::now().toSec();
            }
            
            
            nRead+=nGrabbed;
            
            //have we reached the end of the message?
            if(nRead>0 &&  *(*pData+nRead-1)=='\n')
            {
				*pData = (char*) realloc (*pData, (nRead+1) * sizeof(char));
                *(*pData+nRead)='\0';
                //sTelegram = pData;
                //sTelegram.replace(sTelegram.begin(), sTelegram.end(), "\r\n", ""); //Take out the end characters
                //ROS_ERROR("pData = %s\n", *pData);
				free(buff);
				buff=NULL;
				old_nRead = 0;
                return nRead;
            }            
        }
    }
    //ROS_ERROR("EXIT = %d\n", nRead);
    if(nRead>0){
		//ROS_INFO("%d Bytes were read and discarded %s", nRead, *pData);
		
		buff = (char*) realloc (buff, (nRead) * sizeof(char));
		//Save what was read for the next round
		old_nRead=nRead;
		memcpy(buff,*pData,nRead);
	}
    return -1;
}

/** Write a string out of the port. The time at which it was written is written
to *pTime */
int SerialPort::Write(const char* Str,int nLen)
{
    if (m_nPortFD < 0)
    {
        ROS_ERROR("Cannot Write because port is not open.\n");
        return 0;
    }

    int nChars = 0;

    nChars = write(m_nPortFD, Str,nLen);
    return nChars;


    for(int i = 0; i<nLen;i++)
    {
      //  #ifndef _WIN32

        if(write(m_nPortFD, &Str[i],1)!=1)
        {
            ROS_ERROR("Write Failed");
            nChars = -1;
            break;
        }
        else
        {

        nChars++;
        }
     //   #else

      //  #endif
    }

    //m_PortLock.UnLock();
    //how many chars did we write?

    return nChars;
}

/**
 *The ability to send a Break signal (~.5sec of tying the TX pin low) in Linux
 */
void SerialPort::Break()
{

    if (m_nPortFD < 0)
    {
        ROS_ERROR("Cannot Break because port is not open\n");
        return;
    }

    //according to http://www.mkssoftware.com/docs/man3/tcsendbreak.3.asp
    //a default value of 0 should work fine, sends a break for 250-500ms,
    tcsendbreak(m_nPortFD, 0);
}

/**
 *Call this method in order to free the Output Buffer of any characters
 *that may not have been sent during our last write.  We use the queue_selector
 *TCOFLUSH.
 *@see http://www.mkssoftware.com/docs/man3/tcflush.3.asp
 */
int SerialPort::Flush()
{
    if (m_nPortFD < 0)
    {
        ROS_ERROR("Cannot Flush because port is not open.");
        return -1;
    }

    int nRes = -1;
    
    nRes = tcflush(m_nPortFD, TCOFLUSH);

    return nRes;
}

bool SerialPort::Close()
{
    if (m_nPortFD < 0)
    {
        ROS_ERROR("Cannot Close because port is not open.");
        return false;
    }

    bool bResult = true;

    tcsetattr(m_nPortFD,
          TCSANOW,
          &m_OldPortOptions);

    bResult =  close(m_nPortFD)==0;
    m_nPortFD = -1;

    return bResult;
}

/**
*Sets the termination character for the serial port to watch out for
*when it constructs Telegrams for Streaming Devices.
*/

void SerialPort::SetTermCharacter(char cTermChar)
{
    m_cTermCharacter = cTermChar;
}

/*
*@return the termination character being used by the serial port.
*/
char SerialPort::GetTermCharacter()
{
    return m_cTermCharacter;
}
