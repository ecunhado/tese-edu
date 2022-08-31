#include <string>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define DEFAULT_PORT  "/dev/ttyS0"
#define DEFAULT_BAUDRATE 9600
#define TELEGRAM_LEN 1024

class SerialPort
{
public:
    virtual bool Close();

    SerialPort();

    virtual ~SerialPort();

    char GetTermCharacter();
    void SetTermCharacter(char cTermChar);

    /** Create and set up the port */
    virtual bool Create(const char * pPortNum=DEFAULT_PORT, int nBaudRate=DEFAULT_BAUDRATE);

    /** Write a string out of port */
    int Write(const char* Str,int nLen);
    
    /** send break signal */
    virtual void Break();

    /** Flush the Port */
    virtual int Flush();

    /** returns the file descriptor */
    int GetFD();

    //returns a complete telegram in sTelegram (block of terminated characters)
    int GetTelegram(char **pData,double dfTimeOut=1,double *pTime=NULL);
    
protected:

    //termination character the serial port is interested in
    char m_cTermCharacter;

    /** FileDescriptor of Port */
    int m_nPortFD;

    char Buffer[TELEGRAM_LEN];
	
    /** Just grab N characters NOW */
    virtual int GrabN(char * pBuffer,int nRequired);

    bool IsCompleteReply(char * pData,int nLen, int nRead);

      struct termios m_OldPortOptions;
      struct termios m_PortOptions;
    


};
