/*
 * File:   nmeaSerial.h
 * Author: Jorge Ribeiro
 * Created on June 17, 2016, 10:30 AM
 */
#ifndef NMEASERIAL_H
#define NMEASERIAL_H

#include "AsyncSerial.h"
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/algorithm/string.hpp>

/**
 * @brief nmeaSerial class for reading devices through serial port using nmea messages
 * 
 */
class nmeaSerial
{
public:
  // +.+ Public Variables
  std::string error_message;

  /**
   * @brief Construct a new nmea Serial object
   * 
   */
  nmeaSerial()
  {
    serial = NULL;
  }

//  nmeaSerial(const std::string& devname, const unsigned int baud_rate,
//             bool checksum = true, char startChar = '$')
//      : checksum_(checksum),
//        startChar_(startChar)
//  {
//    serial = NULL;

//    open(devname, baud_rate, checksum);
//  }
  /**
   * @brief Bind a callback
   * 
   */
  void init()
  {
    serial = new CallbackAsyncSerial();

    // bind receive callback
    serial->setCallback(boost::bind(&nmeaSerial::receivedBytes, this, _1, _2));

    // set ack default to false
    ack_waiting = false;
  }

  /**
   * @brief 
   * 
   * @param devname serial device name, example "/dev/ttyS0" or "COM1"
   * @param baud_rate serial baud rate
   * @param checksum flag to use or not checksum. Default true
   * @param startChar starting char of the message. Default '$'
   * @return true success opening the port
   * @return false failed to open the port
   */
  bool open(const std::string& devname, unsigned int baud_rate,
            bool checksum = true, char startChar = '$')
  {
    if (isOpen() || (serial!= NULL))
    {
      close();
    }

    init();

    // Set checksum flag
    checksum_ = checksum;
    startChar_ = startChar;

    // open Serial Port
    bool open_flag = serial->open(devname, baud_rate);

    if(!open_flag)
       getErrorMessage();

    return open_flag;
  }

  /**
   * @brief Destroy the nmea Serial object
   * 
   */
  ~nmeaSerial()
  {
    close();
  }

  /**
   * @brief subscribe function stores the function pointers
   * to receive notifications for each nmea message
   * Only one callback is allowed for each message.
   * @param message
   * @param callback
   */
  void subscribe(const std::string message,
                 const boost::function<void(std::vector<std::string>)>& callback)
  {
    nmea_callbacks[message] = callback;
  }

  /**
   * @brief subscriberaw function stores the function pointer
   * to be called when new data arrives.
   * @param callback
   */
  void subscriberaw(const boost::function<void(const std::string)>& callback)
  {
    raw_callback = callback;
  }

  /**
   * @brief unsubscribe function clears the function pointers
   * to stop receive notifications
   * @param message
   */
  void unsubscribe(const std::string message) { nmea_callbacks.erase(message); }

  /**
   * @brief checkSum function computes the checksum for a specific message
   * @param message
   * @return uint8_t with checksum
   */
  uint8_t checkSum(std::string message)
  {
    char check = 0;
    // iterate over the std::string, XOR each byte with the total sum:
    for (std::string::iterator it = message.begin(); it != message.end(); ++it)
    {
      check = uint8_t(check ^ *it);
    }
    // return the result
    return check;
  }

  /**
   * @brief write_nmea function writes on serial port
   * it includes '$', '*' and checksum
   * @param message
   */
  void write_nmea(const std::string message)
  {
    uint8_t ck = checkSum(message);
    std::string str = startChar_ + message;
    if (checksum_)
      str += "*" + intToHexString(ck);
    str += "\r\n";
    write(str);
  }

  /**
   * @brief write function writes the message on serial port
   * @param message
   */
  void write(const std::string message, int pub_raw = 1)
  {
    if (!isOpen())
      return;

    // For log purpose, local-echo is enable
    if (pub_raw)
    {
      if (raw_callback)
        raw_callback(message);
    }

    serial->writeString(message);
  }

  /**
   * @brief write_with_ack function writes a nmea message to the serial port
   * and waits for an ack_header
   * @param message to written on the serial_port
   * @param ack_header format of the acknowledge
   * @param nack_header format of the error acknowledge empty for nothing
   * @param timeout time to wait in milliseconds
   * @return valid or invalid ack
   */
  bool write_with_ack(const std::string message, const std::string ack_header,
                      const std::string nack_header, const double timeout_duration)
  {
    if (!isOpen())
      return false;

    write_nmea(message);

    // prepare to wait for the acknowledge message
    boost::unique_lock<boost::mutex> lock(ack_mtx);
    ack_wait_header = ack_header;
    nack_wait_header = nack_header;
    ack_message.clear();
    ack_waiting = true;

    const boost::system_time wait_until =
        boost::get_system_time() +
        boost::posix_time::milliseconds(int(timeout_duration));

    // nack_header only serves to get out faster
    while (!starts_with_or_with(ack_message, ack_header, nack_header))
    {
      // until the timeout occurs
      // TODO: maybe fire a timeout exception
      if (!ack_cond.timed_wait(lock, wait_until))
      {
        ack_waiting = false;
        return false;
      }
    }
    ack_waiting = false;
    return boost::algorithm::starts_with(ack_message, ack_header);
  }

  /**
   * @brief Check if serial port connection is open
   * 
   * @return true open 
   * @return false closed
   */
  bool isOpen()
  {
    if(serial == NULL)
      return false;

    return serial->isOpen();
  }

  /**
   * @brief Close serial port connection
   * 
   */
  void close()
  {
    bufferData.clear();

    if(serial!=NULL)
    {
      getErrorMessage();
      serial->close();
      serial->~CallbackAsyncSerial();
      serial = NULL;
    }
  }

private:

  /**
   * @brief Get the Error Message object
   * 
   */
  void getErrorMessage()
  {
    error_message = serial->errorMessage();
  }

  /**
   * @brief intToHexString converts an integer to std::string
   * @param value
   * @return
   */
  std::string intToHexString(uint8_t value)
  {
    std::string hexStr;

    // integer value to hex-std::string
    std::stringstream sstream;
    sstream << std::setfill('0') << std::setw(2) << std::hex << std::uppercase
            << (int)value;

    hexStr = sstream.str();
    sstream.clear();

    return hexStr;
  }

  /**
   * @brief starts_with_or_with tests if a std::strings starts with or another
   * @param s1
   * @param test1
   * @param test2
   * @return
   */
  bool starts_with_or_with(std::string s1, std::string test1, std::string test2)
  {
    // do not test if test2 is empty
    if (test2.empty())
      return boost::algorithm::starts_with(s1, test1);
    return (boost::algorithm::starts_with(s1, test1) ||
            boost::algorithm::starts_with(s1, test2));
  }

  /**
   * @brief processline function is called when \n is reached
   * it validates the integrity of the message and calls the correspondent
   * callbacks.
   * @param line
   */
  // void processline(vector<char> &line){
  void processline(std::string linestr)
  {
    // std::string linestr(line.begin(), line.end());

    // at least it should have  startChar_('$'), '*' and checksum
    if (linestr.length() < 3 || linestr[0] != startChar_)
      return;

    // compute message ckecksum
    std::string clean_message =
        linestr.substr(1, linestr.find("*") - 1); // no dollar neither checksum
    if (checksum_)
    {
      uint8_t computedCK = checkSum(clean_message);

      // convert the checksum to integer
      uint8_t messageCK =
          strtoul(linestr.substr(linestr.find("*") + 1).c_str(), NULL, 16);

      // check for message integrity
      if (messageCK != computedCK)
      {
        std::cerr << "nmea checksum is not valid" << std::endl
             << "[" << linestr << "]" << std::endl;
        return;
      }
    }
    // check if there's anyone waiting for an acknowlege message
    // and if the acknowlege message is the same as this one
    /*if(ack_waiting && boost::algorithm::starts_with(clean_message,
    ack_wait_header) ){
        boost::lock_guard<boost::mutex> lock(ack_mtx);
        ack_waiting = false;
        ack_reply_message = clean_message;
        ack_cond.notify_one();
    }*/
    if (ack_waiting)
    {
      boost::lock_guard<boost::mutex> lock(ack_mtx);
      ack_message = clean_message;
      ack_cond.notify_one();
    }

    // tokenize the message
    std::vector<std::string> nmea_elements;
    boost::char_separator<char> sep(",", "", boost::keep_empty_tokens);
    boost::tokenizer<boost::char_separator<char> > tok(clean_message, sep);

    // for (boost::tokenizer< boost::char_separator<char> >::iterator
    // it=tok.begin(); it!=tok.end(); ++it)
    //    cout << "tok[" << *it << "]" << endl;

    try
    {
      transform(tok.begin(), tok.end(), back_inserter(nmea_elements),
                boost::lexical_cast<std::string, std::string>);
    }
    catch (const boost::bad_lexical_cast& e)
    {
      std::cerr << "message is not valid" << std::endl
           << "[" << linestr << "]" << std::endl;
      return;
    }

    // call the callback if there's any subscriber to the message
    /*map<std::string, boost::function<void (std::vector<string >) > > ::iterator ptr
    = nmea_callbacks.find(nmea_elements[0]);
    if(ptr != nmea_callbacks.end())
        ptr->second(nmea_elements);*/

    // call all callbacks that has the std::string on the first element
    std::map<std::string, boost::function<void(std::vector<std::string>)>>::iterator ptr;
    for (ptr = nmea_callbacks.begin(); ptr != nmea_callbacks.end(); ++ptr)
    {
      size_t pos = nmea_elements[0].find(ptr->first);

      if (pos != std::string::npos)
        ptr->second(nmea_elements);
    }
  }

  /**
   * @brief receivedBytes function is called whenever new bytes
   * are received and waits for the '\n' to process the nmea line.
   * Fills a buffer to not loose any data
   * @param data
   * @param len
   */
  void receivedBytes(const char* data, unsigned int len)
  {
    // insert the received data to the buffer
    bufferData.append(data, (size_t)len);

    // instantiate variable for while loop
    std::size_t found_newline;

    // retrive complete bufferdata
    do
    {
      // for binary data
      std::size_t found_dollar = bufferData.find(startChar_);
      if (found_dollar == std::string::npos)
      {
        // cout << "there's no $ buffer is [" << bufferData << "]" << endl;
        if (raw_callback)
          raw_callback(bufferData);
        bufferData.clear();
        return;
        // found and it is not on the first position
      }
      else if (found_dollar != 0)
      {
        if (raw_callback)
          raw_callback(bufferData.substr(0, found_dollar));
        // cout << "there's is $ but not on first position" << endl;
        // clear std::string excluding dollar
        bufferData.erase(bufferData.begin(), bufferData.begin() + found_dollar);
      }

      found_newline = bufferData.find('\n');
      if (found_newline != std::string::npos)
      {
        // cout << "found new line" << endl;
        // Call the callback if there's any subscriber for the raw data
        if (raw_callback)
          raw_callback(bufferData.substr(0, found_newline + 1));

        std::string line = bufferData.substr(0, found_newline);
        boost::replace_all(line, "\r", "");
        processline(line);

        // clear std::string including new line
        bufferData.erase(bufferData.begin(),
                         bufferData.begin() + found_newline + 1);
      }
      // avoid growing indefinitely
      else if (bufferData.size() > max_buffer_size)
      {
        // cout << "buffer growed more than " << max_buffer_size << endl;
        if (raw_callback)
          raw_callback(bufferData);
        bufferData.clear();
      }
    } while (found_newline != std::string::npos && bufferData.size() != 0);
  }

  // Variables
  CallbackAsyncSerial* serial;
  bool checksum_;
  char startChar_;
  // map between std::string and function pointer
  std::map<std::string, boost::function<void(std::vector<std::string>)>> nmea_callbacks;

  // function pointer for raw data
  boost::function<void(const std::string)> raw_callback;

  // acknowlege
  bool ack_waiting;
  std::string ack_wait_header, nack_wait_header, ack_reply_message, ack_message;
  boost::condition_variable ack_cond;
  boost::mutex ack_mtx;

  std::string bufferData;
  const static size_t max_buffer_size  = 1024;
};

#endif
