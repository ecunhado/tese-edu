/*
 * File:   AsyncSerial.cpp
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 * Created on September 7, 2009, 10:46 AM
 *
 * v1.02: Fixed a bug in BufferedAsyncSerial: Using the default constructor
 * the callback was not set up and reading didn't work.
 *
 * v1.01: Fixed a bug that did not allow to reopen a closed serial port.
 *
 * v1.00: First release.
 *
 * IMPORTANT:
 * On Mac OS X boost asio's serial ports have bugs, and the usual implementation
 * of this class does not work. So a workaround class was written temporarily,
 * until asio (hopefully) will fix Mac compatibility for serial ports.
 *
 * Please note that unlike said in the documentation on OS X until asio will
 * be fixed serial port *writes* are *not* asynchronous, but at least
 * asynchronous *read* works.
 * In addition the serial port open ignores the following options: parity,
 * character size, flow, stop bits, and defaults to 8N1 format.
 * I know it is bad but at least it's better than nothing.
 *
 *
 */

#include "AsyncSerial.h"

#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/version.hpp> 

/* boost/version.hpp only to check which boost version is being used. Some methods changed in more recent versions
  Only here for cross compatibility */
#define OLDER_BOOST_VERSION 106501

using namespace std;
using namespace boost;

//
// Class AsyncSerial
//

class AsyncSerialImpl : private boost::noncopyable
{
public:
  AsyncSerialImpl() : io(), port(io), backgroundThread(), error(false) {}

  #if BOOST_VERSION==OLDER_BOOST_VERSION
  boost::asio::io_service io;     ///< Io service object
  #else
  boost::asio::io_context io;    //only available on newer versions of the library
  #endif
  
  boost::asio::serial_port port;   ///< Serial port object
  boost::thread backgroundThread;  ///< Thread that runs read/write operations
  bool error;                      ///< Error flag
  string error_message;            ///< Error message
  mutable boost::mutex errorMutex; ///< Mutex for access to error
  /// Data are queued here before they go in writeBuffer
  std::vector<char> writeQueue;
  boost::shared_array<char> writeBuffer; ///< Data being written
  size_t writeBufferSize;                ///< Size of writeBuffer
  boost::mutex writeQueueMutex;          ///< Mutex for access to writeQueue
  char readBuffer[AsyncSerial::readBufferSize]; ///< data being read

  /// Read complete callback
  boost::function<void(const char*, size_t)> callback;
};

AsyncSerial::AsyncSerial() : pimpl(new AsyncSerialImpl) { no_data_time = 10.0; }

AsyncSerial::AsyncSerial(const std::string& devname, unsigned int baud_rate,
                         asio::serial_port_base::parity opt_parity,
                         asio::serial_port_base::character_size opt_csize,
                         asio::serial_port_base::flow_control opt_flow,
                         asio::serial_port_base::stop_bits opt_stop,
                         double timeout_time)
    : pimpl(new AsyncSerialImpl)
{
  no_data_time = int(timeout_time);
  cout << "timeout time is " << no_data_time << endl;
  open(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop);
}

bool AsyncSerial::open(const std::string& devname, unsigned int baud_rate,
                       asio::serial_port_base::parity opt_parity,
                       asio::serial_port_base::character_size opt_csize,
                       asio::serial_port_base::flow_control opt_flow,
                       asio::serial_port_base::stop_bits opt_stop)
{
  if (isOpen())
    close();

  setErrorStatus(true); // If an exception is thrown, error_ remains true

  boost::system::error_code ec;
  try
  {
    pimpl->port.open(devname,ec);
    pimpl->port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    pimpl->port.set_option(opt_parity);
    pimpl->port.set_option(opt_csize);
    pimpl->port.set_option(opt_flow);
    pimpl->port.set_option(opt_stop);

    // This gives some work to the io_service before it is started
    #if BOOST_VERSION==OLDER_BOOST_VERSION
    // only available on older versions of the library
    timer = new boost::asio::deadline_timer(pimpl->port.get_io_service());
    #else
    timer = new boost::asio::deadline_timer(pimpl->port.get_executor());
    #endif
    pimpl->io.reset();
    pimpl->io.post(boost::bind(&AsyncSerial::doRead, this));

    # if BOOST_VERSION==OLDER_BOOST_VERSION
    boost::thread t(boost::bind(&asio::io_service::run, &pimpl->io));
    # else
    boost::thread t(boost::bind(&asio::io_context::run, &pimpl->io));
    #endif

    pimpl->backgroundThread.swap(t);
    setErrorStatus(false); // If we get here, no error
    return true;
  }
  catch (std::exception& ex)
  {
    setErrorMessage(ec.message());
    return false;
  }
}

void AsyncSerial::no_data_timeout(const boost::system::error_code& error)
{
  if (error == boost::asio::error::operation_aborted)
  {
    // cout << "timeout: aborted" << endl;
    return;
  }

  if (isOpen())
  {
    // throw std::runtime_error("timeout: no data comming on the serial port");
    setErrorMessage("Timeout: no data on the serial port");
    pimpl->port.cancel();
    doClose();
  }
}

bool AsyncSerial::isOpen() const
{
  return pimpl->port.is_open();
  // return pimpl->open;
}

bool AsyncSerial::errorStatus() const
{
  boost::lock_guard<boost::mutex> l(pimpl->errorMutex);
  return pimpl->error;
}

std::string AsyncSerial::errorMessage() const
{
  boost::lock_guard<boost::mutex> l(pimpl->errorMutex);
  return pimpl->error_message;
}

void AsyncSerial::close()
{
  if (!isOpen())
    return;

  pimpl->port.cancel();
  pimpl->io.post(boost::bind(&AsyncSerial::doClose, this));
  pimpl->backgroundThread.join();
  pimpl->io.stop();
}

void AsyncSerial::write(const char* data, size_t size)
{
  {
    boost::lock_guard<boost::mutex> l(pimpl->writeQueueMutex);
    pimpl->writeQueue.insert(pimpl->writeQueue.end(), data, data + size);
  }
  pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

void AsyncSerial::write(const std::vector<char>& data)
{
  {
    boost::lock_guard<boost::mutex> l(pimpl->writeQueueMutex);
    pimpl->writeQueue.insert(pimpl->writeQueue.end(), data.begin(), data.end());
  }
  pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

void AsyncSerial::writeString(const std::string& s)
{
  {
    boost::lock_guard<boost::mutex> l(pimpl->writeQueueMutex);
    pimpl->writeQueue.insert(pimpl->writeQueue.end(), s.begin(), s.end());
  }
  pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

AsyncSerial::~AsyncSerial()
{
  if (isOpen())
  {
    try
    {
      close();
    }
    catch (...)
    {
      // Don't throw from a destructor
    }
  }
}

void AsyncSerial::doRead()
{
  if (!isOpen())
    return;

  // wait for a very small time to reduce the load in the pc
  boost::this_thread::sleep(boost::posix_time::milliseconds(10));
  // cout << "receiving data " << endl;
  // timeout if no data arrives
  timer->expires_from_now(boost::posix_time::seconds(int(no_data_time)));
  timer->async_wait(boost::bind(&AsyncSerial::no_data_timeout, this,
                                boost::asio::placeholders::error));

  pimpl->port.async_read_some(
      asio::buffer(pimpl->readBuffer, readBufferSize),
      boost::bind(&AsyncSerial::readEnd, this, asio::placeholders::error,
                  asio::placeholders::bytes_transferred));
}

void AsyncSerial::readEnd(const boost::system::error_code& error,
                          size_t bytes_transferred)
{
  if (error)
  {
    // error can be true even because the serial port was closed.
    // In this case it is not a real error, so ignore
    if (isOpen())
    {
      setErrorStatus(true);
      doClose();
    }
  }
  else
  {
    timer->cancel();
    if (bytes_transferred != 0)
    {
      if (pimpl->callback)
        pimpl->callback(pimpl->readBuffer, bytes_transferred);
    }
    doRead();
  }
}

void AsyncSerial::doWrite()
{
  // If a write operation is already in progress, do nothing
  if (pimpl->writeBuffer == 0)
  {
    boost::lock_guard<boost::mutex> l(pimpl->writeQueueMutex);
    pimpl->writeBufferSize = pimpl->writeQueue.size();
    pimpl->writeBuffer.reset(new char[pimpl->writeQueue.size()]);
    copy(pimpl->writeQueue.begin(), pimpl->writeQueue.end(),
         pimpl->writeBuffer.get());
    pimpl->writeQueue.clear();
    async_write(
        pimpl->port,
        asio::buffer(pimpl->writeBuffer.get(), pimpl->writeBufferSize),
        boost::bind(&AsyncSerial::writeEnd, this, asio::placeholders::error));
  }
}

void AsyncSerial::writeEnd(const boost::system::error_code& error)
{
  if (!error)
  {
    boost::lock_guard<boost::mutex> l(pimpl->writeQueueMutex);
    if (pimpl->writeQueue.empty())
    {
      pimpl->writeBuffer.reset();
      pimpl->writeBufferSize = 0;

      return;
    }
    pimpl->writeBufferSize = pimpl->writeQueue.size();
    pimpl->writeBuffer.reset(new char[pimpl->writeQueue.size()]);
    copy(pimpl->writeQueue.begin(), pimpl->writeQueue.end(),
         pimpl->writeBuffer.get());
    pimpl->writeQueue.clear();
    async_write(
        pimpl->port,
        asio::buffer(pimpl->writeBuffer.get(), pimpl->writeBufferSize),
        boost::bind(&AsyncSerial::writeEnd, this, asio::placeholders::error));
  }
  else
  {
    setErrorStatus(true);
    doClose();
  }
}

void AsyncSerial::doClose()
{
  boost::system::error_code ec;
  timer->cancel();
  pimpl->port.cancel(ec);
  if (ec)
    setErrorStatus(true);
  pimpl->port.close(ec);
  if (ec)
    setErrorStatus(true);

  // pimpl->open = false;
  // cout << "io service stopped?" << std::boolalpha << pimpl->io.stopped() <<
  // endl;
}

void AsyncSerial::setErrorMessage(string err)
{
  boost::lock_guard<boost::mutex> l(pimpl->errorMutex);
  pimpl->error_message = err;
}

void AsyncSerial::setErrorStatus(bool e)
{
  boost::lock_guard<boost::mutex> l(pimpl->errorMutex);
  pimpl->error = e;
}

void AsyncSerial::setReadCallback(
    const boost::function<void(const char*, size_t)>& callback)
{
  pimpl->callback = callback;
}

void AsyncSerial::clearReadCallback() { pimpl->callback.clear(); }

//
// Class CallbackAsyncSerial
//

CallbackAsyncSerial::CallbackAsyncSerial() : AsyncSerial() {}

CallbackAsyncSerial::CallbackAsyncSerial(
    const std::string& devname, unsigned int baud_rate,
    asio::serial_port_base::parity opt_parity,
    asio::serial_port_base::character_size opt_csize,
    asio::serial_port_base::flow_control opt_flow,
    asio::serial_port_base::stop_bits opt_stop)
    : AsyncSerial(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop)
{
}

void CallbackAsyncSerial::setCallback(
    const boost::function<void(const char*, size_t)>& callback)
{
  setReadCallback(callback);
}

void CallbackAsyncSerial::clearCallback() { clearReadCallback(); }

CallbackAsyncSerial::~CallbackAsyncSerial() { clearReadCallback(); }
