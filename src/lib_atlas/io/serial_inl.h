/* Copyright 2012 William Woodall and John Harrison */

#ifndef LIB_ATLAS_IO_SERIAL_H_
#error This file may only be included from serial.h
#endif

#include <algorithm>

#if !defined(__OpenBSD__)
#include <alloca.h>
#endif

#include <lib_atlas/io/serial.h>
#include <lib_atlas/io/details/serial_impl.h>

namespace atlas {

//==============================================================================
// I N N E R   C L A S S   S E C T I O N

//------------------------------------------------------------------------------
//
class Serial::ScopedReadLock {
 public:
  ScopedReadLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->ReadLock();
  }
  ~ScopedReadLock() { this->pimpl_->ReadUnlock(); }

 private:
  // Disable copy constructors
  ScopedReadLock(const ScopedReadLock &) = delete;
  const ScopedReadLock &operator=(ScopedReadLock) = delete;

  SerialImpl *pimpl_;
};

//------------------------------------------------------------------------------
//
class Serial::ScopedWriteLock {
 public:
  ScopedWriteLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->WriteLock();
  }
  ~ScopedWriteLock() { this->pimpl_->WriteUnlock(); }

 private:
  // Disable copy constructors
  ScopedWriteLock(const ScopedWriteLock &) = delete;
  const ScopedWriteLock &operator=(ScopedWriteLock) = delete;
  SerialImpl *pimpl_;
};

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Serial::Serial(const std::string &port, uint32_t baudrate, Timeout timeout,
               bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
               flowcontrol_t flowcontrol)
    : pimpl_(new SerialImpl(port, baudrate, bytesize, parity, stopbits,
                            flowcontrol)) {
  pimpl_->SetTimeout(timeout);
}

//------------------------------------------------------------------------------
//
Serial::~Serial() { delete pimpl_; }

//==============================================================================
// M E T H O D S   S E C T I O N

//------------------------------------------------------------------------------
//
void Serial::open() { pimpl_->Open(); }

//------------------------------------------------------------------------------
//
void Serial::close() { pimpl_->Close(); }

//------------------------------------------------------------------------------
//
bool Serial::isOpen() const { return pimpl_->IsOpen(); }

//------------------------------------------------------------------------------
//
size_t Serial::available() { return pimpl_->Available(); }

//------------------------------------------------------------------------------
//
bool Serial::waitReadable() {
  Timeout timeout(pimpl_->GetTimeout());
  return pimpl_->WaitReadable(timeout.read_timeout_constant);
}

//------------------------------------------------------------------------------
//
void Serial::waitByteTimes(size_t count) { pimpl_->WaitByteTimes(count); }

//------------------------------------------------------------------------------
//
size_t Serial::read_(uint8_t *buffer, size_t size) {
  return this->pimpl_->Read(buffer, size);
}

//------------------------------------------------------------------------------
//
size_t Serial::read(uint8_t *buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  return this->pimpl_->Read(buffer, size);
}

//------------------------------------------------------------------------------
//
size_t Serial::read(std::vector<uint8_t> &buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = this->pimpl_->Read(buffer_, size);
  buffer.insert(buffer.end(), buffer_, buffer_ + bytes_read);
  delete[] buffer_;
  return bytes_read;
}

//------------------------------------------------------------------------------
//
size_t Serial::read(std::string &buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = this->pimpl_->Read(buffer_, size);
  buffer.append(reinterpret_cast<const char *>(buffer_), bytes_read);
  delete[] buffer_;
  return bytes_read;
}

//------------------------------------------------------------------------------
//
std::string Serial::read(size_t size) {
  std::string buffer;
  this->read(buffer, size);
  return buffer;
}

//------------------------------------------------------------------------------
//
size_t Serial::readline(std::string &buffer, size_t size, std::string eol) {
  ScopedReadLock lock(this->pimpl_);
  size_t eol_len = eol.length();
  uint8_t *buffer_ = static_cast<uint8_t *>(alloca(size * sizeof(uint8_t)));
  size_t read_so_far = 0;
  while (true) {
    size_t bytes_read = this->read_(buffer_ + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break;  // Timeout occured on reading 1 byte
    }
    if (std::string(
            reinterpret_cast<const char *>(buffer_ + read_so_far - eol_len),
            eol_len) == eol) {
      break;  // EOL found
    }
    if (read_so_far == size) {
      break;  // Reached the maximum read length
    }
  }
  buffer.append(reinterpret_cast<const char *>(buffer_), read_so_far);
  return read_so_far;
}

//------------------------------------------------------------------------------
//
std::string Serial::readline(size_t size, std::string eol) {
  std::string buffer;
  this->readline(buffer, size, eol);
  return buffer;
}

//------------------------------------------------------------------------------
//
std::vector<std::string> Serial::readlines(size_t size, std::string eol) {
  ScopedReadLock lock(this->pimpl_);
  std::vector<std::string> lines;
  size_t eol_len = eol.length();
  uint8_t *buffer_ = static_cast<uint8_t *>(alloca(size * sizeof(uint8_t)));
  size_t read_so_far = 0;
  size_t start_of_line = 0;
  while (read_so_far < size) {
    size_t bytes_read = this->read_(buffer_ + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      if (start_of_line != read_so_far) {
        lines.push_back(
            std::string(reinterpret_cast<const char *>(buffer_ + start_of_line),
                        read_so_far - start_of_line));
      }
      break;  // Timeout occured on reading 1 byte
    }
    if (std::string(
            reinterpret_cast<const char *>(buffer_ + read_so_far - eol_len),
            eol_len) == eol) {
      // EOL found
      lines.push_back(
          std::string(reinterpret_cast<const char *>(buffer_ + start_of_line),
                      read_so_far - start_of_line));
      start_of_line = read_so_far;
    }
    if (read_so_far == size) {
      if (start_of_line != read_so_far) {
        lines.push_back(
            std::string(reinterpret_cast<const char *>(buffer_ + start_of_line),
                        read_so_far - start_of_line));
      }
      break;  // Reached the maximum read length
    }
  }
  return lines;
}

//------------------------------------------------------------------------------
//
size_t Serial::write(const std::string &data) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(reinterpret_cast<const uint8_t *>(data.c_str()),
                      data.length());
}

//------------------------------------------------------------------------------
//
size_t Serial::write(const std::vector<uint8_t> &data) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(&data[0], data.size());
}

//------------------------------------------------------------------------------
//
size_t Serial::write(const uint8_t *data, size_t size) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(data, size);
}

//------------------------------------------------------------------------------
//
size_t Serial::write_(const uint8_t *data, size_t length) {
  return pimpl_->Write(data, length);
}

//------------------------------------------------------------------------------
//
void Serial::setPort(const std::string &port) {
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  bool was_open = pimpl_->IsOpen();
  if (was_open) close();
  pimpl_->SetPort(port);
  if (was_open) open();
}

//------------------------------------------------------------------------------
//
std::string Serial::getPort() const { return pimpl_->GetPort(); }

//------------------------------------------------------------------------------
//
void Serial::setTimeout(Timeout &timeout) { pimpl_->SetTimeout(timeout); }

//------------------------------------------------------------------------------
//
Timeout Serial::getTimeout() const { return pimpl_->GetTimeout(); }

//------------------------------------------------------------------------------
//
void Serial::setBaudrate(uint32_t baudrate) { pimpl_->SetBaudrate(baudrate); }

//------------------------------------------------------------------------------
//
uint32_t Serial::getBaudrate() const { return uint32_t(pimpl_->GetBaudrate()); }

//------------------------------------------------------------------------------
//
void Serial::setBytesize(bytesize_t bytesize) { pimpl_->SetBytesize(bytesize); }

//------------------------------------------------------------------------------
//
bytesize_t Serial::getBytesize() const { return pimpl_->GetBytesize(); }

//------------------------------------------------------------------------------
//
void Serial::setParity(parity_t parity) { pimpl_->SetParity(parity); }

//------------------------------------------------------------------------------
//
parity_t Serial::getParity() const { return pimpl_->GetParity(); }

//------------------------------------------------------------------------------
//
void Serial::setStopbits(stopbits_t stopbits) { pimpl_->SetStopbits(stopbits); }

//------------------------------------------------------------------------------
//
stopbits_t Serial::getStopbits() const { return pimpl_->GetStopbits(); }

//------------------------------------------------------------------------------
//
void Serial::setFlowcontrol(flowcontrol_t flowcontrol) {
  pimpl_->SetFlowcontrol(flowcontrol);
}

//------------------------------------------------------------------------------
//
flowcontrol_t Serial::getFlowcontrol() const {
  return pimpl_->GetFlowcontrol();
}

//------------------------------------------------------------------------------
//
void Serial::flush() {
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  pimpl_->Flush();
}

//------------------------------------------------------------------------------
//
void Serial::flushInput() {
  ScopedReadLock lock(this->pimpl_);
  pimpl_->FlushInput();
}

//------------------------------------------------------------------------------
//
void Serial::flushOutput() {
  ScopedWriteLock lock(this->pimpl_);
  pimpl_->FlushOutput();
}

//------------------------------------------------------------------------------
//
void Serial::sendBreak(int duration) { pimpl_->SendBreak(duration); }

//------------------------------------------------------------------------------
//
void Serial::setBreak(bool level) { pimpl_->SetBreak(level); }

//------------------------------------------------------------------------------
//
void Serial::setRTS(bool level) { pimpl_->SetRTS(level); }

//------------------------------------------------------------------------------
//
void Serial::setDTR(bool level) { pimpl_->SetDTR(level); }

//------------------------------------------------------------------------------
//
bool Serial::waitForChange() { return pimpl_->WaitForChange(); }

//------------------------------------------------------------------------------
//
bool Serial::getCTS() { return pimpl_->GetCTS(); }

//------------------------------------------------------------------------------
//
bool Serial::getDSR() { return pimpl_->GetDSR(); }

//------------------------------------------------------------------------------
//
bool Serial::getRI() { return pimpl_->GetRI(); }

//------------------------------------------------------------------------------
//
bool Serial::getCD() { return pimpl_->GetCD(); }

}  // namespace atlas
