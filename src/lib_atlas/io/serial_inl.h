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
void Serial::Open() { pimpl_->Open(); }

//------------------------------------------------------------------------------
//
void Serial::Close() { pimpl_->Close(); }

//------------------------------------------------------------------------------
//
bool Serial::IsOpen() const { return pimpl_->IsOpen(); }

//------------------------------------------------------------------------------
//
size_t Serial::Available() { return pimpl_->Available(); }

//------------------------------------------------------------------------------
//
bool Serial::WaitReadable() {
  Timeout timeout(pimpl_->GetTimeout());
  return pimpl_->WaitReadable(timeout.read_timeout_constant);
}

//------------------------------------------------------------------------------
//
void Serial::WaitByteTimes(size_t count) { pimpl_->WaitByteTimes(count); }

//------------------------------------------------------------------------------
//
size_t Serial::read_(uint8_t *buffer, size_t size) {
  return this->pimpl_->Read(buffer, size);
}

//------------------------------------------------------------------------------
//
size_t Serial::Read(uint8_t *buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  return this->pimpl_->Read(buffer, size);
}

//------------------------------------------------------------------------------
//
size_t Serial::Read(std::vector<uint8_t> &buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = this->pimpl_->Read(buffer_, size);
  buffer.insert(buffer.end(), buffer_, buffer_ + bytes_read);
  delete[] buffer_;
  return bytes_read;
}

//------------------------------------------------------------------------------
//
size_t Serial::Read(std::string &buffer, size_t size) {
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = this->pimpl_->Read(buffer_, size);
  buffer.append(reinterpret_cast<const char *>(buffer_), bytes_read);
  delete[] buffer_;
  return bytes_read;
}

//------------------------------------------------------------------------------
//
std::string Serial::Read(size_t size) {
  std::string buffer;
  this->Read(buffer, size);
  return buffer;
}

//------------------------------------------------------------------------------
//
size_t Serial::ReadLine(std::string &buffer, size_t size, std::string eol) {
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
std::string Serial::ReadLine(size_t size, std::string eol) {
  std::string buffer;
  this->ReadLine(buffer, size, eol);
  return buffer;
}

//------------------------------------------------------------------------------
//
std::vector<std::string> Serial::ReadLines(size_t size, std::string eol) {
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
size_t Serial::Write(const std::string &data) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(reinterpret_cast<const uint8_t *>(data.c_str()),
                      data.length());
}

//------------------------------------------------------------------------------
//
size_t Serial::Write(const std::vector<uint8_t> &data) {
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(&data[0], data.size());
}

//------------------------------------------------------------------------------
//
size_t Serial::Write(const uint8_t *data, size_t size) {
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
void Serial::SetPort(const std::string &port) {
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  bool was_open = pimpl_->IsOpen();
  if (was_open) Close();
  pimpl_->SetPort(port);
  if (was_open) Open();
}

//------------------------------------------------------------------------------
//
std::string Serial::GetPort() const { return pimpl_->GetPort(); }

//------------------------------------------------------------------------------
//
void Serial::SetTimeout(Timeout &timeout) { pimpl_->SetTimeout(timeout); }

//------------------------------------------------------------------------------
//
Timeout Serial::GetTimeout() const { return pimpl_->GetTimeout(); }

//------------------------------------------------------------------------------
//
void Serial::SetBaudrate(uint32_t baudrate) { pimpl_->SetBaudrate(baudrate); }

//------------------------------------------------------------------------------
//
uint32_t Serial::GetBaudrate() const { return uint32_t(pimpl_->GetBaudrate()); }

//------------------------------------------------------------------------------
//
void Serial::SetBytesize(bytesize_t bytesize) { pimpl_->SetBytesize(bytesize); }

//------------------------------------------------------------------------------
//
bytesize_t Serial::GetBytesize() const { return pimpl_->GetBytesize(); }

//------------------------------------------------------------------------------
//
void Serial::SetParity(parity_t parity) { pimpl_->SetParity(parity); }

//------------------------------------------------------------------------------
//
parity_t Serial::GetParity() const { return pimpl_->GetParity(); }

//------------------------------------------------------------------------------
//
void Serial::SetStopbits(stopbits_t stopbits) { pimpl_->SetStopbits(stopbits); }

//------------------------------------------------------------------------------
//
stopbits_t Serial::GetStopbits() const { return pimpl_->GetStopbits(); }

//------------------------------------------------------------------------------
//
void Serial::SetFlowcontrol(flowcontrol_t flowcontrol) {
  pimpl_->SetFlowcontrol(flowcontrol);
}

//------------------------------------------------------------------------------
//
flowcontrol_t Serial::GetFlowcontrol() const {
  return pimpl_->GetFlowcontrol();
}

//------------------------------------------------------------------------------
//
void Serial::Flush() {
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  pimpl_->Flush();
}

//------------------------------------------------------------------------------
//
void Serial::FlushInput() {
  ScopedReadLock lock(this->pimpl_);
  pimpl_->FlushInput();
}

//------------------------------------------------------------------------------
//
void Serial::FlushOutput() {
  ScopedWriteLock lock(this->pimpl_);
  pimpl_->FlushOutput();
}

//------------------------------------------------------------------------------
//
void Serial::SendBreak(int duration) { pimpl_->SendBreak(duration); }

//------------------------------------------------------------------------------
//
void Serial::SetBreak(bool level) { pimpl_->SetBreak(level); }

//------------------------------------------------------------------------------
//
void Serial::SetRTS(bool level) { pimpl_->SetRTS(level); }

//------------------------------------------------------------------------------
//
void Serial::SetDTR(bool level) { pimpl_->SetDTR(level); }

//------------------------------------------------------------------------------
//
bool Serial::WaitForChange() { return pimpl_->WaitForChange(); }

//------------------------------------------------------------------------------
//
bool Serial::GetCTS() { return pimpl_->GetCTS(); }

//------------------------------------------------------------------------------
//
bool Serial::GetDSR() { return pimpl_->GetDSR(); }

//------------------------------------------------------------------------------
//
bool Serial::GetRI() { return pimpl_->GetRI(); }

//------------------------------------------------------------------------------
//
bool Serial::GetCD() { return pimpl_->GetCD(); }

}  // namespace atlas
