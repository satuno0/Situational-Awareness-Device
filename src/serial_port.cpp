#include "serial_port.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#if defined(_WIN32)
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace sa {
namespace {

#if !defined(_WIN32)
speed_t baudToSpeed(int baud)
{
    switch (baud)
    {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
    }
}
#endif

} // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& device, int baud_rate, std::string& error)
{
    close();
    if (device.empty())
    {
        error = "serial device path is empty";
        return false;
    }

#if defined(_WIN32)
    std::string path = device;
    if (path.rfind("\\\\.\\", 0) != 0) path = "\\\\.\\" + path;
    HANDLE handle = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                                FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle == INVALID_HANDLE_VALUE)
    {
        error = "failed to open " + device + " (Win32 error " + std::to_string(GetLastError()) + ")";
        return false;
    }
    SetupComm(handle, 64 * 1024, 64 * 1024);
    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle, &dcb))
    {
        error = "GetCommState failed for " + device;
        CloseHandle(handle);
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(baud_rate);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    if (!SetCommState(handle, &dcb))
    {
        error = "SetCommState failed for " + device;
        CloseHandle(handle);
        return false;
    }
    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 20;
    SetCommTimeouts(handle, &timeouts);
    PurgeComm(handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
    handle_ = reinterpret_cast<intptr_t>(handle);
    return true;
#else
    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        error = "failed to open " + device + ": " + std::strerror(errno);
        return false;
    }
    termios tty{};
    if (tcgetattr(fd, &tty) != 0)
    {
        error = "tcgetattr failed for " + device + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    const speed_t speed = baudToSpeed(baud_rate);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        error = "tcsetattr failed for " + device + ": " + std::strerror(errno);
        ::close(fd);
        return false;
    }
    tcflush(fd, TCIOFLUSH);
    handle_ = static_cast<intptr_t>(fd);
    return true;
#endif
}

void SerialPort::close()
{
    if (handle_ == -1) return;
#if defined(_WIN32)
    CloseHandle(reinterpret_cast<HANDLE>(handle_));
#else
    ::close(static_cast<int>(handle_));
#endif
    handle_ = -1;
}

bool SerialPort::readAvailable(std::vector<uint8_t>& out, std::string& error)
{
    if (handle_ == -1)
    {
        error = "serial port is not open";
        return false;
    }
    uint8_t buffer[4096];
#if defined(_WIN32)
    HANDLE handle = reinterpret_cast<HANDLE>(handle_);
    while (true)
    {
        DWORD got = 0;
        if (!ReadFile(handle, buffer, sizeof(buffer), &got, nullptr))
        {
            const DWORD err = GetLastError();
            if (err == ERROR_OPERATION_ABORTED) return true;
            error = "serial read failed (Win32 error " + std::to_string(err) + ")";
            return false;
        }
        if (got == 0) break;
        out.insert(out.end(), buffer, buffer + got);
        if (got < sizeof(buffer)) break;
    }
    return true;
#else
    const int fd = static_cast<int>(handle_);
    while (true)
    {
        const ssize_t got = ::read(fd, buffer, sizeof(buffer));
        if (got > 0)
        {
            out.insert(out.end(), buffer, buffer + got);
            if (static_cast<std::size_t>(got) < sizeof(buffer)) break;
            continue;
        }
        if (got == 0) break;
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        if (errno == EINTR) continue;
        error = std::string("serial read failed: ") + std::strerror(errno);
        return false;
    }
    return true;
#endif
}

bool SerialPort::writeAll(const uint8_t* data, std::size_t size, std::string& error)
{
    if (handle_ == -1)
    {
        error = "serial port is not open";
        return false;
    }
    std::size_t offset = 0;
#if defined(_WIN32)
    HANDLE handle = reinterpret_cast<HANDLE>(handle_);
    while (offset < size)
    {
        DWORD wrote = 0;
        if (!WriteFile(handle, data + offset, static_cast<DWORD>(size - offset), &wrote, nullptr))
        {
            error = "serial write failed (Win32 error " + std::to_string(GetLastError()) + ")";
            return false;
        }
        if (wrote == 0) return false;
        offset += wrote;
    }
#else
    const int fd = static_cast<int>(handle_);
    int retries = 0;
    while (offset < size)
    {
        const ssize_t wrote = ::write(fd, data + offset, size - offset);
        if (wrote > 0)
        {
            offset += static_cast<std::size_t>(wrote);
            retries = 0;
            continue;
        }
        if (wrote < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
            if (++retries > 200)
            {
                error = "serial write timed out";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (wrote < 0 && errno == EINTR) continue;
        error = std::string("serial write failed: ") + std::strerror(errno);
        return false;
    }
#endif
    return true;
}

bool SerialPort::writeAll(const std::string& text, std::string& error)
{
    return writeAll(reinterpret_cast<const uint8_t*>(text.data()), text.size(), error);
}

} // namespace sa
