#pragma once
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#define LOG_ENABLE 1

class DummyLogger {
 public:
  DummyLogger(const char* log_, std::ios_base::openmode mode = std::ios::app) {}
  template <typename T, typename... Args>
  void debug(const char* tag, const T& msg, const Args&... args){}
  template <typename T, typename... Args>
  void info(const char* tag, const T& msg, const Args&... args){}
  template <typename T, typename... Args>
  void warn(const char* tag, const T& msg, const Args&... args){}
  template <typename T, typename... Args>
  void error(const char* tag, const T& msg, const Args&... args){}
  template <typename T, typename... Args>
  void fatal(const char* tag, const T& msg, const Args&... args){}
};

typedef unsigned char byte;
enum class LogLevel : byte {
  Debug = 0,
  Info = 10,
  Warn = 20,
  Error = 30,
  Fatal = 40,
  None = 50,
};

class SimpleLogger {
  std::string logfile;
  std::ofstream handler;
  LogLevel limitation;

  void log_time_header() {
    auto now = std::chrono::system_clock::now();
    auto now_local = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&now_local);
    handler << '[' << now_tm->tm_hour << ':' << now_tm->tm_min << ':'
            << now_tm->tm_sec << ']';
  }
#if __cplusplus >= 201703L
  template <typename T, typename... Args>
  void _log(const T& msg, const Args&... args) {
    handler << msg;

    if constexpr (sizeof...(Args) > 0) {
      _log(args...);
    }
  }

  template <typename T, typename... Args>
  void log(const char* tag, const T& msg, const Args&... args) {
    log_time_header();
    handler << '[' << tag << ']' << msg;

    if constexpr (sizeof...(Args) > 0) {
      _log(args...);
    }
    handler << std::endl;
  }
#elif __cplusplus >= 201103L
  template <typename T, typename... Args>
  void _log(const T& msg, const Args&... args) {
    handler << msg;

    _log(args...);
  }

  template <typename T, typename... Args>
  void _log(const T& msg) {
    handler << msg;
  }

  template <typename T, typename... Args>
  void log(const char* tag, const T& msg, const Args&... args) {
    log_time_header();
    handler << '[' << tag << ']' << msg;

    _log(args...);
    handler << std::endl;
  }
#endif
 public:
  SimpleLogger(const char* log_, LogLevel level = LogLevel::Info,
               std::ios_base::openmode mode = std::ios::out)
      : logfile(log_), limitation(level), handler(log_, mode) {}
  ~SimpleLogger() { handler.close(); }

  template <typename T, typename... Args>
  void log_level(LogLevel level, const char* tag, const T& msg,
                 const Args&... args) {
    if (this->limitation <= level) {
      log(tag, msg, args...);
    }
  }

  template <typename T, typename... Args>
  void debug(const char* tag, const T& msg, const Args&... args) {
    log_level(LogLevel::Debug, tag, msg, args...);
  }
  template <typename T, typename... Args>
  void info(const char* tag, const T& msg, const Args&... args) {
    log_level(LogLevel::Info, tag, msg, args...);
  }
  template <typename T, typename... Args>
  void warn(const char* tag, const T& msg, const Args&... args) {
    log_level(LogLevel::Warn, tag, msg, args...);
  }
  template <typename T, typename... Args>
  void error(const char* tag, const T& msg, const Args&... args) {
    log_level(LogLevel::Error, tag, msg, args...);
  }
  template <typename T, typename... Args>
  void fatal(const char* tag, const T& msg, const Args&... args) {
    log_level(LogLevel::Fatal, tag, msg, args...);
  }
};

#if __cplusplus >= 201103L && __cplusplus < 201402L
// copied from C++14
template <typename _Tp, typename... _Args>
inline std::unique_ptr<_Tp> make_unique(_Args&&... __args) {
  return std::unique_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...));
}
#else
using std::make_unique;
#endif

#if LOG_ENABLE == 1
static std::unique_ptr<SimpleLogger> logger =
    make_unique<SimpleLogger>("cc.log");
#else
static std::unique_ptr<DummyLogger> logger =
    std::make_unique<DummyLogger>("cc.log");
#endif