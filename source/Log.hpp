//
// Created by syl on 14/03/2019.
//

#pragma once

#include <string>
#include <iostream>
#include <chrono>

/**
 * New line definition
 * Windows uses `\r\n` to represent new lines
 * Instead, most of Unix based system use '\n' for new lines
 */
#ifdef _WIN32
#define LOG_NL "\r\n"
#else
#define LOG_NL "\n"
#endif

/**
 * Return current date in the format Day Month Date hh:mm:ss Year
 */
inline std::string get_current_time() noexcept {
  auto        tp   = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string time = ::ctime(&tp);

  return time.substr(0, time.size() - 1);
}

template <typename T>
inline std::string to_string([[maybe_unused]] T arg) {
  return typeid(T).name();
}
template <typename T>
inline std::string to_string([[maybe_unused]] T *arg) {
  return typeid(T).name();
}
template <>
inline std::string to_string<std::string>(std::string arg) {
  return arg;
}
template <>
inline std::string to_string<char *>(char *ptr) {
  return (ptr);
}
template <>
inline std::string to_string<int>(int arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<unsigned>(unsigned arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<long>(long arg) {
  return std::to_string(arg);
}

template <>
inline std::string to_string<unsigned long>(unsigned long arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<long long>(long long arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<unsigned long long>(unsigned long long arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<float>(float arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<double>(double arg) {
  return std::to_string(arg);
}
template <>
inline std::string to_string<long double>(long double arg) {
  return std::to_string(arg);
}

enum class LogLevel { Trace, Debug, Info, Warn, Error, Critical };

class Log {
private:
  inline static constexpr auto LevelToString(LogLevel level) {
    switch (level) {
      case LogLevel::Trace:
        return "Trace";
      case LogLevel::Debug:
        return "Debug";
      case LogLevel::Info:
        return "Info";
      case LogLevel::Warn:
        return "Warn";
      case LogLevel::Error:
        return "Error";
      case LogLevel::Critical:
        return "Critical";
    }
  }

public:
  inline explicit Log(const char *name) noexcept : mName(name) {}

  inline Log(const char *name, LogLevel level) noexcept : mName(name), mLevel(level) {}

  Log(const Log &logger) = delete;

  Log &operator=(const Log &logger) = delete;

  template <typename... Args>
  inline void Write(LogLevel level, const char *fmt, Args &&... args) const noexcept {
    this->Write(level, FormatMsg(fmt, std::forward<Args>(args)...));
  }

  inline void Write(LogLevel level, const std::string &msg) const noexcept {
    this->Write(level, msg.c_str());
  }

  inline void Write(LogLevel level, const char *msg) const noexcept {
    if (!IsLoggable(level))
      return;
    std::cout << "[" << get_current_time() << "][" << LevelToString(level) << "][" << mName << "]: " << msg << std::endl;
  }

  template <typename... Args>
  inline void Trace(const char *fmt, Args &&... args) const noexcept {
    this->Write(LogLevel::Trace, fmt, args...);
  }

  inline void Trace(const char *msg) const noexcept {
    this->Write(LogLevel::Trace, msg);
  }

  template <typename... Args>
  inline void Debug(const char *fmt, Args &&... args) const noexcept {
    this->Write(LogLevel::Debug, fmt, args...);
  }

  inline void Debug(const char *msg) const noexcept {
    this->Write(LogLevel::Debug, msg);
  }

  template <typename... Args>
  inline void Info(const char *fmt, Args &&... args) const noexcept {
    this->Write(LogLevel::Info, fmt, args...);
  }

  inline void Info(const char *msg) const noexcept {
    this->Write(LogLevel::Info, msg);
  }

  template <typename... Args>
  inline void Warn(const char *fmt, Args &&... args) const noexcept {
    this->Write(LogLevel::Warn, fmt, args...);
  }

  inline void Warn(const char *msg) const noexcept {
    this->Write(LogLevel::Warn, msg);
  }

  template <typename... Args>
  inline void Error(const char *fmt, Args &&... args) const noexcept {
    this->Write(LogLevel::Error, fmt, args...);
  }

  inline void Error(const char *msg) const noexcept {
    this->Write(LogLevel::Error, msg);
  }

  template <typename... Args>
  inline void Critical(const char *fmt, Args &&... args) const noexcept {
    this->Write(LogLevel::Critical, fmt, args...);
  }

  inline void Critical(const char *msg) const noexcept {
    this->Write(LogLevel::Critical, msg);
  }

  inline bool IsLoggable(LogLevel level) const noexcept {
    return level >= mLevel;
  }

  inline static LogLevel DefaultLevel() noexcept {
#ifdef ENGINE_DEBUG
    return LogLevel::Trace;
#else
    return LogLevel::Info;
#endif
  }

private:
  inline void FormatMsg(std::string &to_format, std::size_t &arg_pos) const noexcept {}

  template <typename Arg, typename... Args>
  inline void FormatMsg(std::string &to_format, std::size_t &arg_pos, Arg &&arg, Args &&... args) const noexcept {
    std::size_t format_pos = to_format.find("{" + std::to_string(arg_pos) + "}");
    if (format_pos == std::string::npos)
      return;
    to_format.replace(format_pos, 3, to_string(arg));
    arg_pos++;
    FormatMsg(to_format, arg_pos, args...);
  }

  template <typename... Args>
  inline std::string FormatMsg(const char *fmt, Args &&... args) const noexcept {
    std::size_t arg_pos{0};
    std::string to_format{fmt};
    FormatMsg(to_format, arg_pos, args...);
    return to_format;
  }

private:
  std::string mName;
  LogLevel    mLevel{DefaultLevel()};
};
