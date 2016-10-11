/*
* Copyright (c) 2016, Ford Motor Company
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following
* disclaimer in the documentation and/or other materials provided with the
* distribution.
*
* Neither the name of the Ford Motor Company nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
#include "utils/logger.h"
#include "utils/log_message_loop_thread.h"
#include "utils/pimpl_impl.h"

#if defined(LOG4CXX_LOGGER)
#include <apr_time.h>
#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/spi/loggingevent.h>

__attribute__((
    visibility("hidden"))) logger::Logger::Pimpl logger::Logger::impl_;

////////////////////////////////////////////////////////////////////////////////
/// logger::Logger::Impl
////////////////////////////////////////////////////////////////////////////////

class logger::Logger::Impl {
 public:
  Impl();
  ~Impl();

  bool InitLogger(const bool logs_enabled, const std::string& ini_file_name);
  bool InitLogger(const bool logs_enabled,
                  const LogLevel::Type log_level,
                  const std::string& log_file_name);
  void DeinitLogger();

  void FlushLogger();

  bool logs_enabled() const;
  void set_logs_enabled(const bool state);

  LogLevel::Type log_level() const;
  void set_log_level(const LogLevel::Type level);

  bool PushLog(const LoggerType& logger,
               const LogLevel::Type level,
               const std::string& entry,
               const LogLocation& location);

 private:
  bool logs_enabled_;
  LogLevel::Type log_level_;
  logger::LogMessageLoopThread* message_loop_thread_;
};

logger::Logger::Impl::Impl()
    : logs_enabled_(false)
    , log_level_(LogLevel::LL_TRACE)
    , message_loop_thread_(NULL) {}

logger::Logger::Impl::~Impl() {}

bool logger::Logger::Impl::InitLogger(const bool logs_enabled,
                                      const std::string& ini_file_name) {
  if (message_loop_thread_) {
    return false;
  }
  set_logs_enabled(logs_enabled);
  log4cxx::PropertyConfigurator::configure(ini_file_name);
  message_loop_thread_ = new LogMessageLoopThread();
  return true;
}

bool logger::Logger::Impl::InitLogger(const bool logs_enabled,
                                      const LogLevel::Type log_level,
                                      const std::string& log_file_name) {
  return false;
}

namespace {
void RemoveLocalAppenders(log4cxx::LoggerPtr logger) {
  logger->removeAllAppenders();
}
}

void logger::Logger::Impl::DeinitLogger() {
  SDL_CREATE_LOGGER("Logger");
  SDL_DEBUG("Logger deinitialization");

  set_logs_enabled(false);
  set_log_level(LogLevel::LL_TRACE);
  delete message_loop_thread_;
  message_loop_thread_ = NULL;

  // 3rd-party logger deinitialization
  log4cxx::LoggerPtr rootLogger = log4cxx::Logger::getRootLogger();
  log4cxx::spi::LoggerRepositoryPtr repository =
      rootLogger->getLoggerRepository();
  log4cxx::LoggerList loggers = repository->getCurrentLoggers();
  std::for_each(loggers.begin(), loggers.end(), RemoveLocalAppenders);
  rootLogger->removeAllAppenders();
}

void logger::Logger::Impl::FlushLogger() {
  if (message_loop_thread_) {
    message_loop_thread_->WaitDumpQueue();
  }
}

bool logger::Logger::Impl::logs_enabled() const {
  return logs_enabled_;
}

void logger::Logger::Impl::set_logs_enabled(const bool state) {
  logs_enabled_ = state;
}

logger::LogLevel::Type logger::Logger::Impl::log_level() const {
  return log_level_;
}

void logger::Logger::Impl::set_log_level(const logger::LogLevel::Type level) {
  log_level_ = level;
}

bool logger::Logger::Impl::PushLog(const LoggerType& logger,
                                   const LogLevel::Type level,
                                   const std::string& entry,
                                   const LogLocation& location) {
  if (!logs_enabled()) {
    return false;
  }
  if (level < log_level()) {
    return false;
  }
  LogMessage message = {logger,
                        level,
                        entry,
                        location,
                        apr_time_now(),
                        log4cxx::spi::LoggingEvent::getCurrentThreadName()};

  if (message_loop_thread_) {
    message_loop_thread_->PostMessage(message);
    return true;
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////
/// logger::Logger
////////////////////////////////////////////////////////////////////////////////

bool logger::Logger::InitLogger(const bool logs_enabled,
                                const std::string& ini_file_name) {
  return impl_->InitLogger(logs_enabled, ini_file_name);
}

bool logger::Logger::InitLogger(const bool logs_enabled,
                                const LogLevel::Type log_level,
                                const std::string& log_file_name) {
  return impl_->InitLogger(logs_enabled, log_level, log_file_name);
}

void logger::Logger::DeinitLogger() {
  impl_->DeinitLogger();
}

void logger::Logger::FlushLogger() {
  impl_->FlushLogger();
}

bool logger::Logger::logs_enabled() {
  return impl_->logs_enabled();
}

void logger::Logger::set_logs_enabled(const bool state) {
  impl_->set_logs_enabled(state);
}

logger::LogLevel::Type logger::Logger::log_level() {
  return impl_->log_level();
}

void logger::Logger::set_log_level(const LogLevel::Type level) {
  impl_->set_log_level(level);
}

bool logger::Logger::PushLog(const LoggerType& logger,
                             const LogLevel::Type level,
                             const std::string& entry,
                             const LogLocation& location) {
  return impl_->PushLog(logger, level, entry, location);
}

void logger::Logger::SetLogger(logger::Logger::Pimpl& impl) {
  impl_ = impl;
}

logger::Logger::Pimpl& logger::Logger::GetLogger() {
  return impl_;
}

#endif  // LOG4CXX_LOGGER