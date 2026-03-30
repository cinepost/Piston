#ifndef PISTON_LIB_LOGGING_H_
#define PISTON_LIB_LOGGING_H_

#include "framework.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

#if defined(_WIN32)

#ifdef ERROR
#undef ERROR
#endif

#endif

namespace Piston {

enum class LogLevel { TRACE, DEBUG, INFO, WARNING, ERROR, FATAL };

#ifdef PISTON_DEBUG
#define LOG_TRC Logger::getInstance().getStream(LogLevel::TRACE)
#else
#define LOG_TRC if (false) Logger::getInstance().getStream(LogLevel::TRACE)
#endif

#define LOG_DBG Logger::getInstance().getStream(LogLevel::DEBUG)
#define LOG_INF Logger::getInstance().getStream(LogLevel::INFO)
#define LOG_WRN Logger::getInstance().getStream(LogLevel::WARNING)
#define LOG_ERR Logger::getInstance().getStream(LogLevel::ERROR)
#define LOG_FTL Logger::getInstance().getStream(LogLevel::FATAL)

// Log with defotmer name
#define DLOG_TRC LOG_TRC << "[" << getName() <<  "] "
#define DLOG_DBG LOG_DBG << "[" << getName() <<  "] "
#define DLOG_INF LOG_INF << "[" << getName() <<  "] "
#define DLOG_WRN LOG_WRN << "[" << getName() <<  "] "
#define DLOG_ERR LOG_ERR << "[" << getName() <<  "] "
#define DLOG_FTL LOG_FTL << "[" << getName() <<  "] "

class LoggerStream;

// thread-safe async logger
class Logger {
	public:
		struct LogEntry {
			LogLevel level;
			std::string message;
			std::string timestamp;
		};

		static Logger& getInstance() {
			static Logger instance;
			return instance;
		}

		void log(LogLevel level, const std::string& msg) {
			if(level < mLogLevel) return;

			auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::stringstream ss;
			ss << std::put_time(std::localtime(&now), "%H:%M:%S");

			{
				std::lock_guard<std::mutex> lock(mQueueMutex);
				mLogQueue.push({level, msg, ss.str()});
			}
			mCV.notify_one();
		}

		void setOutputFile(const std::string& filename) {
			std::lock_guard<std::mutex> lock(mQueueMutex);
			if (mLogFile.is_open()) {
				mLogFile.close();
			}
			mLogFile.open(filename, std::ios::app);
		}

		void setLogLevel(LogLevel level) {
			if(mLogLevel == level) return;
			std::lock_guard<std::mutex> lock(mQueueMutex);
			mLogLevel = level;
		}

		LogLevel getLogLevel() const {
			std::lock_guard<std::mutex> lock(mQueueMutex);
			return mLogLevel;
		}

		LoggerStream getStream(LogLevel level);

		~Logger() {
			mIsRunning = false;
			mCV.notify_all();
			if (mWorker.joinable()) {
				mWorker.join();
			}
		}

	private:
		Logger(): mIsRunning(true), mLogLevel(LogLevel::DEBUG) {
			mWorker = std::thread(&Logger::processQueue, this);
		}

		Logger(const Logger&) = delete;
		Logger& operator=(const Logger&) = delete;

		void processQueue();

		std::queue<LogEntry> mLogQueue;
		mutable std::mutex mQueueMutex;
		std::condition_variable mCV;
		std::thread mWorker;
		std::atomic<bool> mIsRunning;
		std::ofstream mLogFile;
		LogLevel mLogLevel;
};

class LoggerStream {
	public:
		~LoggerStream() {
			const std::string message = mStream.str();
			if (!message.empty()) {
				mLogger.log(mLevel, message);
			}
		}

		template<typename T>
		LoggerStream& operator<<(const T& value) {
			mStream << value;
			return (*this);
		}

	private:
		friend class Logger;

		LoggerStream(Logger& logger, LogLevel level) : mLogger(logger), mLevel(level) {
		}

		Logger&  			mLogger;
		LogLevel 			mLevel;
		std::ostringstream 	mStream;
};

} // namespace Piston

#endif // PISTON_LIB_LOGGING_H_