#ifndef PISTON_LIB_LOGGING_H_
#define PISTON_LIB_LOGGING_H_

#include <iostream>
#include <fstream>
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


namespace Piston {

static const char* kLevelStrTrace = "TRACE";
static const char* kLevelStrDebug = "DEBUG";
static const char* kLevelStrInfo = "INFO";
static const char* kLevelStrWarning = "WARNING";
static const char* kLevelStrError = "ERROR";
static const char* kLevelStrFatal = "FATAL";

enum class LogLevel { TRACE, DEBUG, INFO, WARNING, ERROR, FATAL };

// #define LOG_TRC(msg) Logger::getInstance().log(LogLevel::TRACE, msg)
// #define LOG_DBG(msg) Logger::getInstance().log(LogLevel::DEBUG, msg)
// #define LOG_INF(msg) Logger::getInstance().log(LogLevel::INFO, msg)
// #define LOG_WRN(msg) Logger::getInstance().log(LogLevel::WARNING, msg)
// #define LOG_ERR(msg) Logger::getInstance().log(LogLevel::ERROR, msg)
// #define LOG_FTL(msg) Logger::getInstance().log(LogLevel::FATAL, msg)

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

		void processQueue() {
			while (mIsRunning || !mLogQueue.empty()) {
				std::unique_lock<std::mutex> lock(mQueueMutex);
				mCV.wait(lock, [this] { 
					return !mLogQueue.empty() || !mIsRunning;
				});

				if (!mLogQueue.empty()) {
					LogEntry entry = mLogQueue.front();
					mLogQueue.pop();
					lock.unlock(); // Unlock early to allow other threads to push logs

					const char* pLevelStr = kLevelStrTrace;
					switch(entry.level) {
						case LogLevel::DEBUG:
							pLevelStr = kLevelStrDebug;
							break;
						case LogLevel::INFO:
							pLevelStr = kLevelStrInfo;
							break;
						case LogLevel::WARNING:
							pLevelStr = kLevelStrWarning;
							break;
						case LogLevel::ERROR:
							pLevelStr = kLevelStrError;
							break;
						case LogLevel::FATAL:
							pLevelStr = kLevelStrFatal;
							break;
						default:
							break;
					}
					std::cout << "[" << entry.timestamp << "] [" << pLevelStr << "] " << entry.message << std::endl;
				}
			}
		}

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