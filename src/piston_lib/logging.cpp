#include "logging.h"


static const char* kLevelStrTrace = "TRACE";
static const char* kLevelStrDebug = "DEBUG";
static const char* kLevelStrInfo = "INFO";
static const char* kLevelStrWarning = "WARNING";
static const char* kLevelStrError = "ERROR";
static const char* kLevelStrFatal = "FATAL";

namespace Piston {

LoggerStream Logger::getStream(LogLevel level) {
	return LoggerStream(*this, level);
}

void Logger::processQueue() {
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
				default:
					pLevelStr = kLevelStrFatal;
					break;
			}
			std::cout << "[" << entry.timestamp << "] [" << pLevelStr << "] " << entry.message << std::endl;
		}
	}
}


} // namespace Piston
