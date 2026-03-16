#include "logging.h"

namespace Piston {

LoggerStream Logger::getStream(LogLevel level) {
	return LoggerStream(*this, level);
}

} // namespace Piston
