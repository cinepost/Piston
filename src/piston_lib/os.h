#ifndef PISTON_LIB_OS_H_
#define PISTON_LIB_OS_H_

#include <string>

namespace Piston {

bool getEnvVar(const std::string& var_name, std::string& var_value);

} // namespace Piston

#endif // PISTON_LIB_OS_H_