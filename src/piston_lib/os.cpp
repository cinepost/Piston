#include "framework.h"
#include "os.h"

#include <string>
#include <cassert>
#include <cstdlib>

namespace Piston {

bool getEnvVar(const std::string& var_name, std::string& var_value) {
    assert(!var_name.empty());
    if(var_name.empty()) return false;

    const char* env_p = std::getenv(var_name.c_str());

    if (env_p != nullptr) {
        var_value = std::string(env_p);
        return true;
    }
    return false;
}

} // namespace Piston
