#pragma once
#include <stdexcept>
namespace franka { struct Exception : std::runtime_error { using std::runtime_error::runtime_error; }; }
