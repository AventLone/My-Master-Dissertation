#pragma once
#include <iostream>
#include <cstring>

#define __FILENAME__ (std::strrchr(__FILE__, '/') ? std::strrchr(__FILE__, '/') + 1 : __FILE__)

#define THROW_ERROR(msg)                                                                 \
    std::string mgs_str(msg);                                                            \
    std::cerr << "[\033[1;91mError\033[0m](" << __FILENAME__ << ":" << __LINE__ << ") "; \
    throw std::runtime_error("\033[0;91m" + mgs_str + "\033[0m");

#define PRINT_ERROR(msg)                                                                  \
    std::cerr << "[\033[1;91mError\033[0m](" << __FILENAME__ << ":" << __LINE__ << ") " \
              << "\033[0;91m" << msg << "\033[0m\n"

#define PRINT_WARN(msg)                                                                  \
    std::cerr << "[\033[1;93mWarn\033[0m](" << __FILENAME__ << ":" << __LINE__ << ") " \
              << "\033[0;93m" << msg << "\033[0m\n"

#define PRINT_INFO(msg)                       \
    std::cout << "[\033[1;92mInfo\033[0m] " \
              << "\033[0;92m" << msg << "\033[0m\n"

#define PRINT_DEBUG(msg)                       \
    std::cout << "[\033[1;94mDebug\033[0m] " \
              << "\033[0;94m" << msg << "\033[0m\n"
