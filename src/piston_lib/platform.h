#ifndef SRC_PISTON_PLATFORM_H_
#define SRC_PISTON_PLATFORM_H_

/**
 * Compilers.
 */
#define PISTON_COMPILER_MSVC 1
#define PISTON_COMPILER_CLANG 2
#define PISTON_COMPILER_GCC 3

/**
 * Determine the compiler in use.
 * http://sourceforge.net/p/predef/wiki/Compilers/
 */
#ifndef PISTON_COMPILER
#if defined(_MSC_VER)
#define PISTON_COMPILER PISTON_COMPILER_MSVC
#elif defined(__clang__)
#define PISTON_COMPILER PISTON_COMPILER_CLANG
#elif defined(__GNUC__)
#define PISTON_COMPILER PISTON_COMPILER_GCC
#else
#error "Unsupported compiler"
#endif
#endif // PISTON_COMPILER

#define PISTON_MSVC (PISTON_COMPILER == PISTON_COMPILER_MSVC)
#define PISTON_CLANG (PISTON_COMPILER == PISTON_COMPILER_CLANG)
#define PISTON_GCC (PISTON_COMPILER == PISTON_COMPILER_GCC)

/**
 * Platforms.
 */
#define PISTON_PLATFORM_WINDOWS 1
#define PISTON_PLATFORM_LINUX 2

/**
 * Determine the target platform in use.
 * http://sourceforge.net/p/predef/wiki/OperatingSystems/
 */
#ifndef PISTON_PLATFORM
#if defined(_WIN64)
#define PISTON_PLATFORM PISTON_PLATFORM_WINDOWS
#elif defined(__linux__)
#define PISTON_PLATFORM PISTON_PLATFORM_LINUX
#else
#error "Unsupported target platform"
#endif
#endif // PISTON_PLATFORM

#define PISTON_WINDOWS (PISTON_PLATFORM == PISTON_PLATFORM_WINDOWS)
#define PISTON_LINUX (PISTON_PLATFORM == PISTON_PLATFORM_LINUX)

#endif  // SRC_PISTON_PLATFORM_H_