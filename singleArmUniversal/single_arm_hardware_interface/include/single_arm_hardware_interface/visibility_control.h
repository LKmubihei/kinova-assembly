#ifndef SINGLE_ARM_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define SINGLE_ARM_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SINGLE_ARM_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define SINGLE_ARM_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define SINGLE_ARM_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define SINGLE_ARM_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef SINGLE_ARM_HARDWARE_INTERFACE_BUILDING_LIBRARY
    #define SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC SINGLE_ARM_HARDWARE_INTERFACE_EXPORT
  #else
    #define SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC SINGLE_ARM_HARDWARE_INTERFACE_IMPORT
  #endif
  #define SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC_TYPE SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC
  #define SINGLE_ARM_HARDWARE_INTERFACE_LOCAL
#else
  #define SINGLE_ARM_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define SINGLE_ARM_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define SINGLE_ARM_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC
    #define SINGLE_ARM_HARDWARE_INTERFACE_LOCAL
  #endif
  #define SINGLE_ARM_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // SINGLE_ARM_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
