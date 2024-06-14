#ifndef XMATE_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define XMATE_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define XMATE_HARDWARE_INTERFACE_EXPORT __attribute__ ((dllexport))
    #define XMATE_HARDWARE_INTERFACE_IMPORT __attribute__ ((dllimport))
  #else
    #define XMATE_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
    #define XMATE_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
  #endif
  #ifdef XMATE_HARDWARE_INTERFACE_BUILDING_LIBRARY
    #define XMATE_HARDWARE_INTERFACE_PUBLIC XMATE_HARDWARE_INTERFACE_EXPORT
  #else
    #define XMATE_HARDWARE_INTERFACE_PUBLIC XMATE_HARDWARE_INTERFACE_IMPORT
  #endif
  #define XMATE_HARDWARE_INTERFACE_PUBLIC_TYPE XMATE_HARDWARE_INTERFACE_PUBLIC
  #define XMATE_HARDWARE_INTERFACE_LOCAL
#else
  #define XMATE_HARDWARE_INTERFACE_EXPORT __attribute__ ((visibility("default")))
  #define XMATE_HARDWARE_INTERFACE_IMPORT
  #if __GNUC__ >= 4
    #define XMATE_HARDWARE_INTERFACE_PUBLIC __attribute__ ((visibility("default")))
    #define XMATE_HARDWARE_INTERFACE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define XMATE_HARDWARE_INTERFACE_PUBLIC
    #define XMATE_HARDWARE_INTERFACE_LOCAL
  #endif
  #define XMATE_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // XMATE_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
