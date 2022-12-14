#ifndef STR_ROS2__VISIBILITY_CONTROL_H_
#define STR_ROS2__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define STR_ROS2_EXPORT __attribute__ ((dllexport))
    #define STR_ROS2_IMPORT __attribute__ ((dllimport))
  #else
    #define STR_ROS2_EXPORT __declspec(dllexport)
    #define STR_ROS2_IMPORT __declspec(dllimport)
  #endif
  #ifdef STR_ROS2_BUILDING_LIBRARY
    #define STR_ROS2_PUBLIC STR_ROS2_EXPORT
  #else
    #define STR_ROS2_PUBLIC STR_ROS2_IMPORT
  #endif
  #define STR_ROS2_PUBLIC_TYPE STR_ROS2_PUBLIC
  #define STR_ROS2_LOCAL
#else
  #define STR_ROS2_EXPORT __attribute__ ((visibility("default")))
  #define STR_ROS2_IMPORT
  #if __GNUC__ >= 4
    #define STR_ROS2_PUBLIC __attribute__ ((visibility("default")))
    #define STR_ROS2_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define STR_ROS2_PUBLIC
    #define STR_ROS2_LOCAL
  #endif
  #define STR_ROS2_PUBLIC_TYPE
#endif

#endif  // STR_ROS2__VISIBILITY_CONTROL_H_
