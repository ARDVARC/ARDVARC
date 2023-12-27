#ifndef ARDVARC__VISIBILITY_CONTROL_H_
#define ARDVARC__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ARDVARC_EXPORT __attribute__ ((dllexport))
    #define ARDVARC_IMPORT __attribute__ ((dllimport))
  #else
    #define ARDVARC_EXPORT __declspec(dllexport)
    #define ARDVARC_IMPORT __declspec(dllimport)
  #endif
  #ifdef ARDVARC_BUILDING_LIBRARY
    #define ARDVARC_PUBLIC ARDVARC_EXPORT
  #else
    #define ARDVARC_PUBLIC ARDVARC_IMPORT
  #endif
  #define ARDVARC_PUBLIC_TYPE ARDVARC_PUBLIC
  #define ARDVARC_LOCAL
#else
  #define ARDVARC_EXPORT __attribute__ ((visibility("default")))
  #define ARDVARC_IMPORT
  #if __GNUC__ >= 4
    #define ARDVARC_PUBLIC __attribute__ ((visibility("default")))
    #define ARDVARC_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ARDVARC_PUBLIC
    #define ARDVARC_LOCAL
  #endif
  #define ARDVARC_PUBLIC_TYPE
#endif
#endif  // ARDVARC__VISIBILITY_CONTROL_H_
// Generated 03-Jan-2024 13:25:48
// Copyright 2019-2020 The MathWorks, Inc.
