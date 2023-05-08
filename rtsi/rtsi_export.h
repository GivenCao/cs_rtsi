#pragma once

#ifdef RTSI_STATIC_DEFINE
#  define RTSI_EXPORT
#  define RTSI_NO_EXPORT
#else
#  ifndef RTSI_EXPORT
#    ifdef rtsi_EXPORTS
/* We are building this library */
#      define RTSI_EXPORT __declspec(dllexport)
#    else
/* We are using this library */
#      define RTSI_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef RTSI_NO_EXPORT
#    define RTSI_NO_EXPORT 
#  endif
#endif

#ifndef RTSI_DEPRECATED
#  define RTSI_DEPRECATED __declspec(deprecated)
#endif

#ifndef RTSI_DEPRECATED_EXPORT
#  define RTSI_DEPRECATED_EXPORT RTSI_EXPORT RTSI_DEPRECATED
#endif

#ifndef RTSI_DEPRECATED_NO_EXPORT
#  define RTSI_DEPRECATED_NO_EXPORT RTSI_NO_EXPORT RTSI_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef RTSI_NO_DEPRECATED
#    define RTSI_NO_DEPRECATED
#  endif
#endif