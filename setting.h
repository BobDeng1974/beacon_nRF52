#include "config/custom_board.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

/*
#define printf(...)          \
  NRF_LOG_INFO(__VA_ARGS__); \
  NRF_LOG_FLUSH()
#define print(...)           \
  NRF_LOG_INFO(__VA_ARGS__); \
  NRF_LOG_FLUSH()

/*/
#define printf(...)                 \
  do                                \
  {                                 \
    char str[256];                  \
    sprintf(str, __VA_ARGS__);      \
    NRF_LOG_INFO(__VA_ARGS__);      \
  } while (0)
#define print(...) printf(__VA_ARGS__)
//*/

/*
#ifdef USE_SEGGER_RTT
  #include <stdio.h>
  #include <string.h>
  #include "SEGGER_RTT.h"
  #include "SEGGER_RTT_Conf.h"
  #include "sdlog.h"
  #define RTT_PRINTF(...) \
  do { \
       char str[128];\
       sprintf(str, __VA_ARGS__);\
       SEGGER_RTT_WriteString(0, str);\
       sdlog(str);\
   } while(0)
  #define printf RTT_PRINTF

  #define RTT_PRINT(str) \
  do { \
       if (strlen(str) >= BUFFER_SIZE_UP) {SEGGER_RTT_WriteString(0, "\n----dekai----\n\n");break;}\
       SEGGER_RTT_WriteString(0, str);\
       sdlog(str);\
   } while(0)
  #define print RTT_PRINT
#else
  #define printf(...)
  #define print(str)
#endif
*/