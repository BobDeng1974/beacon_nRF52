#include <stdio.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "stack_macros.h"
#include "tracetask.h"

uint16_t  TraceTask_LogSw;
uint16_t  TraceTask_LogCnt = 0;
uint32_t  TraceTask_Tick = 0;

typedef struct {
  char  task[ configMAX_TASK_NAME_LEN ];
  char  marker;
  uint32_t  tick;
  UBaseType_t uxPriority;
} TraceTask_t;

#define TRACETASK_NUM 100
TraceTask_t TraceTask_LogBuff[ TRACETASK_NUM ];

void tracetask_writelog( char marker, char *task_name, uint32_t uxPriority)
{
  TraceTask_t *p;

  if (TraceTask_LogSw)
  {
    p = &TraceTask_LogBuff[ TraceTask_LogCnt ];

    strncpy( p->task, task_name, sizeof(p->task) );
    p->marker = marker;
    p->tick = TraceTask_Tick++;
    p->uxPriority = uxPriority;

    if( TraceTask_LogCnt < TRACETASK_NUM -1 )
    {
      TraceTask_LogCnt++;
    }else{
      TraceTask_LogCnt = 0;
    }
  }
}

uint16_t tracetask_write_switch( uint16_t sw )
{
  TraceTask_LogSw = sw;
  return TraceTask_LogCnt;
}

void tracetask_printlog( uint16_t start, uint16_t stop )
{
  TraceTask_t *p;
  uint16_t  idx;

  if ((start<TRACETASK_NUM) && (stop<TRACETASK_NUM))
  {
    idx = start;
    p = &TraceTask_LogBuff[ idx ];

    while( (idx != stop) && (p->marker != 0) )
    {
      printf( "%c, %s, %d¥n", p->marker, p->task, p->tick );
      if( idx < TRACETASK_NUM -1 )
      {
        idx++;
        p++;
      }else{
        idx = 0;
        p = &TraceTask_LogBuff[ 0 ];
      }
    }
  }
}

void tracetask_printall( void )
{
  tracetask_printlog( 0, TRACETASK_NUM -1 );
}