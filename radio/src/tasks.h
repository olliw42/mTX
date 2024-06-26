/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _TASKS_H_
#define _TASKS_H_

#include "rtos.h"

// stack sizes should be in multiples of 8 for better alignment
#if defined (COLORLCD)
  #define MENUS_STACK_SIZE     4000
#else
  #define MENUS_STACK_SIZE     2000
#endif
#define MIXER_STACK_SIZE       400
#define AUDIO_STACK_SIZE       400
#define CLI_STACK_SIZE         1000  // only consumed with CLI build option
#if defined(INTERNAL_MODULE_CRSF)
#define CROSSFIRE_STACK_SIZE   64
#define SYSTEM_STACK_SIZE      504
#endif

#define MIXER_TASK_PRIO        5
#define AUDIO_TASK_PRIO        7
#define MENUS_TASK_PRIO        10
#define CLI_TASK_PRIO          10
#define CROSSFIRE_TASK_PRIO    0
#define RTOS_SYS_TASK_PRIO     10

extern RTOS_TASK_HANDLE menusTaskId;
extern RTOS_DEFINE_STACK(menusStack, MENUS_STACK_SIZE);

extern RTOS_TASK_HANDLE mixerTaskId;
extern RTOS_DEFINE_STACK(mixerStack, MIXER_STACK_SIZE);

extern RTOS_TASK_HANDLE audioTaskId;
extern RTOS_DEFINE_STACK(audioStack, AUDIO_STACK_SIZE);

extern RTOS_MUTEX_HANDLE mixerMutex;
extern RTOS_FLAG_HANDLE openTxInitCompleteFlag;

#if defined(INTERNAL_MODULE_CRSF)
extern RTOS_TASK_HANDLE crossfireTaskId;
extern RTOS_DEFINE_STACK(crossfireStack, CROSSFIRE_STACK_SIZE);

extern RTOS_TASK_HANDLE systemTaskId;
extern RTOS_DEFINE_STACK(systemStack, SYSTEM_STACK_SIZE);

void crossfireTasksStop();
void crossfireTasksCreate();
#endif
//OW
#if defined(TELEMETRY_MAVLINK)
#define MAVLINK_STACK_SIZE       400 //consumes 4x
#define MAVLINK_TASK_PRIO        8

extern RTOS_TASK_HANDLE mavlinkTaskId;
extern RTOS_DEFINE_STACK(mavlinkStack, MAVLINK_STACK_SIZE);
#endif
//OWEND

void stackPaint();
void tasksStart();

extern volatile uint16_t timeForcePowerOffPressed;
inline void resetForcePowerOffRequest()
{
  timeForcePowerOffPressed = 0;
}

#endif // _TASKS_H_
