/*
 * COPYRIGHT (C) STMicroelectronics 2015. All rights reserved.
 *
 * This software is the confidential and proprietary information of
 * STMicroelectronics ("Confidential Information").  You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered into
 * with STMicroelectronics
 *
 * Programming Golden Rule: Keep it Simple!
 *
 */

/**
 * @file   vl53l1_platform_log.c
 *
 * @brief  Code function definitions for EwokPlus25 Platform Logging Layer
 */

#include <stdio.h>    // sprintf(), vsnprintf(), printf()
#include <string.h>
#include <stdarg.h>
#include <malloc.h>

#include "vl53l1_platform_log.h"
#include "vl53l1_platform_user_config.h"
#include "stm32f4xx_hal.h"

#ifdef VL53L1_LOG_ENABLE

	// char * _trace_filename = NULL;
	// FILE *_tracefile = NULL;

	uint32_t _trace_level     = VL53L1_TRACE_LEVEL_ALL;
	uint32_t _trace_modules   = VL53L1_TRACE_MODULE_ALL;
	uint32_t _trace_functions = VL53L1_TRACE_FUNCTION_ALL;

	int8_t VL53L1_trace_config(
		char *filename,
		uint32_t modules,
		uint32_t level,
		uint32_t functions)
	{
		int8_t status = 0;

		if (filename != NULL)
		{
			printf("STM32 don't support file operation\n");
		}

		_trace_modules   = modules;
		_trace_level     = level;
		_trace_functions = functions;

		return status;
	}

	void VL53L1_trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
	{
		if ( ((level <=_trace_level) && ((module & _trace_modules) > 0))
			|| ((function & _trace_functions) > 0) )
		{
			va_list arg_list;
			char message[VL53L1_MAX_STRING_LENGTH];

			va_start(arg_list, format);
			vsnprintf(message, VL53L1_MAX_STRING_LENGTH-1, format, arg_list); /*lint !e534  ignore return*/
			va_end(arg_list);

	        printf(message);
		}  /*lint !e438  ignore issues with arg_list*/ 
	}


	uint32_t VL53L1_get_trace_functions(void)
	{
		return _trace_functions;
	}


	void VL53L1_set_trace_functions(uint32_t function)
	{
		_trace_functions = function;
	}


	uint32_t VL53L1_clock(void)
	{
		/* Returns current tick count in [ms] */
		uint32_t tick_count_ms = (uint32_t)HAL_GetTick();
		return tick_count_ms;
	}
#else

#ifdef STACK_USAGE_DISPLAY
	void  UpdateStackBot(void) {
		int a;
		stack_bottom = (stack_bottom < (char *)&a ? stack_bottom : (char *)&a);
	}
#endif

#endif // VL53L1_LOG_ENABLE
