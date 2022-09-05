/*
 * Copyright Copyright (c) 2022 Ingics Technology
 */

#ifndef _SENSOR_CTRL_EVENT_H_
#define _SENSOR_CTRL_EVENT_H_

/**
 * @file sensor_ctrl_event.h
 * @brief For sensor manager control capability
 * @{
 */

#include <app_event_manager.h>
#include <app_event_manager_profiler_tracer.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_CTRL_CMD_LIST	  \
	X(CHANGE_SAMPLING_PERIOD) \
	X(TRIGGER_SAMPLING)

enum sensor_ctrl_cmd {
#define X(name) _CONCAT(SENSOR_CTRL_CMD_, name),
        SENSOR_CTRL_CMD_LIST
#undef X

        SENSOR_CTRL_CMD_COUNT
};

struct sensor_ctrl_event {
        struct app_event_header header;

        const char *descr;
        uint8_t cmd;
        struct event_dyndata dyndata;
};

APP_EVENT_TYPE_DYNDATA_DECLARE(sensor_ctrl_event);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _SENSOR_CTRL_EVENT_H_ */
