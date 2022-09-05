/*
 * Copyright (c) 2022 Ingics Technology
 */

#include <stdio.h>
#include <caf/events/sensor_ctrl_event.h>

static const char *const sensor_ctrl_cmd_name[] = {
#define X(name) STRINGIFY(name),
	SENSOR_CTRL_CMD_LIST
#undef X
};

static void log_sensor_ctrl_event(const struct app_event_header *aeh)
{
	const struct sensor_ctrl_event *event = cast_sensor_ctrl_event(aeh);

	APP_EVENT_MANAGER_LOG(aeh, "%s %s", event->descr, sensor_ctrl_cmd_name[event->cmd]);
}

static void profile_sensor_ctrl_event(struct log_event_buf *buf,
				      const struct app_event_header *aeh)
{
        const struct sensor_ctrl_event *event = cast_sensor_ctrl_event(aeh);

        nrf_profiler_log_encode_string(buf, event->descr);
        nrf_profiler_log_encode_string(buf, sensor_ctrl_cmd_name[event->cmd]);
}

APP_EVENT_INFO_DEFINE(sensor_ctrl_event,
                ENCODE(NRF_PROFILER_ARG_STRING, NRF_PROFILER_ARG_STRING),
                ENCODE("descr", "cmd"),
                profile_sensor_ctrl_event);

APP_EVENT_TYPE_DEFINE(sensor_ctrl_event,
                log_sensor_ctrl_event,
                &sensor_ctrl_event_info,
                APP_EVENT_FLAGS_CREATE(
                        IF_ENABLED(true, (APP_EVENT_TYPE_FLAGS_INIT_LOG_ENABLE))));
