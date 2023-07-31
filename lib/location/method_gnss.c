/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <date_time.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/timeutil.h>
#include <modem/location.h>
#include <modem/lte_lc.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <nrf_errno.h>
#include "location_core.h"
#include "location_utils.h"
#if defined(CONFIG_NRF_CLOUD_AGPS)
#include <net/nrf_cloud_rest.h>
#include <net/nrf_cloud_agps.h>
#include <stdlib.h>
#endif
#if defined(CONFIG_NRF_CLOUD_PGPS)
#include <net/nrf_cloud_rest.h>
#include <net/nrf_cloud_pgps.h>
#endif
#if defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
#include <mcc_location_table.h>
#endif
LOG_MODULE_DECLARE(location, CONFIG_LOCATION_LOG_LEVEL);

#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS)
/* Verify that MQTT, REST or external service is enabled */
BUILD_ASSERT(
	IS_ENABLED(CONFIG_NRF_CLOUD_MQTT) ||
	IS_ENABLED(CONFIG_NRF_CLOUD_REST) ||
	IS_ENABLED(CONFIG_LOCATION_SERVICE_EXTERNAL),
	"CONFIG_NRF_CLOUD_MQTT, CONFIG_NRF_CLOUD_REST or "
	"CONFIG_LOCATION_SERVICE_EXTERNAL must be enabled");
#endif

/* Maximum waiting time before GNSS is started regardless of RRC or PSM state [min]. This prevents
 * Location library from getting stuck indefinitely if the application keeps LTE connection
 * constantly active.
 */
#define SLEEP_WAIT_BACKSTOP 5
#if !defined(CONFIG_NRF_CLOUD_AGPS)
/* range 10240-3456000000 ms, see AT command %XMODEMSLEEP */
#define MIN_SLEEP_DURATION_FOR_STARTING_GNSS 10240
#define AT_MDM_SLEEP_NOTIF_START "AT%%XMODEMSLEEP=1,%d,%d"
#endif
#if (defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS))
#define AGPS_REQUEST_RECV_BUF_SIZE 3500
#define AGPS_REQUEST_HTTPS_RESP_HEADER_SIZE 400
/* Minimum time between two A-GPS data requests in seconds. */
#define AGPS_REQUEST_MIN_INTERVAL (60 * 60)
/* A-GPS data expiration threshold in seconds before requesting fresh data. An 80 minute threshold
 * is used because it leaves enough time to try again after an hour if fetching of A-GPS data fails
 * once. Also, because of overlapping ephemeris validity times, fresh ephemerides are
 * needed on average every two hours with both 80 minute and shorter expiration thresholds.
 */
#define AGPS_EXPIRY_THRESHOLD (80 * 60)
/* P-GPS data expiration threshold is zero seconds, because there is no overlap between
 * predictions.
 */
#define PGPS_EXPIRY_THRESHOLD 0
/* A-GPS minimum number of expired ephemerides to request all ephemerides. */
#define AGPS_EPHE_MIN_COUNT 3
/* P-GPS minimum number of expired ephemerides to trigger injection of predictions.
 * With P-GPS, predictions are not available for all satellites, especially when
 * predictions are made further into the future, so it is natural that predictions are
 * missing for some of the satellites.
 */
#define PGPS_EPHE_MIN_COUNT 12
/* A-GPS minimum number of expired almanacs to request all almanacs. */
#define AGPS_ALM_MIN_COUNT 3
#endif

#define VISIBILITY_DETECTION_EXEC_TIME CONFIG_LOCATION_METHOD_GNSS_VISIBILITY_DETECTION_EXEC_TIME
#define VISIBILITY_DETECTION_SAT_LIMIT CONFIG_LOCATION_METHOD_GNSS_VISIBILITY_DETECTION_SAT_LIMIT

static struct k_work method_gnss_start_work;
static struct k_work method_gnss_pvt_work;
#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS) || defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
/* Flag indicating whether nrf_modem_gnss_agps_expiry_get() is supported. If the API is
 * supported, NRF_MODEM_GNSS_EVT_AGPS_REQ events are ignored.
 */
static bool agps_expiry_get_supported;
static struct k_work method_gnss_agps_req_event_handle_work;
#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS)
static struct k_work method_gnss_agps_req_work;
#endif
#endif

#if defined(CONFIG_NRF_CLOUD_AGPS)
static int64_t agps_req_timestamp;
#if !defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && \
	defined(CONFIG_NRF_CLOUD_REST) && !defined(CONFIG_NRF_CLOUD_MQTT)
static char agps_rest_data_buf[AGPS_REQUEST_RECV_BUF_SIZE];
#endif
#endif

#if defined(CONFIG_NRF_CLOUD_PGPS)
#if !defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
static struct k_work method_gnss_pgps_request_work;
#endif
static struct k_work method_gnss_inject_pgps_work;
static struct k_work method_gnss_notify_pgps_work;
static struct nrf_cloud_pgps_prediction *prediction;
static struct gps_pgps_request pgps_request;
#endif

static bool running;
static struct location_gnss_config gnss_config;
static K_SEM_DEFINE(entered_psm_mode, 0, 1);
static K_SEM_DEFINE(entered_rrc_idle, 1, 1);

#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS) || defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
static struct nrf_modem_gnss_agps_data_frame agps_request;
#endif

#if defined(CONFIG_NRF_CLOUD_PGPS)
static struct nrf_modem_gnss_agps_data_frame pgps_agps_request = {
	/* Ephe mask is initially all set, because event PGPS_EVT_AVAILABLE may be received before
	 * the assistance request from GNSS. If ephe mask would be zero, no predictions would be
	 * injected.
	 */
	.sv_mask_ephe = 0xffffffff,
	.sv_mask_alm = 0x00000000,
	/* Also inject current time by default. */
	.data_flags = NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST
};
#endif

#if defined(CONFIG_NRF_CLOUD_REST) && !defined(CONFIG_NRF_CLOUD_MQTT)
#if (defined(CONFIG_NRF_CLOUD_AGPS) && !defined(CONFIG_LOCATION_SERVICE_EXTERNAL)) || \
	(defined(CONFIG_NRF_CLOUD_PGPS) && !defined(CONFIG_LOCATION_SERVICE_EXTERNAL))
static char rest_api_recv_buf[CONFIG_NRF_CLOUD_REST_FRAGMENT_SIZE +
			      AGPS_REQUEST_HTTPS_RESP_HEADER_SIZE];
#endif
#endif

#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_NRF_CLOUD_PGPS)
static struct k_work method_gnss_pgps_ext_work;
static void method_gnss_pgps_ext_work_fn(struct k_work *item);
#endif

/* Count of consecutive NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME flags in PVT data. Used for
 * triggering GNSS priority mode if enabled.
 */
static int insuf_timewin_count;
static int fixes_remaining;

#if defined(CONFIG_LOCATION_DATA_DETAILS)
static struct location_data_details_gnss location_data_details_gnss;
#endif

#if defined(CONFIG_NRF_CLOUD_PGPS)
static void method_gnss_inject_pgps_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	int err;

	LOG_DBG("Sending prediction to modem (ephe: 0x%08x)...", pgps_agps_request.sv_mask_ephe);

	err = nrf_cloud_pgps_inject(prediction, &pgps_agps_request);
	if (err) {
		LOG_ERR("Unable to send prediction to modem: %d", err);
	}
}

void method_gnss_pgps_handler(struct nrf_cloud_pgps_event *event)
{
	LOG_DBG("P-GPS event type: %d", event->type);

	if (event->type == PGPS_EVT_READY) {
		/* P-GPS has finished downloading predictions; request the current prediction. */
		k_work_submit_to_queue(location_core_work_queue_get(),
				       &method_gnss_notify_pgps_work);
	} else if (event->type == PGPS_EVT_AVAILABLE) {
		/* Inject the specified prediction into the modem. */
		prediction = event->prediction;
		k_work_submit_to_queue(location_core_work_queue_get(),
				       &method_gnss_inject_pgps_work);
	} else if (event->type == PGPS_EVT_REQUEST) {
		memcpy(&pgps_request, event->request, sizeof(pgps_request));
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
		k_work_submit_to_queue(location_core_work_queue_get(), &method_gnss_pgps_ext_work);
#else
		k_work_submit_to_queue(location_core_work_queue_get(),
				       &method_gnss_pgps_request_work);
#endif
	}
}

static void method_gnss_notify_pgps_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	int err = nrf_cloud_pgps_notify_prediction();

	if (err) {
		LOG_ERR("Error requesting notification of prediction availability: %d", err);
	}
}
#endif

void method_gnss_lte_ind_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_MODEM_SLEEP_ENTER:
		if (evt->modem_sleep.type == LTE_LC_MODEM_SLEEP_PSM) {
			/* Allow GNSS operation once LTE modem is in power saving mode. */
			k_sem_give(&entered_psm_mode);
		}
		break;
	case LTE_LC_EVT_MODEM_SLEEP_EXIT:
		/* Prevent GNSS from starting while LTE is active. */
		k_sem_reset(&entered_psm_mode);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		/* If PSM becomes disabled e.g. due to network change, allow GNSS to be started
		 * in case there was a pending position request waiting for the sleep to start. If
		 * PSM becomes enabled, block GNSS until the modem enters PSM by taking the
		 * semaphore.
		 */
		if (evt->psm_cfg.active_time == -1) {
			k_sem_give(&entered_psm_mode);
		} else if (evt->psm_cfg.active_time > 0) {
			k_sem_take(&entered_psm_mode, K_NO_WAIT);
		}
		break;
	case LTE_LC_EVT_RRC_UPDATE:
		if (evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED) {
			/* Prevent GNSS from starting while RRC is in connected mode. */
			k_sem_reset(&entered_rrc_idle);
		} else if (evt->rrc_mode == LTE_LC_RRC_MODE_IDLE) {
			/* Allow GNSS operation once RRC is in idle mode. */
			k_sem_give(&entered_rrc_idle);
		}
		break;
	default:
		break;
	}
}

#if defined(CONFIG_NRF_CLOUD_AGPS) && !defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
#if defined(CONFIG_NRF_CLOUD_MQTT)
static void method_gnss_nrf_cloud_agps_request(void)
{
	int err = nrf_cloud_agps_request(&agps_request);

	if (err) {
		LOG_ERR("nRF Cloud A-GPS request failed, error: %d", err);
		return;
	}

	LOG_DBG("A-GPS data requested");
}

#elif defined(CONFIG_NRF_CLOUD_REST)
static void method_gnss_nrf_cloud_agps_request(void)
{
	const char *jwt_buf;
	int err;
	struct nrf_cloud_rest_context rest_ctx = {
		.connect_socket = -1,
		.keep_alive = false,
		.timeout_ms = NRF_CLOUD_REST_TIMEOUT_NONE,
		.rx_buf = rest_api_recv_buf,
		.rx_buf_len = sizeof(rest_api_recv_buf),
		.fragment_size = 0
	};

	jwt_buf = location_utils_nrf_cloud_jwt_generate();
	if (jwt_buf == NULL) {
		return;
	}
	rest_ctx.auth = (char *)jwt_buf;

	struct nrf_cloud_rest_agps_request request = {
		NRF_CLOUD_REST_AGPS_REQ_CUSTOM,
		&agps_request,
		NULL,
		false,
		0
	};

	struct lte_lc_cells_info net_info = {0};
	struct location_utils_modem_params_info modem_params = { 0 };

	/* Get network info for the A-GPS location request. */
	err = location_utils_modem_params_read(&modem_params);

	if (err < 0) {
		LOG_WRN("Requesting A-GPS data without location assistance");
	} else {
		net_info.current_cell.id = modem_params.cell_id;
		net_info.current_cell.tac = modem_params.tac;
		net_info.current_cell.mcc = modem_params.mcc;
		net_info.current_cell.mnc = modem_params.mnc;
		net_info.current_cell.phys_cell_id = modem_params.phys_cell_id;
		request.net_info = &net_info;
	}

	struct nrf_cloud_rest_agps_result result = {
		agps_rest_data_buf,
		sizeof(agps_rest_data_buf),
		0};

	err = nrf_cloud_rest_agps_data_get(&rest_ctx, &request, &result);
	if (err) {
		LOG_ERR("nRF Cloud A-GPS request failed, error: %d", err);
		return;
	}

	LOG_DBG("A-GPS data requested");

	err = nrf_cloud_agps_process(result.buf, result.agps_sz);
	if (err) {
		LOG_ERR("A-GPS data processing failed, error: %d", err);
		return;
	}

	LOG_DBG("A-GPS data processed");

#if defined(CONFIG_NRF_CLOUD_PGPS)
	err = nrf_cloud_pgps_notify_prediction();
	if (err) {
		LOG_ERR("Error requesting notification of prediction availability: %d", err);
	}
#endif
}
#endif /* #elif defined(CONFIG_NRF_CLOUD_REST) */
#endif /* defined(CONFIG_NRF_CLOUD_AGPS) && !defined(CONFIG_LOCATION_SERVICE_EXTERNAL) */

#if defined(CONFIG_NRF_CLOUD_PGPS) && !defined(CONFIG_NRF_CLOUD_MQTT) && \
	!defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
static void method_gnss_pgps_request_work_fn(struct k_work *item)
{
	const char *jwt_buf;
	int err;
	struct nrf_cloud_rest_context rest_ctx = {
		.connect_socket = -1,
		.keep_alive = false,
		.timeout_ms = NRF_CLOUD_REST_TIMEOUT_NONE,
		.rx_buf = rest_api_recv_buf,
		.rx_buf_len = sizeof(rest_api_recv_buf),
		.fragment_size = 0
	};

	jwt_buf = location_utils_nrf_cloud_jwt_generate();
	if (jwt_buf == NULL) {
		return;
	}
	rest_ctx.auth = (char *)jwt_buf;

	struct nrf_cloud_rest_pgps_request request = {
		.pgps_req = &pgps_request
	};

	err = nrf_cloud_rest_pgps_data_get(&rest_ctx, &request);
	if (err) {
		nrf_cloud_pgps_request_reset();
		LOG_ERR("nRF Cloud P-GPS request failed, error: %d", err);
		return;
	}

	LOG_DBG("P-GPS data requested");

	err = nrf_cloud_pgps_process(rest_ctx.response, rest_ctx.response_len);
	if (err) {
		nrf_cloud_pgps_request_reset();
		LOG_ERR("P-GPS data processing failed, error: %d", err);
		return;
	}

	LOG_DBG("P-GPS data processed");

	err = nrf_cloud_pgps_notify_prediction();
	if (err) {
		LOG_ERR("GNSS: Failed to request current prediction, error: %d", err);
	} else {
		LOG_DBG("P-GPS prediction requested");
	}
}
#endif

#if defined(CONFIG_NRF_CLOUD_AGPS)
bool method_gnss_agps_required(struct nrf_modem_gnss_agps_data_frame *request)
{
	int32_t time_since_agps_req;

	/* Check if A-GPS data is needed. */
	if (request->sv_mask_ephe == 0 && request->sv_mask_alm == 0 && request->data_flags == 0) {
		LOG_DBG("No A-GPS data types requested");
		return false;
	}

	/* A-GPS data is needed, check if enough time has passed since the last A-GPS data
	 * request.
	 */
	if (agps_req_timestamp != 0) {
		time_since_agps_req = k_uptime_get() - agps_req_timestamp;
		if (time_since_agps_req < (AGPS_REQUEST_MIN_INTERVAL * MSEC_PER_SEC)) {
			LOG_DBG("Skipping A-GPS request, time since last request: %d s",
				time_since_agps_req / 1000);
			return false;
		}
	}

	return true;
}
#endif

#if defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)

/* (6.1.1980 UTC - 1.1.1970 UTC) */
#define GPS_TO_UNIX_UTC_OFFSET_SECONDS	(315964800UL)
/* UTC/GPS time offset as of 1st of January 2017. */
#define GPS_TO_UTC_LEAP_SECONDS		(18UL)
#define SEC_PER_MIN			(60UL)
#define MIN_PER_HOUR			(60UL)
#define SEC_PER_HOUR			(MIN_PER_HOUR * SEC_PER_MIN)
#define HOURS_PER_DAY			(24UL)
#define SEC_PER_DAY			(HOURS_PER_DAY * SEC_PER_HOUR)
#define DAYS_PER_WEEK			(7UL)
#define PLMN_STR_MAX_LEN		8 /* MCC + MNC + quotes */

static int64_t utc_to_gps_sec(const int64_t utc_sec)
{
	return (utc_sec - GPS_TO_UNIX_UTC_OFFSET_SECONDS) + GPS_TO_UTC_LEAP_SECONDS;
}

static void gps_sec_to_day_time(int64_t gps_sec,
				uint16_t *gps_day,
				uint32_t *gps_time_of_day)
{
	*gps_day = (uint16_t)(gps_sec / SEC_PER_DAY);
	*gps_time_of_day = (uint32_t)(gps_sec % SEC_PER_DAY);
}

static void time_inject(void)
{
	int ret;
	struct tm date_time;
	int64_t utc_sec;
	int64_t gps_sec;
	struct nrf_modem_gnss_agps_data_system_time_and_sv_tow gps_time = { 0 };

	/* Read current UTC time from the modem. */
	ret = nrf_modem_at_scanf("AT+CCLK?",
		"+CCLK: \"%u/%u/%u,%u:%u:%u",
		&date_time.tm_year,
		&date_time.tm_mon,
		&date_time.tm_mday,
		&date_time.tm_hour,
		&date_time.tm_min,
		&date_time.tm_sec
	);
	if (ret != 6) {
		LOG_WRN("Couldn't read current time from modem, time assistance unavailable");
		return;
	}

	/* Convert to struct tm format. */
	date_time.tm_year = date_time.tm_year + 2000 - 1900; /* years since 1900 */
	date_time.tm_mon--; /* months since January */

	/* Convert time to seconds since Unix time epoch (1.1.1970). */
	utc_sec = timeutil_timegm64(&date_time);
	/* Convert time to seconds since GPS time epoch (6.1.1980). */
	gps_sec = utc_to_gps_sec(utc_sec);

	gps_sec_to_day_time(gps_sec, &gps_time.date_day, &gps_time.time_full_s);

	ret = nrf_modem_gnss_agps_write(&gps_time, sizeof(gps_time),
					NRF_MODEM_GNSS_AGPS_GPS_SYSTEM_CLOCK_AND_TOWS);
	if (ret != 0) {
		LOG_ERR("Failed to inject time, error %d", ret);
		return;
	}

	LOG_INF("Injected time (GPS day %u, GPS time of day %u)",
		gps_time.date_day, gps_time.time_full_s);
}

static void location_inject(void)
{
	int err;
	char plmn_str[PLMN_STR_MAX_LEN + 1];
	uint16_t mcc;
	const struct mcc_table *mcc_info;
	struct nrf_modem_gnss_agps_data_location location = { 0 };

	/* Read PLMN string from modem to get the MCC. */
	err = nrf_modem_at_scanf(
		"AT%XMONITOR",
		"%%XMONITOR: "
		"%*d"                                  /* <reg_status>: ignored */
		",%*[^,]"                              /* <full_name>: ignored */
		",%*[^,]"                              /* <short_name>: ignored */
		",%"STRINGIFY(PLMN_STR_MAX_LEN)"[^,]", /* <plmn> */
		plmn_str);
	if (err != 1) {
		LOG_WRN("Couldn't read PLMN from modem, location assistance unavailable");
		return;
	}

	/* NULL terminate MCC and read it. */
	plmn_str[4] = '\0';
	mcc = strtol(plmn_str + 1, NULL, 10);

	mcc_info = mcc_lookup(mcc);
	if (mcc_info == NULL) {
		LOG_WRN("No location found for MCC %u", mcc);
		return;
	}

	location.latitude = lat_convert(mcc_info->lat);
	location.longitude = lon_convert(mcc_info->lon);
	location.unc_semimajor = mcc_info->unc_semimajor;
	location.unc_semiminor = mcc_info->unc_semiminor;
	location.orientation_major = mcc_info->orientation;
	location.confidence = mcc_info->confidence;
	location.unc_altitude = 255; /* altitude not used */

	err = nrf_modem_gnss_agps_write(&location, sizeof(location), NRF_MODEM_GNSS_AGPS_LOCATION);
	if (err) {
		LOG_ERR("Failed to inject location for MCC %u, error %d", mcc, err);
		return;
	}

	LOG_INF("Injected location for MCC %u", mcc);
}
#endif


#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS) || defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
/* Triggers A-GPS data request and/or injection of P-GPS predictions.
 *
 * Before this function is called, the assistance data need from GNSS must be stored into
 * 'agps_request'.
 *
 * There are three possible assistance configurations with different behavior:
 *
 * A-GPS only:
 * Ephemerides, almanacs and additional assistance data is handled by A-GPS.
 * If any additional assistance data (UTC, Klobuchar, GPS time, integrity or position) is needed,
 * all additional assistance is requested at the same time.
 *
 * A-GPS and P-GPS:
 * Ephemerides are handled by P-GPS.
 * Almanacs are handled by A-GPS, but only if the downloaded P-GPS prediction set is longer than
 * one week.
 * Additional assistance data is handled by A-GPS.
 * If any additional assistance data (UTC, Klobuchar, GPS time, integrity or position) is needed,
 * all additional assistance is requested at the same time.
 *
 * P-GPS only:
 * Ephemerides are handled by P-GPS.
 * Almanacs are not used.
 * GPS time and position assistance are handled by P-GPS.
 *
 * The frequency of A-GPS data requests is limited to avoid requesting data repeatedly, for
 * example in case the server is down.
 *
 * Almanacs are not used with P-GPS prediction sets up to one week in length, because GNSS always
 * has valid (and more accurate) ephemerides available. With longer prediction sets, the number of
 * satellite ephemerides in the later prediction periods decreases due to accumulated errors,
 * so having almanacs may be beneficial.
 */
static void method_gnss_assistance_request(void)
{
#if defined(CONFIG_NRF_CLOUD_PGPS)
	/* Ephemerides come from P-GPS. */
	pgps_agps_request.sv_mask_ephe = agps_request.sv_mask_ephe;
	pgps_agps_request.data_flags = agps_request.data_flags;
	agps_request.sv_mask_ephe = 0;
	if (CONFIG_NRF_CLOUD_PGPS_NUM_PREDICTIONS <= 42) {
		/* Almanacs not needed in this configuration. */
		agps_request.sv_mask_alm = 0;
	}

	if (IS_ENABLED(CONFIG_NRF_CLOUD_AGPS)) {
		/* Time and position come from A-GPS. */
		pgps_agps_request.data_flags = 0;
	}

	LOG_DBG("P-GPS request sv_mask_ephe: 0x%08x, data_flags 0x%02x",
		pgps_agps_request.sv_mask_ephe, pgps_agps_request.data_flags);
#endif /* CONFIG_NRF_CLOUD_PGPS */

#if defined(CONFIG_NRF_CLOUD_AGPS)
	if (agps_request.data_flags != 0) {
		/* With A-GPS, if any of the flags in the data_flags field is set, it is
		 * feasible to request everything at the same time, because of the small amount
		 * of data.
		 */
		agps_request.data_flags =
			NRF_MODEM_GNSS_AGPS_GPS_UTC_REQUEST |
			NRF_MODEM_GNSS_AGPS_KLOBUCHAR_REQUEST |
			NRF_MODEM_GNSS_AGPS_NEQUICK_REQUEST |
			NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST |
			NRF_MODEM_GNSS_AGPS_INTEGRITY_REQUEST |
			NRF_MODEM_GNSS_AGPS_POSITION_REQUEST;
	}

	LOG_DBG("A-GPS request sv_mask_ephe: 0x%08x, sv_mask_alm: 0x%08x, data_flags: 0x%02x",
		agps_request.sv_mask_ephe, agps_request.sv_mask_alm, agps_request.data_flags);

	/* Check the request. If no A-GPS data types except ephemeris or almanac are requested,
	 * jump to P-GPS (if enabled).
	 */
	if (method_gnss_agps_required(&agps_request)) {
		enum lte_lc_nw_reg_status reg_status = LTE_LC_NW_REG_NOT_REGISTERED;

		lte_lc_nw_reg_status_get(&reg_status);
		if (reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
		    reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING) {
			/* Only store the timestamp when LTE is connected, otherwise it is not
			 * very likely that the A-GPS request succeeds.
			 */
			agps_req_timestamp = k_uptime_get();
		}
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
		location_core_event_cb_agps_request(&agps_request);
#else
		method_gnss_nrf_cloud_agps_request();
#endif
	}
#endif /* CONFIG_NRF_CLOUD_AGPS */

#if defined(CONFIG_NRF_CLOUD_PGPS)
	if (pgps_agps_request.sv_mask_ephe != 0) {
		/* When A-GPS is used, the nRF Cloud library also calls this function after
		 * A-GPS data has been processed. However, the call happens too late to trigger
		 * the initial P-GPS data download at the correct stage.
		 */
		int err = nrf_cloud_pgps_notify_prediction();

		if (err) {
			LOG_ERR("Error requesting notification of prediction availability: %d",
				err);
		}
	}
#endif /* CONFIG_NRF_CLOUD_PGPS */

#if defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
	if (!(agps_request.data_flags & NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST) &&
	    !(agps_request.data_flags & NRF_MODEM_GNSS_AGPS_POSITION_REQUEST)) {
		LOG_INF("Ignoring assistance request because no GPS time or position is requested");
		return;
	    }

	if (agps_request.data_flags & NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST) {
		time_inject();
	}

	if (agps_request.data_flags & NRF_MODEM_GNSS_AGPS_POSITION_REQUEST) {
		location_inject();
	}
#endif /* LOCATION_GNSS_MINIMAL_ASSISTANCE */
}

static void method_gnss_agps_req_event_handle_work_fn(struct k_work *item)
{
	int err = nrf_modem_gnss_read(&agps_request,
				      sizeof(agps_request),
				      NRF_MODEM_GNSS_DATA_AGPS_REQ);

	if (err) {
		LOG_WRN("Reading A-GPS req data from GNSS failed, error: %d", err);
		return;
	}

	method_gnss_assistance_request();
}
#endif /* defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS) */

void method_gnss_event_handler(int event)
{
	switch (event) {
	case NRF_MODEM_GNSS_EVT_PVT:
		k_work_submit_to_queue(location_core_work_queue_get(), &method_gnss_pvt_work);
		break;

	case NRF_MODEM_GNSS_EVT_AGPS_REQ:
#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS) || defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
		if (!agps_expiry_get_supported) {
			k_work_submit_to_queue(
				location_core_work_queue_get(),
				&method_gnss_agps_req_event_handle_work);
		}
#endif
		break;
	}
}

int method_gnss_cancel(void)
{
	int err = nrf_modem_gnss_stop();
	int sleeping;
	int rrc_idling;

	if ((err != 0) && (err != -NRF_EPERM)) {
		LOG_ERR("Failed to stop GNSS");
	}

	running = false;

	/* Cancel any work that has not been started yet */
	(void)k_work_cancel(&method_gnss_start_work);

	/* If we are currently not in PSM, i.e., LTE is running, reset the semaphore to unblock
	 * method_gnss_positioning_work_fn() and allow the ongoing location request to terminate.
	 * Otherwise, don't reset the semaphore in order not to lose information about the current
	 * sleep state.
	 */
	sleeping = k_sem_count_get(&entered_psm_mode);
	if (!sleeping) {
		k_sem_reset(&entered_psm_mode);
	}

	/* If we are currently in RRC connected mode, reset the semaphore to unblock
	 * method_gnss_positioning_work_fn() and allow the ongoing location request to terminate.
	 * Otherwise, don't reset the semaphore in order not to lose information about the current
	 * RRC state.
	 */
	rrc_idling = k_sem_count_get(&entered_rrc_idle);
	if (!rrc_idling) {
		k_sem_reset(&entered_rrc_idle);
	}

#if defined(CONFIG_NRF_CLOUD_PGPS)
	int ret;

	ret = nrf_cloud_pgps_preemptive_updates();
	if (ret) {
		LOG_ERR("Error requesting P-GPS pre-emptive updates: %d", ret);
	}
#endif /* CONFIG_NRF_CLOUD_PGPS */

	return err;
}

int method_gnss_timeout(void)
{
	if (insuf_timewin_count == -1 && gnss_config.priority_mode == false) {
		LOG_WRN("GNSS timed out possibly due to too short GNSS time windows");
	}

	return method_gnss_cancel();
}

#if !defined(CONFIG_NRF_CLOUD_AGPS)
static bool method_gnss_psm_enabled(void)
{
	int ret = 0;
	int tau;
	int active_time;

	ret = lte_lc_psm_get(&tau, &active_time);
	if (ret < 0) {
		LOG_ERR("Cannot get PSM config: %d. Starting GNSS right away.", ret);
		return false;
	}

	LOG_DBG("LTE active time: %d seconds", active_time);

	if (active_time >= 0) {
		return true;
	}

	return false;
}

static bool method_gnss_entered_psm(void)
{
	LOG_DBG("%s", k_sem_count_get(&entered_psm_mode) == 0 ?
		"Waiting for LTE to enter PSM..." :
		"LTE already in PSM");

	/* Wait for the PSM to start. If semaphore is reset during the waiting
	 * period, the position request was canceled.
	 */
	if (k_sem_take(&entered_psm_mode, K_MINUTES(SLEEP_WAIT_BACKSTOP))
		== -EAGAIN) {
		if (!running) { /* Location request was cancelled */
			return false;
		}
		/* We're still running, i.e., the wait for PSM timed out */
		LOG_WRN("PSM is configured, but modem did not enter PSM "
			"in %d minutes. Starting GNSS anyway.",
			SLEEP_WAIT_BACKSTOP);
	}
	k_sem_give(&entered_psm_mode);
	return true;
}

static void method_gnss_modem_sleep_notif_subscribe(uint32_t threshold_ms)
{
	int err;

	err = nrf_modem_at_printf(AT_MDM_SLEEP_NOTIF_START, 0, threshold_ms);
	if (err) {
		LOG_ERR("Cannot subscribe to modem sleep notifications, err %d", err);
	} else {
		LOG_DBG("Subscribed to modem sleep notifications");
	}
}
#endif /* !CONFIG_NRF_CLOUD_AGPS */

static bool method_gnss_allowed_to_start(void)
{
	enum lte_lc_system_mode mode;

	if (lte_lc_system_mode_get(&mode, NULL) != 0) {
		/* Failed to get system mode, try to start GNSS anyway */
		return true;
	}

	/* Don't care about LTE state if we are in GNSS only mode */
	if (mode == LTE_LC_SYSTEM_MODE_GPS) {
		return true;
	}

	LOG_DBG("%s", k_sem_count_get(&entered_rrc_idle) == 0 ?
		"Waiting for the RRC connection release..." :
		"RRC already in idle mode");

	/* If semaphore is reset during the waiting period, the position request was canceled.*/
	if (k_sem_take(&entered_rrc_idle, K_MINUTES(SLEEP_WAIT_BACKSTOP)) == -EAGAIN) {
		if (!running) { /* Location request was cancelled */
			return false;
		}
		/* We're still running, i.e., the wait for RRC idle timed out */
		LOG_WRN("RRC connection was not released in %d minutes. Starting GNSS anyway.",
			SLEEP_WAIT_BACKSTOP);
		return true;
	}
	k_sem_give(&entered_rrc_idle);

#if !defined(CONFIG_NRF_CLOUD_AGPS)
	/* If A-GPS is used, a GNSS fix can be obtained fast even in RRC idle mode (without PSM).
	 * Without A-GPS, it's practical to wait for the modem to sleep before attempting a fix.
	 */
	if (method_gnss_psm_enabled()) {
		return method_gnss_entered_psm();
	}
#endif
	return true;
}

static uint8_t method_gnss_tracked_satellites(const struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked = 0;

	for (uint32_t i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
		if (pvt_data->sv[i].sv == 0) {
			break;
		}

		tracked++;
	}

	return tracked;
}

static uint8_t method_gnss_tracked_satellites_nonzero_cn0(
		const struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked = 0;

	for (uint32_t i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
		if ((pvt_data->sv[i].sv == 0) || (pvt_data->sv[i].cn0 == 0)) {
			break;
		}

		tracked++;
	}

	return tracked;
}

static void method_gnss_print_pvt(const struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	LOG_DBG("Tracked satellites: %d, fix valid: %s, insuf. time window: %s",
		method_gnss_tracked_satellites(pvt_data),
		pvt_data->flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID ? "true" : "false",
		pvt_data->flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME ?
		"true" : "false");

	/* Print details for each satellite */
	for (uint32_t i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
		if (pvt_data->sv[i].sv == 0) {
			break;
		}

		const struct nrf_modem_gnss_sv *sv_data = &pvt_data->sv[i];

		LOG_DBG("PRN: %3d, C/N0: %4.1f, in fix: %d, unhealthy: %d",
			sv_data->sv,
			sv_data->cn0 / 10.0,
			sv_data->flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX ? 1 : 0,
			sv_data->flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY ? 1 : 0);
	}
}

static void method_gnss_pvt_work_fn(struct k_work *item)
{
	struct nrf_modem_gnss_pvt_data_frame pvt_data;
	static struct location_data location_result = { 0 };
	uint8_t satellites_tracked_nonzero_cn0;

	if (!running) {
		/* Cancel has already been called, so ignore the notification. */
		return;
	}

	if (nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT) != 0) {
		LOG_ERR("Failed to read PVT data from GNSS");
		location_core_event_cb_error();
		return;
	}

	/* Only satellites with a reasonable C/N0 count towards the obstructed visibility limit */
	satellites_tracked_nonzero_cn0 = method_gnss_tracked_satellites_nonzero_cn0(&pvt_data);

	method_gnss_print_pvt(&pvt_data);

#if defined(CONFIG_LOCATION_DATA_DETAILS)
	location_data_details_gnss.pvt_data = pvt_data;
	location_data_details_gnss.satellites_tracked = method_gnss_tracked_satellites(&pvt_data);
#endif

	/* Store fix data only if we get a valid fix. Thus, the last valid data is always kept
	 * in memory and it is not overwritten in case we get an invalid fix.
	 */
	if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
		fixes_remaining--;

		location_result.latitude = pvt_data.latitude;
		location_result.longitude = pvt_data.longitude;
		location_result.accuracy = pvt_data.accuracy;
		location_result.datetime.valid = true;
		location_result.datetime.year = pvt_data.datetime.year;
		location_result.datetime.month = pvt_data.datetime.month;
		location_result.datetime.day = pvt_data.datetime.day;
		location_result.datetime.hour = pvt_data.datetime.hour;
		location_result.datetime.minute = pvt_data.datetime.minute;
		location_result.datetime.second = pvt_data.datetime.seconds;
		location_result.datetime.ms = pvt_data.datetime.ms;

		if (fixes_remaining <= 0) {
			/* We are done, stop GNSS and publish the fix. */
			method_gnss_cancel();
			location_core_event_cb(&location_result);
		}
	} else if (gnss_config.visibility_detection) {
		if (pvt_data.execution_time >= VISIBILITY_DETECTION_EXEC_TIME &&
		    pvt_data.execution_time < (VISIBILITY_DETECTION_EXEC_TIME + MSEC_PER_SEC) &&
		    satellites_tracked_nonzero_cn0 < VISIBILITY_DETECTION_SAT_LIMIT) {
			LOG_DBG("GNSS visibility obstructed, canceling");
			method_gnss_cancel();
			location_core_event_cb_error();
		}
	}

	/* Trigger GNSS priority mode if GNSS indicates that it is not getting long enough time
	 * windows for 5 consecutive epochs. If the priority mode option is not enabled, a trace
	 * is output in case of a timeout to warn that GNSS may be getting too short time windows
	 * to get a fix.
	 */
	if ((pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) &&
	    (insuf_timewin_count >= 0)) {
		insuf_timewin_count++;

		if (insuf_timewin_count == 5) {
			if (gnss_config.priority_mode) {
				LOG_DBG("GNSS is not getting long enough time windows. "
					"Triggering GNSS priority mode.");
				int err = nrf_modem_gnss_prio_mode_enable();

				if (err) {
					LOG_ERR("Unable to trigger GNSS priority mode.");
				}
			}

			/* Special value -1 indicates that the condition has been already triggered.
			 * Allow triggering priority mode only once in order to not block LTE
			 * operation excessively.
			 */
			insuf_timewin_count = -1;
		}
	} else if (insuf_timewin_count > 0) {
		/* GNSS must indicate that it is not getting long enough time windows for 5
		 * consecutive epochs to trigger priority mode.
		 */
		insuf_timewin_count = 0;
	}
}

#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL) && defined(CONFIG_NRF_CLOUD_PGPS)
static void method_gnss_pgps_ext_work_fn(struct k_work *item)
{
	location_core_event_cb_pgps_request(&pgps_request);
}
#endif

static void method_gnss_positioning_work_fn(struct k_work *work)
{
	int err = 0;

	if (!method_gnss_allowed_to_start()) {
		/* Location request was cancelled while waiting for RRC idle or PSM. Do nothing. */
		return;
	}

	/* Configure GNSS to continuous tracking mode */
	err = nrf_modem_gnss_fix_interval_set(1);
	if (err == -EINVAL) {
		LOG_WRN("First nrf_modem_gnss API function failed. It could be that "
			"modem's system or functional mode doesn't allow GNSS usage.");
	}

#if defined(CONFIG_NRF_CLOUD_AGPS_ELEVATION_MASK)
	err |= nrf_modem_gnss_elevation_threshold_set(CONFIG_NRF_CLOUD_AGPS_ELEVATION_MASK);
#endif

	insuf_timewin_count = 0;

	/* By default we take the first fix. */
	fixes_remaining = 1;

	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	switch (gnss_config.accuracy) {
	case LOCATION_ACCURACY_LOW:
		use_case |= NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
		break;

	case LOCATION_ACCURACY_NORMAL:
		break;

	case LOCATION_ACCURACY_HIGH:
		/* In high accuracy mode, use the configured fix count. */
		fixes_remaining = gnss_config.num_consecutive_fixes;
		break;
	}

	err |= nrf_modem_gnss_use_case_set(use_case);

	if (err) {
		LOG_ERR("Failed to configure GNSS");
		location_core_event_cb_error();
		running = false;
		return;
	}

	err = nrf_modem_gnss_start();
	if (err) {
		LOG_ERR("Failed to start GNSS");
		location_core_event_cb_error();
		running = false;
		return;
	}

	location_core_timer_start(gnss_config.timeout);
}

#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS)
/* Processes A-GPS expiry data from nrf_modem_gnss_agps_expiry_get() function.
 *
 * This function is only used with modem firmware v1.3.2 or later. With older modem firmware
 * versions, the logic in this function is handled by GNSS.
 *
 * GNSS gives the expiration times in seconds, so logic is needed to process the expiration times
 * and to determine which data (if any) should be requested.
 *
 * Ephemerides:
 * Each satellite has its own ephemeris expiration time. The code goes though all satellites and
 * checks whether its ephemeris has expired or is going to expire. If there are enough satellites
 * with expired ephemerides, ephemerides for all satellites are requested. The time when an
 * ephemeris is considered expired and the threshold for the number of expired satellites is
 * different with A-GPS and P-GPS. The used constants are described in the beginning of the file.
 *
 * Almanacs:
 * Each satellite has its own almanac expiration time. The code goes though all satellites and
 * checks whether its almanac has expired or is going to expire. If there are enough satellites
 * with expired almanacs, almanacs for all satellites are requested.
 *
 * Other assistance data:
 * For other assistance data, the same A-GPS expiration threshold is used if expiry time is
 * provided by GNSS. For some data, there is only a flag indicating whether the data is needed or
 * not.
 */
static bool method_gnss_agps_expiry_process(const struct nrf_modem_gnss_agps_expiry *agps_expiry)
{
	uint16_t ephe_expiry_threshold;
	uint8_t expired_ephes_min_count;
	uint8_t expired_ephes = 0;
	uint8_t expired_alms = 0;

	memset(&agps_request, 0, sizeof(agps_request));

	if (IS_ENABLED(CONFIG_NRF_CLOUD_PGPS)) {
		ephe_expiry_threshold = PGPS_EXPIRY_THRESHOLD;
	} else {
		ephe_expiry_threshold = AGPS_EXPIRY_THRESHOLD;
	}

	for (int i = 0; i < NRF_MODEM_GNSS_NUM_GPS_SATELLITES; i++) {
		if (agps_expiry->ephe_expiry[i] <= ephe_expiry_threshold) {
			expired_ephes++;
		}

		if (agps_expiry->alm_expiry[i] <= AGPS_EXPIRY_THRESHOLD) {
			expired_alms++;
		}
	}

	if (IS_ENABLED(CONFIG_NRF_CLOUD_PGPS)) {
		expired_ephes_min_count = PGPS_EPHE_MIN_COUNT;
	} else {
		expired_ephes_min_count = AGPS_EPHE_MIN_COUNT;
	}

	if (expired_ephes >= expired_ephes_min_count) {
		agps_request.sv_mask_ephe = 0xffffffff;
	}

	if (expired_alms >= AGPS_ALM_MIN_COUNT) {
		agps_request.sv_mask_alm = 0xffffffff;
	}

	if (agps_expiry->utc_expiry <= AGPS_EXPIRY_THRESHOLD) {
		agps_request.data_flags |= NRF_MODEM_GNSS_AGPS_GPS_UTC_REQUEST;
	}

	if (agps_expiry->klob_expiry <= AGPS_EXPIRY_THRESHOLD) {
		agps_request.data_flags |= NRF_MODEM_GNSS_AGPS_KLOBUCHAR_REQUEST;
	}

	if (agps_expiry->neq_expiry <= AGPS_EXPIRY_THRESHOLD) {
		agps_request.data_flags |= NRF_MODEM_GNSS_AGPS_NEQUICK_REQUEST;
	}

	if (agps_expiry->data_flags & NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST) {
		agps_request.data_flags |= NRF_MODEM_GNSS_AGPS_SYS_TIME_AND_SV_TOW_REQUEST;
	}

	if (agps_expiry->integrity_expiry <= AGPS_EXPIRY_THRESHOLD) {
		agps_request.data_flags |= NRF_MODEM_GNSS_AGPS_INTEGRITY_REQUEST;
	}

	/* Position is reported as being valid for 2h. The uncertainty increases with time,
	 * but in practice the position remains usable for a longer time. Because of this no
	 * margin is used when checking if position assistance is needed.
	 *
	 * Position is only requested when also some other assistance data is needed.
	 */
	if (agps_expiry->position_expiry == 0 &&
	    (agps_request.sv_mask_ephe != 0 ||
	     agps_request.sv_mask_alm != 0 ||
	     agps_request.data_flags != 0)) {
		agps_request.data_flags |= NRF_MODEM_GNSS_AGPS_POSITION_REQUEST;
	}

	LOG_DBG("Expired ephemerides: %d, almanacs: %d", expired_ephes, expired_alms);

	LOG_DBG("Assistance request sv_mask_ephe: 0x%08x, sv_mask_alm: 0x%08x, data_flags: 0x%02x",
		agps_request.sv_mask_ephe, agps_request.sv_mask_alm, agps_request.data_flags);

	return agps_request.sv_mask_ephe != 0x0 ||
	       agps_request.sv_mask_alm != 0x0 ||
	       agps_request.data_flags != 0x0;
}

/* Queries assistance data need from GNSS.
 *
 * To maintain backward compatibility with older modem firmware versions, two different methods are
 * supported. If the nrf_modem_gnss_agps_expiry_get() API is not supported (pre-v1.3.2 modem
 * firmware), GNSS is briefly started to trigger the NRF_MODEM_GNSS_EVT_AGPS_REQ event from GNSS
 * in case assistance data is currently needed.
 */
static void method_gnss_agps_req_work_fn(struct k_work *work)
{
	int err;
	struct nrf_modem_gnss_agps_expiry agps_expiry;

	err = nrf_modem_gnss_agps_expiry_get(&agps_expiry);
	if (err) {
		if (err == -NRF_EOPNOTSUPP) {
			LOG_DBG("nrf_modem_gnss_agps_expiry_get() not supported by the modem "
				"firmware");

			/* Start and stop GNSS to trigger NRF_MODEM_GNSS_EVT_AGPS_REQ event if
			 * assistance data is needed.
			 */
			nrf_modem_gnss_start();
			nrf_modem_gnss_stop();
		} else {
			LOG_WRN("nrf_modem_gnss_agps_expiry_get() failed, err %d", err);
		}

		return;
	}

	agps_expiry_get_supported = true;

	if (method_gnss_agps_expiry_process(&agps_expiry)) {
		method_gnss_assistance_request();
	}
}
#endif

int method_gnss_location_get(const struct location_request_info *request)
{
	int err;

	gnss_config = *request->gnss;
#if defined(CONFIG_LOCATION_DATA_DETAILS)
	memset(&location_data_details_gnss, 0, sizeof(location_data_details_gnss));
#endif
	/* GNSS event handler is already set once in method_gnss_init(). If no other thread is
	 * using GNSS, setting it again is not needed.
	 */
	err = nrf_modem_gnss_event_handler_set(method_gnss_event_handler);
	if (err) {
		LOG_ERR("Failed to set GNSS event handler, error %d", err);
		return err;
	}

#if defined(CONFIG_NRF_CLOUD_PGPS)
	/* P-GPS is only initialized here because initialization may trigger P-GPS data request
	 * which would fail if the device is not registered to a network.
	 */
	static bool initialized;

	if (!initialized) {
		struct nrf_cloud_pgps_init_param param = {
			.event_handler = method_gnss_pgps_handler,
			/* storage is defined by CONFIG_NRF_CLOUD_PGPS_STORAGE */
			.storage_base = 0u,
			.storage_size = 0u
		};

		err = nrf_cloud_pgps_init(&param);
		if (err) {
			LOG_ERR("Error from PGPS init: %d", err);
		} else {
			initialized = true;
		}
	}
#endif
#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS)
	k_work_submit_to_queue(location_core_work_queue_get(), &method_gnss_agps_req_work);
	/* Sleep for a while before submitting the next work, otherwise A-GPS data may not be
	 * downloaded before GNSS is started. If the nrf_modem_gnss_agps_expiry_get() API is not
	 * supported, GNSS is briefly started and stopped to trigger the NRF_MODEM_GNSS_EVT_AGPS_REQ
	 * event, which in turn causes the A-GPS data download work item to be submitted into the
	 * work queue. This all needs to happen before the work item below is submitted.
	 */
	k_sleep(K_MSEC(100));
#endif

	k_work_submit_to_queue(location_core_work_queue_get(), &method_gnss_start_work);

	running = true;

	return 0;
}

#if defined(CONFIG_LOCATION_DATA_DETAILS)
void method_gnss_details_get(struct location_data_details *details)
{
	details->gnss = location_data_details_gnss;
}
#endif

int method_gnss_init(void)
{
	int err;
	running = false;

	err = nrf_modem_gnss_event_handler_set(method_gnss_event_handler);
	if (err) {
		LOG_ERR("Failed to set GNSS event handler, error %d", err);
		return err;
	}

	k_work_init(&method_gnss_pvt_work, method_gnss_pvt_work_fn);
	k_work_init(&method_gnss_start_work, method_gnss_positioning_work_fn);
#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS) || defined(CONFIG_LOCATION_GNSS_MINIMAL_ASSISTANCE)
	k_work_init(&method_gnss_agps_req_event_handle_work,
		    method_gnss_agps_req_event_handle_work_fn);
#if defined(CONFIG_NRF_CLOUD_AGPS) || defined(CONFIG_NRF_CLOUD_PGPS)
	k_work_init(&method_gnss_agps_req_work, method_gnss_agps_req_work_fn);
#endif
#endif

#if defined(CONFIG_NRF_CLOUD_PGPS)
#if defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
	k_work_init(&method_gnss_pgps_ext_work, method_gnss_pgps_ext_work_fn);
#endif
#if !defined(CONFIG_NRF_CLOUD_MQTT) && !defined(CONFIG_LOCATION_SERVICE_EXTERNAL)
	k_work_init(&method_gnss_pgps_request_work, method_gnss_pgps_request_work_fn);
#endif
	k_work_init(&method_gnss_inject_pgps_work, method_gnss_inject_pgps_work_fn);
	k_work_init(&method_gnss_notify_pgps_work, method_gnss_notify_pgps_work_fn);

#endif

#if !defined(CONFIG_NRF_CLOUD_AGPS)
	/* Subscribe to sleep notification to monitor when modem enters power saving mode */
	method_gnss_modem_sleep_notif_subscribe(MIN_SLEEP_DURATION_FOR_STARTING_GNSS);
#endif
	lte_lc_register_handler(method_gnss_lte_ind_handler);
	return 0;
}
