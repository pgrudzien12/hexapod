/*
 * Minimal Bluetooth-only RPC command parser implementation.
 * Supports commands:
 *   get <ns> <param>
 *   set <ns> <param> <value>
 *   setpersist <ns> <param> <value>
 *   export <ns>
 *   factory-reset
 *   save [ns]
 *   help
 *   version
 *   list namespaces | list <ns>
 */

#include "rpc_commands.h"
#include "rpc_transport.h"
#include "config_manager.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "controller_internal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>

static const char *TAG = "rpc"; // used for logging

// Line buffer
static char line_buf[256];
static size_t line_len = 0;
static rpc_send_fn_t g_send_cb = NULL;
static rpc_transport_type_t g_active_transport = RPC_TRANSPORT_BLUETOOTH;

static void rpc_send(const char *fmt, ...) {
	ESP_LOGI(TAG, "rpc_send called");
	char stack_buf[256];
	va_list ap; va_start(ap, fmt);
	int needed = vsnprintf(stack_buf, sizeof(stack_buf), fmt, ap);
	va_end(ap);
	if (needed < (int)sizeof(stack_buf)) {
		// Fits in stack buffer
		size_t len = (size_t)needed;
		if (len >= sizeof(stack_buf)-2) len = sizeof(stack_buf)-3;
		stack_buf[len++]='\r'; stack_buf[len++]='\n'; stack_buf[len]='\0';
		if (g_send_cb) {
			g_send_cb(stack_buf);
		} else {
			rpc_transport_tx_send(g_active_transport, stack_buf, len);
		}
	} else {
		// Allocate exact size (+CRLF)
		size_t len_total = (size_t)needed + 3; // CRLF + null
		char *dyn = (char*)malloc(len_total);
		if (!dyn) {
			// Fallback truncated
			size_t len = sizeof(stack_buf)-3; stack_buf[len++]='\r'; stack_buf[len++]='\n'; stack_buf[len]='\0';
			if (g_send_cb) {
				g_send_cb(stack_buf);
			} else {
				rpc_transport_tx_send(g_active_transport, stack_buf, len);
			}
			return;
		}
		va_list ap2; va_start(ap2, fmt);
		vsnprintf(dyn, len_total, fmt, ap2);
		va_end(ap2);
		size_t l = strlen(dyn);
		dyn[l++]='\r'; dyn[l++]='\n'; dyn[l]='\0';
		if (g_send_cb) {
			g_send_cb(dyn);
		} else {
			rpc_transport_tx_send(g_active_transport, dyn, l);
		}
		free(dyn);
	}
}

void rpc_set_send_callback(rpc_send_fn_t fn) { g_send_cb = fn; }

static void rpc_processing_task(void* param) {
    ESP_LOGI(TAG, "RPC processing task started");
    
    while (1) {
        rpc_rx_message_t msg;
        esp_err_t err = rpc_transport_rx_receive(&msg, 1000); // 1 second timeout
        
        if (err == ESP_OK) {
            // Set active transport for responses
            g_active_transport = msg.transport;
            // Process the received data
            rpc_feed_bytes(msg.data, msg.len);
        }
    }
}

void rpc_init(void) {
    line_len = 0;
    
    // Initialize transport layer
    esp_err_t err = rpc_transport_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RPC transport: %s", esp_err_to_name(err));
        return;
    }
    
    // Start RPC processing task
    xTaskCreate(rpc_processing_task, "rpc_proc", 4096, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "RPC initialized");
}static void trim(char *s) {
	// remove leading/trailing whitespace
	char *start = s; while (*start && isspace((unsigned char)*start)) start++;
	char *end = start + strlen(start); while (end>start && isspace((unsigned char)end[-1])) --end; *end='\0';
	if (start != s) memmove(s, start, end-start+1);
}

static int tokenize(char *line, char *argv[], int max_args) {
	int argc = 0;
	char *p = line;
	while (*p && argc < max_args) {
		while (isspace((unsigned char)*p)) ++p;
		if (!*p) break;
		argv[argc++] = p;
		while (*p && !isspace((unsigned char)*p)) ++p;
		if (*p) { *p = '\0'; ++p; }
	}
	return argc;
}

static void cmd_help(void) {
	rpc_send("Commands: get set setpersist export factory-reset save list help version");
}

static void cmd_version(void) {
	rpc_send("version: firmware %s", esp_get_idf_version());
}

// Helper function to safely concatenate string list into buffer
static void safe_string_list_concat(char* buf, size_t buf_size, const char** items, size_t count) {
	buf[0] = '\0';
	size_t pos = 0;
	
	for (size_t i = 0; i < count; i++) {
		int written = snprintf(buf + pos, buf_size - pos, "%s%s", 
		                       items[i], 
		                       (i + 1 < count) ? ", " : "");
		
		if (written < 0 || (size_t)written >= buf_size - pos) {
			// Would overflow or error, stop here
			break;
		}
		pos += written;
	}
}

static void cmd_list(int argc, char *argv[]) {
	if (argc >= 2 && strcmp(argv[1], "namespaces") == 0) {
		const char* names[CONFIG_NS_COUNT]; size_t count=0;
		if (config_list_namespaces(names, &count)==ESP_OK) {
			char buf[128];
			safe_string_list_concat(buf, sizeof(buf), names, count);
			rpc_send("namespaces: %s", buf);
		} else rpc_send("ERROR list namespaces");
		return;
	}
	if (argc >= 2) {
		const char* params[10]; size_t count=0; // Reduced from 32 to 10 to fit in response
		if (config_list_parameters(argv[1], params, 10, &count)==ESP_OK) {
			char buf[256];
			safe_string_list_concat(buf, sizeof(buf), params, count);
			rpc_send("list %s: %s", argv[1], buf);
		} else rpc_send("ERROR list %s", argv[1]);
	} else {
		rpc_send("usage: list namespaces | list <namespace>");
	}
}

static void cmd_get(int argc, char *argv[]) {
	if (argc < 3) { rpc_send("usage: get <namespace> <param>"); return; }
	config_param_info_t info; if (config_get_parameter_info(argv[1], argv[2], &info)!=ESP_OK){rpc_send("get: not found"); return;}
	char vbuf[128];
	switch(info.type) {
		case CONFIG_TYPE_BOOL: { bool v; if(hexapod_config_get_bool(argv[1], argv[2], &v)==ESP_OK) rpc_send("%s=%s", argv[2], v?"true":"false"); else rpc_send("ERROR get bool"); break; }
		case CONFIG_TYPE_INT32: { int32_t v; if(hexapod_config_get_int32(argv[1], argv[2], &v)==ESP_OK) rpc_send("%s=%ld", argv[2], (long)v); else rpc_send("ERROR get int32"); break; }
		case CONFIG_TYPE_UINT16: { uint32_t v; if(hexapod_config_get_uint32(argv[1], argv[2], &v)==ESP_OK) rpc_send("%s=%lu", argv[2], (unsigned long)v); else rpc_send("ERROR get u16"); break; }
		case CONFIG_TYPE_UINT32: { uint32_t v; if(hexapod_config_get_uint32(argv[1], argv[2], &v)==ESP_OK) rpc_send("%s=%lu", argv[2], (unsigned long)v); else rpc_send("ERROR get u32"); break; }
		case CONFIG_TYPE_FLOAT: { float v; if(hexapod_config_get_float(argv[1], argv[2], &v)==ESP_OK) rpc_send("%s=%0.3f", argv[2], v); else rpc_send("ERROR get float"); break; }
		case CONFIG_TYPE_STRING: { if(hexapod_config_get_string(argv[1], argv[2], vbuf, sizeof(vbuf))==ESP_OK) rpc_send("%s=%s", argv[2], vbuf); else rpc_send("ERROR get string"); break; }
		default: rpc_send("ERROR unsupported type");
	}
}

static void cmd_set_common(const char* ns, const char* param, const char* value, bool persist) {
	config_param_info_t info; if (config_get_parameter_info(ns, param, &info)!=ESP_OK){rpc_send("set: not found"); return;}
	esp_err_t err = ESP_FAIL;
	switch(info.type) {
		case CONFIG_TYPE_BOOL: err = hexapod_config_set_bool(ns, param, (strcmp(value,"true")==0 || strcmp(value,"1")==0), persist); break;
		case CONFIG_TYPE_INT32: err = hexapod_config_set_int32(ns, param, (int32_t)strtol(value,NULL,10), persist); break;
		case CONFIG_TYPE_UINT16: /* fallthrough */
		case CONFIG_TYPE_UINT32: err = hexapod_config_set_uint32(ns, param, (uint32_t)strtoul(value,NULL,10), persist); break;
		case CONFIG_TYPE_FLOAT: err = hexapod_config_set_float(ns, param, (float)strtod(value,NULL), persist); break;
		case CONFIG_TYPE_STRING: err = hexapod_config_set_string(ns, param, value, persist); break;
		default: rpc_send("ERROR type"); return;
	}
	if (err==ESP_OK) rpc_send("%s=%s (%s)", param, value, persist?"persist":"mem"); else rpc_send("ERROR set %s", param);
}

static void cmd_set(int argc, char *argv[], bool persist) {
	if (argc < 4) { rpc_send("usage: %s <ns> <param> <value>", persist?"setpersist":"set"); return; }
	cmd_set_common(argv[1], argv[2], argv[3], persist);
}

static void cmd_export(int argc, char *argv[]) {
	if (argc < 2) { rpc_send("usage: export <namespace>"); return; }
	if (strcmp(argv[1], "system")!=0) { rpc_send("export: unsupported ns"); return; }
	const system_config_t *sys = config_get_system();
	if (!sys) { rpc_send("export: no system config"); return; }
	// Worst case ~1KB, allocate dynamic
	char *json = (char*)malloc(1024);
	if (!json) { rpc_send("export: OOM"); return; }
	int written = snprintf(json, 1024,
		"{\"emergency_stop_enabled\":%s,\"auto_disarm_timeout\":%lu,\"safety_voltage_min\":%.3f,\"temperature_limit_max\":%.2f,\"motion_timeout_ms\":%lu,\"startup_delay_ms\":%lu,\"max_control_frequency\":%lu,\"robot_id\":\"%s\",\"robot_name\":\"%s\",\"config_version\":%u}",
		sys->emergency_stop_enabled?"true":"false",
		(unsigned long)sys->auto_disarm_timeout,
		sys->safety_voltage_min,
		sys->temperature_limit_max,
		(unsigned long)sys->motion_timeout_ms,
		(unsigned long)sys->startup_delay_ms,
		(unsigned long)sys->max_control_frequency,
		sys->robot_id,
		sys->robot_name,
		sys->config_version);
	if (written < 0 || written >= 1024) {
		rpc_send("export: truncation");
	} else {
		rpc_send("export system: %s", json);
	}
	free(json);
}

static void cmd_save(int argc, char *argv[]) {
	if (argc == 1) {
		// Save all namespaces (currently only system)
		if (config_manager_save_namespace(CONFIG_NS_SYSTEM)==ESP_OK) rpc_send("save: SYSTEM OK");
		if (config_manager_save_namespace(CONFIG_NS_JOINT_CALIB)==ESP_OK) rpc_send("save: JOINT_CAL OK");
		else rpc_send("save: ERROR");
		return;
	}
	if (strcmp(argv[1], CONFIG_NAMESPACE_NAMES[CONFIG_NS_SYSTEM])==0) {
		if (config_manager_save_namespace(CONFIG_NS_SYSTEM)==ESP_OK) rpc_send("save SYSTEM: OK"); else rpc_send("save SYSTEM: ERROR");
	} else if (strcmp(argv[1], CONFIG_NAMESPACE_NAMES[CONFIG_NS_JOINT_CALIB])==0) {
		if (config_manager_save_namespace(CONFIG_NS_JOINT_CALIB)==ESP_OK) rpc_send("save JOINT_CAL: OK"); else rpc_send("save JOINT_CAL: ERROR");
	} else rpc_send("save: unsupported namespace");
}

static void cmd_factory_reset(void) {
	if (config_factory_reset()==ESP_OK) rpc_send("factory-reset: OK"); else rpc_send("factory-reset: ERROR");
}

static void cmd_set_controller(int argc, char *argv[]) {
    if (argc < 3) { // at least "set" "controller" are required
        rpc_send("usage: set controller <ch0>...<ch31>");
        return;
    }
    int16_t channels[CONTROLLER_MAX_CHANNELS];
    int num_channels_provided = argc - 2;

    for (int i = 0; i < CONTROLLER_MAX_CHANNELS; i++) {
        if (i < num_channels_provided) {
            channels[i] = (int16_t)strtol(argv[i + 2], NULL, 10);
        } else {
            channels[i] = 0;
        }
    }
    controller_internal_update_channels(channels);
	// No response needed for high-frequency commands from external sources
}

static void rpc_execute_line(char *line) {
	trim(line);
	if (line[0]=='\0') return;
	char *argv[34]; // Increased for set controller
    int argc = tokenize(line, argv, 34);
	if (argc == 0) return;

    if (strcmp(argv[0], "set") == 0 && argc > 1 && strcmp(argv[1], "controller") == 0) {
        cmd_set_controller(argc, argv);
    }
	else if (strcmp(argv[0], "help")==0) { cmd_help(); }
	else if (strcmp(argv[0], "version")==0) { cmd_version(); }
	else if (strcmp(argv[0], "list")==0) { cmd_list(argc, argv); }
	else if (strcmp(argv[0], "get")==0) { cmd_get(argc, argv); }
	else if (strcmp(argv[0], "set")==0) { cmd_set(argc, argv, false); }
	else if (strcmp(argv[0], "setpersist")==0) { cmd_set(argc, argv, true); }
	else if (strcmp(argv[0], "export")==0) { cmd_export(argc, argv); }
	else if (strcmp(argv[0], "save")==0) { cmd_save(argc, argv); }
	else if (strcmp(argv[0], "factory-reset")==0) { cmd_factory_reset(); }
	else { rpc_send("unknown: %s", argv[0]); }
}

void rpc_feed_bytes(const uint8_t *data, size_t len) {
	if (!data || len==0) return;
	for (size_t i=0;i<len;i++) {
		char c = (char)data[i];
		if (isprint((unsigned char)c) || isspace((unsigned char)c)) {
			if (line_len < sizeof(line_buf)-1) {
				line_buf[line_len++] = c;
			} else {
				// overflow, reset
				line_len = 0;
				rpc_send("ERROR line too long");
			}
		}
	}
	
	line_buf[line_len] = '\0';
	rpc_execute_line(line_buf);
	line_len = 0;
}

void rpc_poll(void) {
	// Reserved for future periodic tasks
}

