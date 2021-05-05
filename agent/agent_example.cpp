#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // sleep
#include <getopt.h>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <glib.h>
#include <NetworkManager.h>
#include "rmt_agent.h"
#include "mraa/led.h"

static unsigned long myid = 0;
static char *my_interface = NULL;
static char interface[50];

int get_cpu(char *payload)
{
    int ret = 0;
    int cpu_usage;
    char column[10];
    unsigned int user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
    unsigned int total_jiffies[2], work_jiffies[2];
    FILE *fp;

    if (!payload) return -1;

    for (int i = 0; i < 2; i++) {
        fp = fopen("/proc/stat", "r");
        if (!fp) {
            ret = -1;
            goto exit;
        }
        ret = fscanf(fp, "%s %u %u %u %u %u %u %u %u %u %u", column, &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal, &guest, &guest_nice);
        if (ret < 0) {
            ret = -1;
            fclose(fp);
            goto exit;
        }
        total_jiffies[i] = user + nice + system + idle + iowait + irq + softirq;
        work_jiffies[i] = user + nice + system;
        fclose(fp);
        if (i == 0) usleep(500000); // sleep 500ms
    }

    cpu_usage = (work_jiffies[1] - work_jiffies[0]) * 100 / (total_jiffies[1] - total_jiffies[0]);

    printf("cpu usage: %d\n", cpu_usage);
    sprintf(payload, "%d", cpu_usage);

exit:
    return ret;
}

int get_ram(char *payload)
{
    int ret = 0;
    char column[20], dummy[20];
    unsigned int mem_value[5];
    unsigned int total_mem, free_mem, buffer_mem, cached_mem;
    FILE *fp;
    int ram_usage;

    if (!payload) return -1;

    fp = fopen("/proc/meminfo", "r");
    if (!fp) {
        ret = -1;
        goto exit;
    }
    for (int i = 0; i < 5; i++) {
        ret = fscanf(fp, "%s %u %s", column, &mem_value[i], dummy);
        if (ret < 0) {
            ret = -1;
            fclose(fp);
            goto exit;
        }
    }
    total_mem = mem_value[0];
    free_mem = mem_value[1];
    buffer_mem = mem_value[3];
    cached_mem = mem_value[4];
    fclose(fp);

    printf("Total memory: %d\n", total_mem);
    printf("Available memory: %d\n", free_mem + buffer_mem + cached_mem);
    printf("Used memory: %d\n", total_mem - free_mem - buffer_mem - cached_mem);

    ram_usage = (total_mem - free_mem - buffer_mem - cached_mem) * 100 / total_mem;

    printf("RAM usage: %d\n", ram_usage);
    sprintf(payload, "%d", ram_usage);

exit:
    return ret;
}

int get_hostname(char *payload)
{
    char hostname[1024];

    if (!payload) return -1;

    gethostname(hostname, sizeof(hostname));
    printf("hostname: %s\n", hostname);
    sprintf(payload, "%s", hostname);

    return 0;
}

int set_hostname(char *payload)
{
    if (!payload) return -1;

    printf("hostname to be set: %s\n", payload);
    if (sethostname(payload, strlen(payload)) != 0) {
        return -1;
    }

    return 0;
}

typedef struct {
	GMainLoop *loop;
	NMConnection *local;
	const char *setting_name;
} GetSecretsData;

static void
secrets_cb (GObject *source_object, GAsyncResult *res, gpointer user_data)
{
	NMRemoteConnection *remote = NM_REMOTE_CONNECTION (source_object);
	GetSecretsData *data = (GetSecretsData*)user_data;
	GVariant *secrets;
	GError *error = NULL;

	secrets = nm_remote_connection_get_secrets_finish (remote, res, NULL);
	if (secrets) {
		if (!nm_connection_update_secrets (data->local, NULL, secrets, &error) && error) {
			g_print("Error updating secrets for %s: %s\n",
			            data->setting_name, error->message);
			g_clear_error (&error);
		}
		g_variant_unref (secrets);
	}

	g_main_loop_quit (data->loop);
}

const char*
connection_get_password(NMClient *client, const char *con_id)
{
    NMRemoteConnection *rem_con = NULL;
    NMConnection *new_connection;
    NMSettingWirelessSecurity * s_secure;
    const char *   password;
    GetSecretsData data = { 0, };

    rem_con = nm_client_get_connection_by_id(client, con_id);
    new_connection = nm_simple_connection_new_clone(NM_CONNECTION(rem_con));
    data.loop = g_main_loop_new (NULL, FALSE);
    data.local = new_connection;
    data.setting_name = "802-11-wireless-security";

    nm_remote_connection_get_secrets_async (rem_con,
		                                    "802-11-wireless-security",
		                                    NULL,
		                                    secrets_cb,
		                                    &data);
	g_main_loop_run (data.loop);

	g_main_loop_unref(data.loop);

    s_secure = (NMSettingWirelessSecurity *) nm_connection_get_setting_wireless_security(new_connection);
    password = nm_setting_wireless_security_get_psk(s_secure);

    return password;
}

int get_wifi(char *payload)
{
    int ret = 0;
    char buffer[512];
    FILE *fp;
    char interface[24];
    int rssi;
    int interface_num = 0;
    NMDevice *device;
    NMActiveConnection *active_con;
    NMConnection *new_connection;
    const char *   con_id;
    const char *   password;
    NMClient  *client;
    GError    *error = NULL;

    client = nm_client_new(NULL, &error);
    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return -1;
    }

    if (!payload) return -1;

    fp = fopen("/proc/net/wireless", "r");
    if (!fp) {
        ret = -1;
        goto exit;
    }

    // skip the first 2 lines
    for (int i = 0; i < 2; i++) {
        if (!fgets(buffer, sizeof(buffer), fp)) {
            ret = -1;
            goto exit;
        }
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        // get interface and RSSI
        sscanf(buffer, "%[^:]: %*s %*d. %d. %*d %*d %*d %*d %*d %*d %*d\n",
               interface, &rssi);
        // get SSID
        int sock_fd;
        struct iwreq wreq;
        char ssid[IW_ESSID_MAX_SIZE + 1];

        memset(&wreq, 0, sizeof(struct iwreq));
        memset(ssid, 0, sizeof(ssid));
        strcpy(wreq.ifr_name, interface);
        if ((sock_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            ret = -1;
            goto exit;
        }
        wreq.u.essid.pointer = ssid;
        wreq.u.essid.length = IW_ESSID_MAX_SIZE;
        if (ioctl(sock_fd, SIOCGIWESSID, &wreq)) {
            ret = -1;
            goto exit;
        }
        close(sock_fd);

        device = nm_client_get_device_by_iface(client, interface);
        active_con = nm_device_get_active_connection(device);
        new_connection = nm_simple_connection_new_clone(NM_CONNECTION(nm_active_connection_get_connection(active_con)));
        con_id = nm_connection_get_id(new_connection);
        password = connection_get_password(client, con_id);

        printf("%s: ssid=%s rssi=%d\n", interface, ssid, rssi);
        if (interface_num != 0) {
            sprintf(payload, ",");
        }
        sprintf(payload, "%s %s %d %s", interface, ssid, rssi, password);
        interface_num++;
    }
    if (interface_num == 0) {
        sprintf(payload, "none");
    }
    fclose(fp);
    g_object_unref(client);

exit:
    return ret;
}

NMConnection* get_client_nmconnection(const char* uuid, GString* ssid, const char * password)
{
    NMConnection *connection = NULL;
    NMSettingConnection *s_con;
    NMSettingWireless   *s_wireless;
    NMSettingIP4Config  *s_ip4;
    NMSettingWirelessSecurity * s_secure;

    connection = nm_simple_connection_new();
    s_wireless = (NMSettingWireless *) nm_setting_wireless_new();
    s_secure = (NMSettingWirelessSecurity *) nm_setting_wireless_security_new();
    s_con = (NMSettingConnection *) nm_setting_connection_new();

    g_object_set(G_OBJECT(s_con),
                 NM_SETTING_CONNECTION_UUID,
                 uuid,
                 NM_SETTING_CONNECTION_ID,
                 "RMTClient",
                 NM_SETTING_CONNECTION_TYPE,
                 "802-11-wireless",
                 NM_SETTING_CONNECTION_AUTOCONNECT_PRIORITY,
                 NM_SETTING_CONNECTION_AUTOCONNECT_PRIORITY_MAX,
                 NULL);
    nm_connection_add_setting(connection, NM_SETTING(s_con));

    s_wireless = (NMSettingWireless *) nm_setting_wireless_new();

    g_object_set(G_OBJECT(s_wireless),
                 NM_SETTING_WIRELESS_SSID,
                 ssid,
                 NULL);
    nm_connection_add_setting(connection, NM_SETTING(s_wireless));

    s_secure = (NMSettingWirelessSecurity *) nm_setting_wireless_security_new();
    g_object_set(G_OBJECT(s_secure),
                 NM_SETTING_WIRELESS_SECURITY_KEY_MGMT,
                 "wpa-psk",
                 NM_SETTING_WIRELESS_SECURITY_PSK,
                 password,
                 NULL);
    nm_connection_add_setting(connection, NM_SETTING(s_secure));

    s_ip4 = (NMSettingIP4Config *) nm_setting_ip4_config_new();
    g_object_set(G_OBJECT(s_ip4),
                 NM_SETTING_IP_CONFIG_METHOD,
                 NM_SETTING_IP4_CONFIG_METHOD_AUTO,
                 NULL);
    nm_connection_add_setting(connection, NM_SETTING(s_ip4));

    return connection;
}

static void
added_cb(GObject *client, GAsyncResult *result, gpointer user_data)
{
    NMRemoteConnection *remote;
    GError             *error = NULL;

    /* NM responded to our request; either handle the resulting error or
     * print out the object path of the connection we just added.
     */
    remote = nm_client_add_connection_finish(NM_CLIENT(client), result, &error);
    if (error) {
        g_print("Error adding connection: %s", error->message);
        g_error_free(error);
    } else {
        g_print("Added: %s\n", nm_connection_get_path(NM_CONNECTION(remote)));
        g_object_unref(remote);
    }
    /* Tell the mainloop we're done and we can quit now */
    g_main_loop_quit((GMainLoop*)user_data);
}

static void
add_wifi_connection(NMClient *client)
{
    NMConnection *connection;
    GMainLoop    *loop;
    GError       *error = NULL;
    const char   *uuid;
    const char   *password;

    loop = g_main_loop_new(NULL, FALSE);

    /* Create a new connection object */
    uuid  = nm_utils_uuid_generate();
    GString* ssid = g_string_new("RMTserver");
    password = "adlinkros";
    connection = get_client_nmconnection(uuid, ssid, password);

    /* Ask the settings service to add the new connection; we'll quit the
     * mainloop and exit when the callback is called.
     */
    nm_client_add_connection_async(client, connection, TRUE, NULL, added_cb, loop);
    g_object_unref(connection);

    g_main_loop_run(loop);
    /* Clean up */
    g_object_unref(client);
}

static void
modify_connection_cb (GObject *connection,
                      GAsyncResult *result,
                      gpointer user_data)
{
	GError *error = NULL;
	if (!nm_remote_connection_commit_changes_finish (NM_REMOTE_CONNECTION (connection),
	                                                 result, &error)) {
		printf(("Error: Failed to modify connection '%s': %s"),
                nm_connection_get_id (NM_CONNECTION (connection)),
                error->message);
	} 
    else {
        printf(("Connection '%s' (%s) successfully modified.\n"),
                nm_connection_get_id (NM_CONNECTION (connection)),
                nm_connection_get_uuid (NM_CONNECTION (connection)));
	}
    g_main_loop_quit((GMainLoop*)user_data);

}

static void
modify_wifi(const char * ssid, const char * password)
{
    NMClient  *client;
    GMainLoop *loop;
    GError    *error = NULL;
    loop = g_main_loop_new(NULL, FALSE);
    client = nm_client_new(NULL, &error);
    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return ;
    }

    NMRemoteConnection *rem_con = NULL;
    NMConnection       *connection = NULL;
    gboolean            temporary = FALSE;
    const char         *uuid;

    rem_con = nm_client_get_connection_by_id(client, "RMTClient");
    connection = nm_simple_connection_new();

    GString* g_ssid = g_string_new(ssid);
    uuid  = nm_setting_connection_get_uuid(nm_connection_get_setting_connection(NM_CONNECTION (rem_con)));

    connection = get_client_nmconnection(uuid, g_ssid, password);

    nm_connection_replace_settings_from_connection (NM_CONNECTION (rem_con),
					                                            connection);
    nm_remote_connection_commit_changes_async(rem_con,
	                                           !temporary,
	                                           NULL,
	                                           modify_connection_cb,
	                                           loop);
    g_object_unref(connection);
    g_main_loop_run(loop);
    g_object_unref(client);

}

/*
 * Set RMTClient connection with "<ssid> <password>""
 */
int set_wifi(char *payload)
{
    if (!payload) return -1;

    char ssid[32];
    char password[32];
    
    sscanf(payload, "%s %s", ssid, password);
    modify_wifi(ssid, password);

    return 0;
}

#define LED_NUM 0
static int locate_on = 0;
int set_locate(char *payload)
{
    if (!payload) return -1;

    if (strcmp(payload, "on") == 0) {
        locate_on = 1;
    } else {
        locate_on = 0;
    }

    printf("set LED: %d\n", locate_on);

    return 0;
}

/*
 * status=0: dark
 * status=1: bright
 */
void set_led_status(int status)
{
    mraa_led_context led;
    mraa_result_t result;

    led = mraa_led_init(LED_NUM);
    if (led == NULL) {
        perror("Unable to init LED");
        goto exit;
    }

    result = mraa_led_set_brightness(led, status);
    if (result != MRAA_SUCCESS) {
        perror("Unable to set LED");
        goto exit;
    }

exit:
    if (led)
        mraa_led_close(led);
}

void locate_daemon(void)
{
    static int init = 0;
    static int led_status;
    static time_t start_time;

    if (!init) {
        init = 1;
        led_status = 0;
        start_time = time(NULL);
    }
    if (locate_on && (start_time != time(NULL))) {
        led_status = !led_status;
        set_led_status(led_status);
        start_time = time(NULL);
    }
    // If we do not locate the device, we need to make the led_status back to 0.
    // RMT_TODO: We need to close LED while user presses ctrl+C.
    if (!locate_on && (led_status != 0)) {
        led_status = 0;
        set_led_status(led_status);
    }
}

static datainfo_func func_maps[] = {
    {"cpu",      get_cpu,      NULL         },
    {"ram",      get_ram,      NULL         },
    {"hostname", get_hostname, set_hostname },
    {"wifi",     get_wifi,     set_wifi         },
    {"locate",   NULL,         set_locate   },
    {0,          0,            0            },
};

const char *short_options = "i:n:h";
struct option long_options[] = {
    {"id",   required_argument, NULL, 'i'},
    {"net",  required_argument, NULL, 'n'},
    {"help", no_argument,       NULL, 'h'},
    { 0,     0,                 0,    0  },
};

void print_help(void)
{
    printf("Usage: ./agent_example [options]\n");
    printf("* --help: Showing this messages.\n");
    printf("* --id [myID]: Use myID as the ID.\n");
    printf("* --net [interface]: Decide which interface agent uses.\n");
}

int main(int argc, char *argv[])
{
    int cmd_opt = 0;
    GError *   error = NULL;
    NMClient * client;
    client = nm_client_new(NULL, &error);
    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return 1;
    }
    // Parse argument
    while ((cmd_opt = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) {
        switch (cmd_opt) {
            case 'i':
                myid = atoi(optarg);
                break;
            case 'n':
                if (strlen(optarg) > 0) {
                    strcpy(interface, optarg);
                    my_interface = interface;
                }
                break;
            case 'h':
                print_help();
                return 0;
            case '?':
            default:
                printf("Not supported option\n");
                print_help();
                return 1;
        }
    }

    printf("This is RMT Agent. id=%lu and network interface=%s\n", myid, my_interface);
    rmt_agent_config(my_interface, myid);
    rmt_agent_init(func_maps);
    mraa_init();
    if (! nm_client_get_connection_by_id(client, "RMTClient")){
        g_print("Create RMTClient wifi connection\n");
        add_wifi_connection(client);
    }
    while (1) {
        rmt_agent_running();
        locate_daemon();
        usleep(10000); // sleep 10ms
    }
    mraa_deinit();
    rmt_agent_deinit();
    return 0;
}
