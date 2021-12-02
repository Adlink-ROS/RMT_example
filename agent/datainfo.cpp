#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <glib.h>
#include <NetworkManager.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#ifdef SUPPORT_ROS
 #include <rclcpp/rclcpp.hpp>
#endif /*SUPPORT_ROS*/
#include <sstream>
#include <fstream>
#include <iostream>
#include <pwd.h>
#ifdef SUPPORT_NLIB
 #include "mraa/led.h"
#endif /*SUPPORT_NLIB*/
#include "yaml-cpp/yaml.h"
#include <vector>

char interface[50];
static bool reapply_required = false;
static char current_device[16];
#ifdef SUPPORT_ROS
rclcpp::Node::SharedPtr node;
#endif /*SUPPORT_ROS*/

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

    char cmd[1024] = "hostnamectl set-hostname ";
    strcat(cmd, payload);
    std::array < char, 128 > buffer;
    std::string result;
    std::unique_ptr < FILE, decltype(&pclose) > pipe(popen(cmd, "r"), pclose);

    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    std::cout << result << std::endl;

    return 0;
}

typedef struct {
    GMainLoop *loop;
    NMConnection *local;
    const char *setting_name;
} GetSecretsData;

typedef struct {
    NMClient  *client;
    NMDevice  *device;
    GMainLoop *loop;
    int result;
} WifiModifyData;

void secrets_cb (GObject *source_object, GAsyncResult *res, gpointer user_data)
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

const char* connection_get_password(NMClient *client, const char *con_id)
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
    NMConnection *new_connection;
    const char *   con_id;
    const char *   password;
    const char *   ssid;
    NMClient   *client;
    GError     *error = NULL;
    GBytes             *active_ssid;
    NMRemoteConnection *rem_con = NULL;
    NMSettingWireless  *s_wireless;

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

        rem_con = nm_client_get_connection_by_id(client, "RMTClient");
        new_connection = nm_simple_connection_new_clone(NM_CONNECTION(rem_con));
        con_id = nm_connection_get_id(new_connection);
        s_wireless = nm_connection_get_setting_wireless(new_connection);
        active_ssid = nm_setting_wireless_get_ssid(s_wireless);
        password = connection_get_password(client, con_id);
        ssid = nm_utils_ssid_to_utf8((guint8*)g_bytes_get_data(active_ssid, NULL),
                                     g_bytes_get_size(active_ssid));

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

NMConnection* get_client_nmconnection(const char* connection_id, const char* uuid, GString* ssid, const char * password)
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
                 connection_id,
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

void added_cb(GObject *client, GAsyncResult *result, gpointer user_data)
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

void add_wifi_connection(NMClient *client)
{
    NMConnection *connection;
    GMainLoop    *loop;
    const char   *uuid;
    const char   *password;

    loop = g_main_loop_new(NULL, FALSE);

    /* Create a new connection object */
    uuid = nm_utils_uuid_generate();
    GString* ssid = g_string_new("RMTserver");
    password = "adlinkros";
    connection = get_client_nmconnection("RMTClient", uuid, ssid, password);

    /* Ask the settings service to add the new connection; we'll quit the
     * mainloop and exit when the callback is called.
     */
    nm_client_add_connection_async(client, connection, TRUE, NULL, added_cb, loop);
    g_object_unref(connection);

    g_main_loop_run(loop);
    /* Clean up */
    g_object_unref(client);
}

void activate_cb (GObject *client, GAsyncResult *result, gpointer user_data)
{
    WifiModifyData *data = (WifiModifyData*)user_data;
    GError *error = NULL;

    nm_client_activate_connection_finish(data->client, result, &error);
    if (error) {
        data->result = -1;
        g_print("Error in activating connection: %s", error->message);
        g_error_free(error);
    } else {
        data->result = 0;
        g_print(("Connection successfully activated.\n"));
    }

    g_main_loop_quit(data->loop);
}

void modify_wifi_cb (GObject *connection, GAsyncResult *result, gpointer user_data)
{
    GError *error = NULL;
    const GPtrArray *connections;
    NMConnection *new_connection;
    const char *name;
    WifiModifyData *data = (WifiModifyData*)user_data;

    if (!nm_remote_connection_commit_changes_finish (NM_REMOTE_CONNECTION (connection),
                                                     result, &error)) {
        g_print(("Error: Failed to modify connection '%s': %s\n"),
                nm_connection_get_id (NM_CONNECTION (connection)),
                error->message);
        data->result = -1;
        g_error_free(error);
        g_main_loop_quit(data->loop);
    } else {
        g_print(("Connection '%s' (%s) successfully modified.\n"),
                nm_connection_get_id (NM_CONNECTION (connection)),
                nm_connection_get_uuid (NM_CONNECTION (connection)));

        connections = nm_device_get_available_connections(data->device);
        bool find_connection = false;

        for (int i = 0; i < connections->len; i++) {
            new_connection = nm_simple_connection_new_clone(NM_CONNECTION(connections->pdata[i]));
            name = nm_connection_get_id(new_connection);

            if (!strcmp(name, "RMTClient")) {
                nm_client_activate_connection_async(data->client, new_connection, data->device,
                                                    NULL, NULL, activate_cb, data);
                find_connection = true;
                break;
            }
        }

        if (!find_connection) {
            data->result = 0;
            g_main_loop_quit(data->loop);
        }
    }
}

void reapply_cb(GObject *device, GAsyncResult *result, gpointer user_data)
{
    GError *error = NULL;
    WifiModifyData *data = (WifiModifyData*)user_data;

    nm_device_reapply_finish(data->device, result, &error);

    if (error) {
        data->result = -1;
        g_print("Error reapply connection: %s", error->message);
        g_error_free(error);
    } else {
        data->result = 0;
        g_print(("Connection configuration successfully reapplied.\n"));
    }

    g_main_loop_quit(data->loop);
}

void modify_address_cb (GObject *connection, GAsyncResult *result, gpointer user_data)
{
    GError *error = NULL;
    WifiModifyData *data = (WifiModifyData*)user_data;

    if (!nm_remote_connection_commit_changes_finish (NM_REMOTE_CONNECTION (connection),
                                                     result, &error)) {
        g_print(("Error: Failed to modify connection '%s': %s\n"),
                nm_connection_get_id (NM_CONNECTION (connection)),
                error->message);
        g_error_free(error);
        data->result = -1;
    } else {
        g_print(("Connection '%s' (%s) successfully modified IPv4 address.\n"),
                nm_connection_get_id (NM_CONNECTION (connection)),
                nm_connection_get_uuid (NM_CONNECTION (connection)));
        reapply_required = true;
        data->result = 0;
    }

    g_main_loop_quit(data->loop);
}

int modify_wifi(const char * ssid, const char * password)
{
    NMClient  *client;
    GMainLoop *loop;
    GError    *error = NULL;
    WifiModifyData *wifi_modify_data;
    const GPtrArray *devices;
    NMDevice *device;
    NMRemoteConnection *rem_con = NULL;
    NMConnection       *new_connection;
    gboolean temporary = FALSE;
    NMSettingWireless   *s_wireless;
    NMSettingWirelessSecurity * s_secure;

    loop = g_main_loop_new(NULL, FALSE);
    client = nm_client_new(NULL, &error);

    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return -1;
    }

    devices = nm_client_get_devices(client);

    for (int i = 0; i < devices->len; i++) {
        device = (NMDevice *)devices->pdata[i];
        if (NM_IS_DEVICE_WIFI(device)) break;
    }

    wifi_modify_data = g_slice_new(WifiModifyData);
    *wifi_modify_data = (WifiModifyData) {
        .client = client,
        .device = device,
        .loop = loop,
    };

    rem_con = nm_client_get_connection_by_id(client, "RMTClient");
    new_connection = nm_simple_connection_new_clone(NM_CONNECTION(rem_con));
    GString* g_ssid = g_string_new(ssid);

    nm_connection_remove_setting(new_connection, NM_TYPE_SETTING_WIRELESS_SECURITY);
    nm_connection_remove_setting(new_connection, NM_TYPE_SETTING_WIRELESS);

    s_wireless = (NMSettingWireless *) nm_setting_wireless_new();

    g_object_set(G_OBJECT(s_wireless),
                 NM_SETTING_WIRELESS_SSID,
                 g_ssid,
                 NULL);
    nm_connection_add_setting(new_connection, NM_SETTING(s_wireless));

    s_secure = (NMSettingWirelessSecurity *) nm_setting_wireless_security_new();
    g_object_set(G_OBJECT(s_secure),
                 NM_SETTING_WIRELESS_SECURITY_KEY_MGMT,
                 "wpa-psk",
                 NM_SETTING_WIRELESS_SECURITY_PSK,
                 password,
                 NULL);

    nm_connection_add_setting(new_connection, NM_SETTING(s_secure));

    nm_connection_replace_settings_from_connection (NM_CONNECTION (rem_con),
                                                    new_connection);
    nm_remote_connection_commit_changes_async(rem_con,
                                              !temporary,
                                              NULL,
                                              modify_wifi_cb,
                                              wifi_modify_data);
    g_object_unref(new_connection);
    g_main_loop_run(wifi_modify_data->loop);
    g_object_unref(wifi_modify_data->client);

    return wifi_modify_data->result;
}

/*
 * Set RMTClient connection with "<ssid> <password>""
 */
int set_wifi(char *payload)
{
    if (!payload) return -1;

    char ssid[33];
    char password[33];
    int result;

    sscanf(payload, "%s %s", ssid, password);
    result = modify_wifi(ssid, password);

    return result;
}

int update_ip4(const char* device_name, const char* method, const char* address, int prefix, const char* gateway)
{
    NMClient  *client;
    NMDevice *device;
    GMainLoop *loop;
    GError    *error = NULL;
    WifiModifyData *wifi_modify_data;
    gboolean temporary = FALSE;
    NMSettingIPConfig * s_ip4;
    NMConnection *new_connection;
    NMActiveConnection *active_con;
    NMRemoteConnection *rem_con = NULL;
    NMIPAddress *ip_address;

    loop = g_main_loop_new(NULL, FALSE);
    client = nm_client_new(NULL, &error);

    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return -1;
    }

    device = nm_client_get_device_by_iface(client, device_name);

    if (!device) {
        g_print("No such device found, device name error!\n");
        g_object_unref(client);

        return -1;
    }

    active_con = nm_device_get_active_connection(device);

    if (!active_con) {
        printf("No current active connection on this device!\n");
        g_object_unref(client);

        return -1;
    }

    rem_con = nm_active_connection_get_connection(active_con);
    new_connection = nm_simple_connection_new_clone(NM_CONNECTION(rem_con));
    s_ip4 = (NMSettingIPConfig *) nm_setting_ip4_config_new();

    nm_connection_remove_setting(new_connection, NM_TYPE_SETTING_IP4_CONFIG);

    if (!strcmp(method, "auto")) {
        g_object_set(s_ip4,
                     NM_SETTING_IP_CONFIG_METHOD,
                     NM_SETTING_IP4_CONFIG_METHOD_AUTO,
                     NULL);
    } else if (!strcmp(method, "manual")) {
        ip_address = nm_ip_address_new(AF_INET, address, prefix, NULL);

        g_object_set(s_ip4,
                     NM_SETTING_IP_CONFIG_METHOD,
                     NM_SETTING_IP4_CONFIG_METHOD_MANUAL,
                     NULL);

        if (strlen(gateway)) {
            g_object_set(s_ip4,
                         NM_SETTING_IP_CONFIG_GATEWAY,
                         gateway,
                         NULL);
        }

        nm_setting_ip_config_add_address(s_ip4, ip_address);
        nm_ip_address_unref(ip_address);
    } else {
        g_print("IP setting method not found\n");
        g_object_unref(client);

        return -1;
    }

    nm_connection_add_setting(new_connection, NM_SETTING(s_ip4));

    if (!nm_connection_verify(new_connection, NULL)) {
        g_print("Error: invalid property of connection, abort action.\n");
        g_object_unref(client);

        return -1;
    }

    wifi_modify_data = g_slice_new(WifiModifyData);
    *wifi_modify_data = (WifiModifyData) {
        .client = client,
        .device = device,
        .loop = loop,
    };

    nm_connection_replace_settings_from_connection (NM_CONNECTION (rem_con),
                                                    new_connection);
    nm_remote_connection_commit_changes_async(rem_con,
                                              !temporary,
                                              NULL,
                                              modify_address_cb,
                                              wifi_modify_data);
    g_object_unref(new_connection);
    g_main_loop_run(wifi_modify_data->loop);
    g_object_unref(wifi_modify_data->client);

    return wifi_modify_data->result;
}

/*
 * Set RMTClient connection ip address with "<device> <method> <address> <prefix> <gateway>(optional)"
 * For setting to auto(DHCP), the method should be "auto", input except "device" and "method" is unnecessary.
 * For setting to manual, the input would be like "wlp1s0 manual 192.168.50.26 24".
 */
int set_ip_address(char *payload)
{
    if (!payload) return -1;

    char method[10], address[16];
    char gateway[16] = "";
    char *temp;
    int prefix, result;
    std::vector < char* > ip_setting;

    temp = strtok(payload, " ");

    while (temp) {
        ip_setting.push_back(temp);
        temp = strtok(NULL, " ");
    }

    if (ip_setting.size() < 2) return -1;

    strcpy(current_device, ip_setting[0]);
    strcpy(method, ip_setting[1]);

    if (ip_setting.size() > 2) {
        strcpy(address, ip_setting[2]);
        prefix = atoi(ip_setting[3]);

        if (ip_setting.size() > 4) strcpy(gateway, ip_setting[4]);
    }

    result = update_ip4(current_device, method, address, prefix, gateway);

    return result;
}

const char* connection_ip_config(NMConnection* connection)
{
    NMSettingIPConfig * s_ip4;
    NMIPAddress *ip_address;
    const char *address, *gateway, *method;
    char* ip_config;
    int prefix;

    s_ip4 = nm_connection_get_setting_ip4_config(connection);
    method = nm_setting_ip_config_get_method(s_ip4);

    if (!strcmp(method, "manual")) {
        ip_address = nm_setting_ip_config_get_address(s_ip4, 0);
        prefix = nm_ip_address_get_prefix(ip_address);
        address = nm_ip_address_get_address(ip_address);
        gateway = nm_setting_ip_config_get_gateway(s_ip4);

        if (gateway) {
            asprintf(&ip_config, "%s %s %d %s", method, address, prefix, gateway);
        } else {
            asprintf(&ip_config, "%s %s %d", method, address, prefix);
        }
    } else {
        asprintf(&ip_config, "%s", method);
    }

    return ip_config;
}

/*
 * Detect NM device which has active connection and return device name and ip configuration of connection.
 * return value: "device_1 device_type_1 ip_config_1#device_2 ..."
 */
int get_ip_address(char *payload)
{
    NMDevice *device;
    NMActiveConnection *active_con;
    NMClient   *client;
    GError     *error = NULL;
    NMConnection *new_connection;
    const GPtrArray *devices;
    const char *device_name, *type, *ip_config;
    char *temp;
    int ret = 0;

    if (!payload) return -1;

    client = nm_client_new(NULL, &error);

    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return -1;
    }

    devices = nm_client_get_devices(client);

    for (int i = 0; i < devices->len; i++) {
        device = (NMDevice *)devices->pdata[i];
        active_con = nm_device_get_active_connection(device);
        device_name = nm_device_get_iface(device);

        if (!active_con) continue;

        new_connection = nm_simple_connection_new_clone(NM_CONNECTION(nm_active_connection_get_connection(active_con)));

        switch (nm_device_get_device_type(device)) {
            case NM_DEVICE_TYPE_ETHERNET:
                type = "ethernet";
                break;

            case NM_DEVICE_TYPE_WIFI:
                type = "wifi";
                break;
            default:
                continue;
        }

        ip_config = connection_ip_config(new_connection);
        asprintf(&temp, "%s %s %s", device_name, type, ip_config);

        if (strlen(payload)) strcat(payload, "#");

        strcat(payload, temp);
    }

    g_object_unref(client);

exit:
    return ret;
}

void ip_address_apply()
{
    NMClient  *client;
    NMDevice *device;
    GMainLoop *loop;
    GError    *error = NULL;
    WifiModifyData *wifi_modify_data;
    NMActiveConnection *active_con;
    NMRemoteConnection *rem_con = NULL;

    loop = g_main_loop_new(NULL, FALSE);
    client = nm_client_new(NULL, &error);

    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return;
    }

    device = nm_client_get_device_by_iface(client, current_device);

    if (!device) {
        g_print("No such device found, device name error!\n");
        g_object_unref(client);

        return;
    }

    active_con = nm_device_get_active_connection(device);

    if (!active_con) {
        printf("No current active connection on this device!\n");
        g_object_unref(client);

        return;
    }

    rem_con = nm_active_connection_get_connection(active_con);

    wifi_modify_data = g_slice_new(WifiModifyData);
    *wifi_modify_data = (WifiModifyData) {
        .client = client,
        .device = device,
        .loop = loop,
    };

    nm_device_reapply_async(wifi_modify_data->device, NM_CONNECTION (rem_con), 0, 0, NULL, reapply_cb, wifi_modify_data);

    g_main_loop_run(wifi_modify_data->loop);
    g_object_unref(wifi_modify_data->client);
}

void ip_reapply_daemon(void)
{
    if (reapply_required) {
        ip_address_apply();
        reapply_required = false;
    }
}

#define LED_NUM 0
static int locate_on = 0;
int set_locate(char *payload)
{
    if (!payload) return -1;

#ifdef SUPPORT_NLIB
    if (strcmp(payload, "on") == 0) {
        locate_on = 1;
    } else {
        locate_on = 0;
    }

    printf("set LED: %d\n", locate_on);
#else
    printf("locate is not supported.");
#endif /*SUPPORT_NLIB*/

    return 0;
}

int get_locate(char* payload)
{
    int ret = 0;

    if (!payload) return -1;

    if (locate_on) {
        sprintf(payload, "%s", "on");
    } else {
        sprintf(payload, "%s", "off");
    }

exit:
    return ret;
}

#ifdef SUPPORT_NLIB
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

#endif /*SUPPORT_NLIB*/

int get_domain_id(char *payload)
{
    int ret = 0;

    if (!payload) return -1;

    struct passwd *pw = getpwuid(getuid());
    char *config_dir = pw->pw_dir;
    strcat(config_dir, "/ros_menu/config.yaml");
    YAML::Node config = YAML::LoadFile(config_dir);
    strcat(payload, config["Config"]["default_ros_domain_id"].as < std::string > ().c_str());

exit:
    return ret;
}

int set_domain_id(char *payload)
{
    if (!payload) return -1;

    std::stringstream strValue;
    unsigned int id_num;
    strValue << payload;
    strValue >> id_num;

    if (id_num > 232) return -1;

    struct passwd *pw = getpwuid(getuid());
    char *config_dir = pw->pw_dir;
    strcat(config_dir, "/ros_menu/config.yaml");
    YAML::Node config = YAML::LoadFile(config_dir);

    config["Config"]["default_ros_domain_id"] = id_num;
    std::ofstream fout(config_dir);
    fout << config;

    return 0;
}

#ifdef SUPPORT_ROS
int get_node_list(char *payload)
{
    int ret = 0;

    if (!payload) return -1;

    auto node_list = node->get_node_graph_interface()->get_node_names_and_namespaces();
    for (auto & n : node_list) {
        strcat(payload, n.second.c_str());
        strcat(payload, n.first.c_str());
        strcat(payload, " ");
    }

exit:
    return ret;
}

#endif /*SUPPORT_ROS*/

static const char *RMT_TASK_DIR = "/opt/adlink/rmt/tasks/";
int get_task_list(char *payload)
{
    /* Recognize the tasks by reading the file names */
    if (!payload) return -1;

    strcat(payload, "Idle"); // Add Idle task for idle mode

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(RMT_TASK_DIR)) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            if ((strcmp(ent->d_name, ".") != 0) && (strcmp(ent->d_name, "..") != 0)) {
                strcat(payload, " ");           // add delimiter
                strcat(payload, ent->d_name);   // append new task file name
            }
        }
        closedir(dir);
    } else {
        /* could not open directory */
        perror ("");
        return -1;
    }

    printf("task_list: %s\n", payload);
    return 0;
}

static pid_t g_running_pid = 0; // record the running process id
static char g_running_task_name[32];

int check_pid_exists(pid_t pid)
{
    struct stat sts;
    char proc_path[32];

    snprintf(proc_path, sizeof(proc_path), "/proc/%d", pid);
    if ((stat(proc_path, &sts) == -1) && (errno == ENOENT)) {
        // process doesn't exist, return -1
        return -1;
    }
    return 0;
}

int get_task_mode(char *payload)
{
    /* Check current running task by PID */
    if (!payload) return -1;

    struct stat sts;
    char proc_path[32];

    if (g_running_pid == 0) {
        printf("task_mode: Idle\n");
        sprintf(payload, "Idle");
        return 0;
    }

    if (check_pid_exists(g_running_pid) < 0) {
        printf("Task '%s' PID=%d not found! Set to idle mode.\n", g_running_task_name, g_running_pid);
        g_running_pid = 0;
        snprintf(g_running_task_name, sizeof(g_running_task_name), "Idle");
    }

    printf("task_mode: %s\n", g_running_task_name);
    sprintf(payload, "%s", g_running_task_name);

    return 0;
}

static int run_task_script(char *filename)
{
    if (g_running_pid != 0) {
        /* There is already a running task, so we kill that */
        printf("[%d] Stop task '%s', PID=%d\n", getpid(), g_running_task_name, g_running_pid);
        kill(g_running_pid, SIGTERM); // send a signal to terminate current task
        g_running_pid = 0;
        snprintf(g_running_task_name, sizeof(g_running_task_name), "Idle");
    }

    /*
        RETURN VALUE of fork():
        On success, the PID of the child process is returned in the parent,
        and 0 is returned in the child.  On failure, -1 is returned in the parent,
        no child process is created, and errno is set appropriately.
     */
    // #define DISABLE_OUTPUT_MSG
    signal(SIGCHLD, SIG_IGN); // ignore the return value of child process
    if ((g_running_pid = fork()) < 0) {
        perror("fork"); // fork error
    } else if (g_running_pid == 0) {
#ifdef DISABLE_OUTPUT_MSG
        // in the child process, disable child messages output
        int fd_stdout_bak = dup(1); // backup stdout
        int fd_stderr_bak = dup(2); // backup stderr
        int fd = open("/dev/null", O_WRONLY | O_CREAT, 0666);
        dup2(fd, 1); // redirect stdout to /dev/null
        dup2(fd, 2); // redirect stderr to /dev/null
#endif
        // run external task program
        char fullpath[128];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", RMT_TASK_DIR, filename);
        // We need to make sure the child process run in different PGID (Here we use the same ID as child process PID).
        // Therefore we can kill all the child processes by sending signal to the PGID
        setpgid(getpid(), getpid());
        if (execl(fullpath, filename, (char *) NULL) < 0) {
            // error to run, enable stdout/stderror to show error reason
#ifdef DISABLE_OUTPUT_MSG
            dup2(fd_stdout_bak, 1); // restore stdout
            dup2(fd_stderr_bak, 2); // restore stderr
#endif
            perror(filename);       // show execl error
        }
#ifdef DISABLE_OUTPUT_MSG
        close(fd);
#endif
        exit(0); // child finished
    }

    // In parent process, check child pid exists, store the child pid in 'g_running_pid' and task name in 'g_running_task_name'
    sleep(1);
    if (check_pid_exists(g_running_pid) < 0) {
        // failure
        g_running_pid = 0;
        strncpy(g_running_task_name, "Idle", sizeof(g_running_task_name));
        printf("Failed to run task '%s'\n", filename);
    } else {
        // success
        strncpy(g_running_task_name, filename, sizeof(g_running_task_name));
        printf("Task '%s' (PID=%d) is running now.\n", g_running_task_name, g_running_pid);
    }
    return 0;
}

int set_task_mode(char *payload)
{
    /* Ask the robot to run the given task */
    char fpath[PATH_MAX];

    if (!payload) return -1;

    printf("task_mode to be set: %s\n", payload);

    if (strcmp(payload, g_running_task_name) == 0) {
        printf("Request is rejected because this task is already running.\n");
        return 0;
    } else if ((strcmp(payload, "Idle") == 0) && (g_running_pid != 0)) {
        printf("[%d] Stop the running task (%d) due to receive 'Idle' task mode.\n", getpid(), g_running_pid);
        kill(g_running_pid, SIGTERM); // send a signal to terminate current task
        g_running_pid = 0;
        snprintf(g_running_task_name, sizeof(g_running_task_name), "Idle");
        return 0;
    }

    snprintf(fpath, sizeof(fpath), "%s/%s", RMT_TASK_DIR, payload);
    if (access(fpath, F_OK) != 0) {
        // task file not found
        char absolute_path[PATH_MAX];
        char *ret = getcwd(absolute_path, sizeof(absolute_path));
        printf("Task '%s' not found in %s/%s\n", payload, absolute_path, RMT_TASK_DIR);
        return -1;
    } else if (access(fpath, X_OK) != 0) {
        // task file not executable
        printf("Task '%s' not executable! Please add execution permission to the task file.\n", payload);
        return -1;
    }

    printf("Ready to run task '%s'\n", payload);
    run_task_script(payload);

    return 0;
}
