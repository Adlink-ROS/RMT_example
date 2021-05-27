#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <glib.h>
#include <NetworkManager.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include "mraa/led.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <fstream>
#include <pwd.h>
#include "yaml-cpp/yaml.h"

char interface[50];
rclcpp::Node::SharedPtr node;

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
    NMDevice *device;
    NMActiveConnection *active_con;
    NMConnection *new_connection;
    const char *   con_id;
    const char *   password = NULL;
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

void modify_connection_cb (GObject *connection, GAsyncResult *result, gpointer user_data)
{
    GError *error = NULL;

    if (!nm_remote_connection_commit_changes_finish (NM_REMOTE_CONNECTION (connection),
                                                     result, &error)) {
        printf(("Error: Failed to modify connection '%s': %s"),
               nm_connection_get_id (NM_CONNECTION (connection)),
               error->message);
    } else {
        printf(("Connection '%s' (%s) successfully modified.\n"),
               nm_connection_get_id (NM_CONNECTION (connection)),
               nm_connection_get_uuid (NM_CONNECTION (connection)));
    }
    g_main_loop_quit((GMainLoop*)user_data);
}

void modify_wifi(const char * ssid, const char * password)
{
    NMClient  *client;
    GMainLoop *loop;
    GError    *error = NULL;

    loop = g_main_loop_new(NULL, FALSE);
    client = nm_client_new(NULL, &error);

    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return;
    }

    NMRemoteConnection *rem_con = NULL;
    NMConnection       *connection = NULL;
    const char         *uuid;
    const char         *con_id = "RMTClient";
    gboolean temporary = FALSE;


    rem_con = nm_client_get_connection_by_id(client, con_id);
    GString* g_ssid = g_string_new(ssid);
    uuid = nm_setting_connection_get_uuid(nm_connection_get_setting_connection(NM_CONNECTION (rem_con)));

    connection = get_client_nmconnection(con_id, uuid, g_ssid, password);

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

static const char *RMT_TASK_DIR = "neuronbot2_tasks";
int get_task_list(char *payload)
{
    /* Recognize the tasks by reading the file names */

    DIR *dir;
    struct dirent *ent;
    bool need_delimit = false;

    if ((dir = opendir(RMT_TASK_DIR)) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            if ((strcmp(ent->d_name, ".") != 0) && (strcmp(ent->d_name, "..") != 0)) {
                if (need_delimit)
                    strcat(payload, " ");

                // append new task file name
                strcat(payload, ent->d_name);
                need_delimit = true;
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
        printf("Stop task '%s', PID=%d\n", g_running_task_name, g_running_pid);
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
    signal (SIGCHLD, SIG_IGN); // ignore the return value of child process
    if ((g_running_pid = fork()) < 0) {
        perror("fork"); // fork error
    } else if (g_running_pid == 0) {
        // in the child process, disable child messages output
        int fd_stdout_bak = dup(1); // backup stdout
        int fd_stderr_bak = dup(2); // backup stderr
        int fd = open("/dev/null", O_WRONLY | O_CREAT, 0666);
        dup2(fd, 1); // redirect stdout to /dev/null
        dup2(fd, 2); // redirect stderr to /dev/null

        // run external task program
        char fullpath[128];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", RMT_TASK_DIR, filename);
        if (execl(fullpath, filename, (char *) NULL) < 0) {
            // error to run, enable stdout/stderror to show error reason
            dup2(fd_stdout_bak, 1); // restore stdout
            dup2(fd_stderr_bak, 2); // restore stderr
            perror(filename);       // show execl error
        }
        close(fd);
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
        printf("Stop the running task due to receive 'Idle' task mode.\n");
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
