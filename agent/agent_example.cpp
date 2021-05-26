#include <stdlib.h>
#include <stdio.h>
#include <unistd.h> // sleep
#include <getopt.h>
#include <string.h>
#include <time.h>
#include <glib.h>
#include <NetworkManager.h>
#include "mraa/led.h"
#include "rmt_agent.h"
#include "datainfo.hpp"
#include "fileinfo.hpp"
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <syslog.h>

static void skeleton_daemon()
{
    pid_t pid;

    /* Fork off the parent process */
    pid = fork();

    /* An error occurred */
    if (pid < 0)
        exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* On success: The child process becomes session leader */
    if (setsid() < 0)
        exit(EXIT_FAILURE);

    /* Catch, ignore and handle signals */
    /*TODO: Implement a working signal handler */
    signal(SIGCHLD, SIG_IGN);
    signal(SIGHUP, SIG_IGN);

    /* Fork off for the second time*/
    pid = fork();

    /* An error occurred */
    if (pid < 0)
        exit(EXIT_FAILURE);

    /* Success: Let the parent terminate */
    if (pid > 0)
        exit(EXIT_SUCCESS);

    /* Set new file permissions */
    umask(0);

    /* Change the working directory to the root directory */
    /* or another appropriated directory */
    chdir("/usr/");

    /* Close all open file descriptors */
    for (int x = sysconf(_SC_OPEN_MAX); x >= 0; x--) {
        close (x);
    }
}

static unsigned long myid = 0;
static char *my_interface = NULL;

static datainfo_func func_maps[] = {
    {"cpu",         get_cpu,        NULL         },
    {"ram",         get_ram,        NULL         },
    {"hostname",    get_hostname,   set_hostname },
    {"wifi",        get_wifi,       set_wifi     },
    {"locate",      NULL,           set_locate   },
    {"task_list",   get_task_list,  NULL         },
    {"task_mode",   get_task_mode,  set_task_mode},
    {0,             0,              0            },
};

static fileinfo_func file_maps[] = {
    {"custom_callback", "/tmp", import_testfile, export_testfile},
    {0,                 0,      0,               0              },
};

const char *short_options = "i:n:h";
struct option long_options[] = {
    {"id",     required_argument, NULL, 'i'},
    {"net",    required_argument, NULL, 'n'},
    {"daemon", no_argument,       NULL, 'd'},
    {"help",   no_argument,       NULL, 'h'},
    { 0,       0,                 0,    0  },
};

void print_help(void)
{
    printf("Usage: ./agent_example [options]\n");
    printf("* --help: Showing this messages.\n");
    printf("* --id [myID]: Use myID as the ID.\n");
    printf("* --net [interface]: Decide which interface agent uses.\n");
    printf("* --daemon: Run RMT agent as daemon.\n");
}

int main(int argc, char *argv[])
{
    int cmd_opt = 0;
    bool daemon = false;

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
            case 'd':
                daemon = true;
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

    if (daemon) {
        skeleton_daemon();
    }

    GError *   error = NULL;
    NMClient * client;

    client = nm_client_new(NULL, &error);
    if (!client) {
        g_message("Error: Could not connect to NetworkManager: %s.", error->message);
        g_error_free(error);
        return 1;
    }

    printf("This is RMT Agent. id=%lu and network interface=%s\n", myid, my_interface);
    rmt_agent_config(my_interface, myid);
    rmt_agent_init(func_maps, file_maps);
    mraa_init();

    if (!nm_client_get_connection_by_id(client, "RMTClient")) {
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
