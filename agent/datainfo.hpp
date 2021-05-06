#ifndef _DATAINFO_HPP_
#define _DATAINFO_HPP_

extern char interface[50];

int get_cpu(char *payload);
int get_ram(char *payload);
int get_hostname(char *payload);
int set_hostname(char *payload);
int get_wifi(char *payload);
int set_wifi(char *payload);
int set_locate(char *payload);

void add_wifi_connection(NMClient *client);
void locate_daemon(void);

#endif /*_DATAINFO_HPP_*/