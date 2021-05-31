#ifndef _DATAINFO_HPP_
#define _DATAINFO_HPP_

#include <rclcpp/rclcpp.hpp>

extern char interface[50];
extern rclcpp::Node::SharedPtr node;

int get_cpu(char *payload);
int get_ram(char *payload);
int get_hostname(char *payload);
int set_hostname(char *payload);
int get_wifi(char *payload);
int set_wifi(char *payload);
int set_locate(char *payload);
int get_domain_id(char *payload);
int set_domain_id(char *payload);
int get_node_list(char *payload);
int get_task_list(char *payload);
int get_task_mode(char *payload);
int set_task_mode(char *payload);

void add_wifi_connection(NMClient *client);
void locate_daemon(void);

#endif /*_DATAINFO_HPP_*/
