#ifndef _DATAINFO_HPP_
#define _DATAINFO_HPP_

#ifdef SUPPORT_ROS
 #include <rclcpp/rclcpp.hpp>
#endif /*SUPPORT_ROS*/

extern char interface[50];
#ifdef SUPPORT_ROS
extern rclcpp::Node::SharedPtr node;
#endif /*SUPPORT_ROS*/

int get_cpu(char *payload);
int get_ram(char *payload);
int get_hostname(char *payload);
int set_hostname(char *payload);
int get_wifi(char *payload);
int set_wifi(char *payload);
int get_locate(char* payload);
int set_locate(char *payload);
int get_domain_id(char *payload);
int set_domain_id(char *payload);
int get_ip_address(char *payload);
int set_ip_address(char *payload);
void ip_reapply_daemon(void);
#ifdef SUPPORT_ROS
int get_node_list(char *payload);
#endif /*SUPPORT_ROS*/
int get_task_list(char *payload);
int get_task_mode(char *payload);
int set_task_mode(char *payload);

void add_wifi_connection(NMClient *client);
#ifdef SUPPORT_NLIB
void locate_daemon(void);
#endif /*SUPPORT_NLIB*/

#endif /*_DATAINFO_HPP_*/
