#ifndef FOOTPRINT_H
#define FOOTPRINT_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

std::vector< geometry_msgs::Point > makeFootprintFromParams(ros::NodeHandle&);
bool makeFootprintFromString(const std::string&, std::vector< geometry_msgs::Point >&);
void writeFootprintToParam(ros::NodeHandle&, const std::vector< geometry_msgs::Point >&);
std::vector< geometry_msgs::Point > makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue&, const std::string&);
std::vector<geometry_msgs::Point> makeFootprintFromRadius(double);
double getNumberFromXMLRPC(XmlRpc::XmlRpcValue&, const std::string&);
std::vector<std::vector<float> > parseVVF(const std::string&, std::string&);

#endif // MATH_H
