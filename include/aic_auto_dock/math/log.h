#ifndef LOG_H
#define LOG_H

#include <fstream>
#include <iostream>
#include <log4cxx/logger.h>
#include <log4cxx/propertyconfigurator.h>
#include <ros/ros.h>
#include <string>
#include <stdio.h>

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string YELLOW = "3m";
const std::string BLUE = "4m";
const std::string WHITE = "7m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

using namespace std;

#define TRACE_LOG(p_msg)                                                                                               \
  LOG4CXX_TRACE(log4cxx::Logger::getLogger("XXX_LOG"),                                                                 \
                REGULAR << GREEN << "[TRACE] [" << ros::WallTime::now() << ", " << ros::Time::now() << "]: " << p_msg  \
                        << RESET)

#define DEBUG_LOG(p_msg)                                                                                               \
  LOG4CXX_DEBUG(log4cxx::Logger::getLogger("XXX_LOG"),                                                                 \
                REGULAR << GREEN << "[DEBUG] [" << ros::WallTime::now() << ", " << ros::Time::now() << "]: " << p_msg  \
                        << RESET)

#define INFO_LOG(p_msg)                                                                                                \
  LOG4CXX_INFO(log4cxx::Logger::getLogger("XXX_LOG"),                                                                  \
               REGULAR << WHITE << "[ INFO] [" << ros::WallTime::now() << ", " << ros::Time::now() << "]: " << p_msg    \
                       << RESET)

#define WARN_LOG(p_msg)                                                                                                \
  LOG4CXX_WARN(log4cxx::Logger::getLogger("XXX_LOG"),                                                                  \
               REGULAR << YELLOW << "[ WARN] [" << ros::WallTime::now() << ", " << ros::Time::now() << "]: " << p_msg   \
                       << RESET)

#define ERROR_LOG(p_msg)                                                                                               \
  LOG4CXX_ERROR(log4cxx::Logger::getLogger("XXX_LOG"),                                                                 \
                REGULAR << RED << "[ERROR] [" << ros::WallTime::now() << ", " << ros::Time::now() << "]: " << p_msg    \
                        << RESET)

#define FATAL_LOG(p_msg)                                                                                               \
  LOG4CXX_FATAL(log4cxx::Logger::getLogger("XXX_LOG"),                                                                 \
                REGULAR << RED << "[FATAL] [" << ros::WallTime::now() << ", " << ros::Time::now() << "]: " << p_msg    \
                        << RESET)

#define XXX_LOG_PATH "/home/aicrobo/.aiclog/autodock/logs"
#define XXX_CFG_PATH "/home/aicrobo/.aicconfig/autodock/log.properties"

inline void loginit()
{
  std::ifstream fin(XXX_CFG_PATH);
  if(!fin)
  {
    ROS_WARN("no log preperties file");

    std::ofstream openfile(XXX_CFG_PATH, std::ios::trunc);
    openfile << "# 设置root logger为DEBUG级别，使用了ca和fa两个Appender" << std::endl;
    openfile << "log4j.rootLogger=DEBUG, ca, XXX_LOG" << std::endl;
    openfile << std::endl;
    openfile << "#对Appender fa进行设置：" << std::endl;
    openfile << "# 这是一个文件类型的Appender，" << std::endl;
    openfile << "# 其输出文件（File）为./output.log，" << std::endl;
    openfile << "# 输出方式（Append）为覆盖方式，" << std::endl;
    openfile << "# 输出格式（layout）为PatternLayout" << std::endl;
    openfile << "log4j.appender.XXX_LOG=org.apache.log4j.DailyRollingFileAppender" << std::endl;
    openfile << "log4j.appender.XXX_LOG.ImmediateFlush=true" << std::endl;
    openfile << "log4j.appender.XXX_LOG.DatePattern='" << XXX_LOG_PATH << "/'yyyy-MM-dd'.log'" << std::endl;
    openfile << "log4j.appender.XXX_LOG.Append=true" << std::endl;
    openfile << "log4j.appender.XXX_LOG.Threshold=DEBUG" << std::endl;
    openfile << "log4j.appender.XXX_LOG.layout=org.apache.log4j.PatternLayout" << std::endl;
    openfile << "log4j.appender.XXX_LOG.layout.ConversionPattern=%d %5p (%F:%L) - %m%n" << std::endl;
    openfile << "log4j.appender.XXX_LOG.encoding=UTF-8" << std::endl;
    openfile << std::endl;
    openfile << "#对Appender ca进行设置" << std::endl;
    openfile << "# 这是一个控制台类型的Appender" << std::endl;
    openfile << "# 输出格式（layout）为PatternLayout" << std::endl;
    openfile << "log4j.appender.ca=org.apache.log4j.ConsoleAppender" << std::endl;
    openfile << "log4j.appender.ca.layout=org.apache.log4j.PatternLayout" << std::endl;
    openfile << "log4j.appender.ca.layout.ConversionPattern=%m%n" << std::endl;
    openfile << "log4j.appender.ca.encoding=UTF-8" << std::endl;
    openfile.close();
  }

  setlocale(LC_ALL, "");
  // 配置全局日志器
  log4cxx::PropertyConfigurator::configure(XXX_CFG_PATH);
}

#endif // LOG_H
