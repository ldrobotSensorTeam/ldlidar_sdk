/**
* @file         log_module.h
* @author       David Hu (hmd_hubei_cn@163.com)
* @brief         
* @version      0.1
* @date         2022.08.10
* @note          
* @copyright    Copyright (c) 2022  DAVID HU All rights reserved.
* Licensed under the MIT License (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License in the file LICENSE
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**/

#ifndef  __LOGMODULE_H_
#define  __LOGMODULE_H_


#define ENABLE_LOG_DIS_OUTPUT

#define ENABLE_CONSOLE_LOG_DISPALY

//#define ENABLE_LOG_WRITE_TO_FILE

#include <stdio.h>
#include <string>
#include <chrono>
#include <stdlib.h>

#ifndef __linux__
#include <windows.h>
#else
//#include <pthread.h>
#include <stdarg.h>
#define printf_s(fileptr,str)  (fprintf(fileptr,"%s",str))
#define __in
#endif // ??????????????????????


struct LogVersion {
  int		       n_version;     
  std::string  str_descruble;
};


class ILogRealization {
public:
  virtual ~ILogRealization() {

  }
  virtual void Initializion(const char* path = NULL) = 0;
  virtual void LogPrintInf(const char* str) = 0;
  virtual void LogPrintData(const char* str) = 0;

  void free() {
    free(this);
    //this = NULL;
  };
private:
  virtual void free(ILogRealization *plogger) = 0 ;
};


#define  ILOGFREE(LogRealizationClass)  virtual void free(ILogRealization* plogger)	\
{																					\
    LogRealizationClass* prealization = static_cast<LogRealizationClass*>(plogger);    \
    if (prealization != NULL){ delete prealization;}								\
}

class LogPrint :public ILogRealization {
public:
  virtual void Initializion(const char* path = NULL);
  virtual void free(ILogRealization *plogger);
  virtual void LogPrintInf(const char* str);
  virtual void LogPrintData(const char* str);

  inline std::string GetLogFilePathName(void) {
    std::string curr_date_log_file;
    char stdtime_str[50] = {0};
    time_t std_time = 0;
    struct tm* local_time = NULL;
    std_time = time(NULL);
    local_time = localtime(&std_time);
    snprintf(stdtime_str, 50, "./lds-%d-%d-%d-%d.log", 
    local_time->tm_year+1900, local_time->tm_mon+1, local_time->tm_mday,
    local_time->tm_hour);
    curr_date_log_file.assign(stdtime_str);
    return curr_date_log_file;
  }

  inline std::string GetOriginDataFilePathName(void) {
    std::string curr_date_log_file;
    char stdtime_str[50] = {0};
    time_t std_time = 0;
    struct tm* local_time = NULL;
    std_time = time(NULL);
    local_time = localtime(&std_time);
    snprintf(stdtime_str, 50, "./serialdata-%d-%d-%d-%d.log", 
    local_time->tm_year+1900, local_time->tm_mon+1, local_time->tm_mday,
    local_time->tm_hour);
    curr_date_log_file.assign(stdtime_str);
    return curr_date_log_file;
  }
};

#ifndef __linux__
class LogOutputString :public ILogRealization {
public:
  virtual void Initializion(const char* path = NULL) {
    return ;
  }

  virtual void LogPrintInf(const char* str) {
    //OutputDebugString((LPCTSTR)str); // 将字符串发送到调试器进行显示。
    //OutputDebugString("\r\n");

	printf("%s\r\n", str); // 将字符串发送到控制台显示
  }

  virtual void LogPrintData(const char* str) {
    //OutputDebugString((LPCTSTR)str); // 将字符串发送到调试器进行显示。
    //OutputDebugString("\r\n");

	printf("%s\r\n", str); // 将字符串发送到控制台显示
  }

  ILOGFREE(LogOutputString)
/*
    virtual void free(ILogRealization *plogger)
    {
        LogOutputString* poutput = static_cast<LogOutputString*>(plogger);
        if (poutput != NULL)
        {
            delete poutput;
        }
    }
*/
};
#endif


class LogModule {
public:
  enum LogLevel {
    DEBUG_LEVEL,
    WARNING_LEVEL,
    ERROR_LEVEL,
    INFO_LEVEL
  };

  struct LOGMODULE_INFO {
    LogLevel	    loglevel;       
    std::string		str_filename; 
    std::string		str_funcname;  
    int			      n_linenumber;	  
  }logInfo_;

  ILogRealization* p_realization_; 
public:
  static  LogModule* GetInstance( __in const char* filename, __in const char* funcname,__in int lineno, LogLevel level, ILogRealization* plog = NULL);
  static  LogModule* GetInstance(LogLevel level, ILogRealization* plog = NULL);
  static  LogModule* GetInstancePrintOriginData(LogLevel level, ILogRealization* plog = NULL);

  void LogPrintInf(const char* format,...);
  void LogPrintNoLocationInf(const char* format,...);
  void LogPrintOriginData(const char* format,...);

private:
  LogModule();

  ~LogModule();

  void InitLock();

  void RealseLock();

  void Lock();

  void UnLock();

  std::string GetCurrentTime();

  std::string GetCurrentLocalTimeStamp();

  std::string GetFormatValue(std::string str_value);

  std::string  GetFormatValue(int n_value);

  std::string  GetLevelValue(int level);

  static LogModule*  s_plog_module_;

#ifndef __linux__
    CRITICAL_SECTION   mutex_lock_;
#else
    pthread_mutex_t    mutex_lock_;
#endif
};

//// 以下功能支持所处文件、函数、行号信息的打印
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LOG(level,format,...)   LogModule::GetInstance(__FILE__, __FUNCTION__, __LINE__,level)->LogPrintInf(format,__VA_ARGS__);
#define  LOG_DEBUG(format,...)   LOG(LogModule::DEBUG_LEVEL,format,__VA_ARGS__)
#define  LOG_INFO(format,...)    LOG(LogModule::INFO_LEVEL,format,__VA_ARGS__)
#define  LOG_WARN(format,...)    LOG(LogModule::WARNING_LEVEL,format,__VA_ARGS__)
#define  LOG_ERROR(format,...)   LOG(LogModule::ERROR_LEVEL,format,__VA_ARGS__)
#else
#define  LOG_DEBUG(format,...)   do {} while(0)
#define  LOG_INFO(format,...)    do {} while(0)
#define  LOG_WARN(format,...)    do {} while(0)
#define  LOG_ERROR(format,...)   do {} while(0)
#endif

//// 以下功能不支持所处文件、函数、行号信息的打印
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LOG_LITE(level,format,...)   LogModule::GetInstance(level)->LogPrintNoLocationInf(format,__VA_ARGS__);
#define  LOG_DEBUG_LITE(format,...)   LOG_LITE(LogModule::DEBUG_LEVEL,format,__VA_ARGS__)       
#define  LOG_INFO_LITE(format,...)    LOG_LITE(LogModule::INFO_LEVEL,format,__VA_ARGS__)        
#define  LOG_WARN_LITE(format,...)    LOG_LITE(LogModule::WARNING_LEVEL,format,__VA_ARGS__)     
#define  LOG_ERROR_LITE(format,...)   LOG_LITE(LogModule::ERROR_LEVEL,format,__VA_ARGS__)       
#else
#define  LOG_DEBUG_LITE(format,...)   do {} while(0)       
#define  LOG_INFO_LITE(format,...)    do {} while(0)        
#define  LOG_WARN_LITE(format,...)    do {} while(0)     
#define  LOG_ERROR_LITE(format,...)   do {} while(0)     
#endif

//// 等同于printf和fprintf的功能
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LOG_PRINT(level,format,...)   LogModule::GetInstancePrintOriginData(level)->LogPrintOriginData(format,__VA_ARGS__);
#define  LOG_DEBUG_PRINT(format,...)   LOG_PRINT(LogModule::DEBUG_LEVEL,format,__VA_ARGS__)       
#define  LOG_INFO_PRINT(format,...)    LOG_PRINT(LogModule::INFO_LEVEL,format,__VA_ARGS__)     
#else
#define  LOG_DEBUG_PRINT(format,...)   do {} while(0)       
#define  LOG_INFO_PRINT(format,...)    do {} while(0)          
#endif

#endif//__LOGGER_MODULE_H__
/********************* (C) COPYRIGHT DAVID HU *******END OF FILE ********/
