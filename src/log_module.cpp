/**
* @file         log_module.cpp
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
#include "ldlidar_driver/log_module.h"

#include <time.h>
#include <string.h>

#ifndef __linux__
#include <comutil.h>  
#pragma comment(lib, "comsuppw.lib")
#else
#include <stdlib.h>
#endif

/* 使用vswprintf会出现奔溃的情况如果，传入数据大于 VA_PARAMETER_MAX 就会出现崩溃 */
#define  VA_PARAMETER_MAX  (1024 * 2)

LogModule* LogModule::s_plog_module_ = NULL;


LogModule* LogModule::GetInstance(__in const char* filename, __in const char* funcname, __in int lineno,LogLevel level,ILogRealization* plog) {
	if (s_plog_module_ == NULL) {
		s_plog_module_ = new LogModule();
	}
	s_plog_module_->logInfo_.str_filename = filename;
	s_plog_module_->logInfo_.str_funcname = funcname;
	s_plog_module_->logInfo_.n_linenumber = lineno;
	s_plog_module_->logInfo_.loglevel = level;
	
	if (plog != NULL) {
		s_plog_module_->p_realization_->free();
		s_plog_module_->p_realization_ = plog;
	}
	return s_plog_module_;
}

LogModule* LogModule::GetInstance(LogLevel level, ILogRealization* plog) {
	if (s_plog_module_ == NULL) {
		s_plog_module_ = new LogModule();
	}
	s_plog_module_->logInfo_.loglevel = level;
	
	if (plog != NULL) {
		s_plog_module_->p_realization_->free();
		s_plog_module_->p_realization_ = plog;
	}
	return s_plog_module_;
}

LogModule* LogModule::GetInstancePrintOriginData(LogLevel level, ILogRealization* plog) {
  if (s_plog_module_ == NULL) {
		s_plog_module_ = new LogModule();
	}
	s_plog_module_->logInfo_.loglevel = level;
	
	if (plog != NULL) {
		s_plog_module_->p_realization_->free();
		s_plog_module_->p_realization_ = plog;
	}
	return s_plog_module_;
}

LogModule::LogModule() {
	logInfo_.n_linenumber = -1;
	logInfo_.str_filename = "";
	logInfo_.str_funcname = "";
#ifndef __linux__
	p_realization_ = new LogOutputString();
#else
	p_realization_ = new LogPrint();
#endif
	InitLock();
}

LogModule::~LogModule() {
	RealseLock();
}

void LogModule::LogPrintInf(const char* format,...) {
	Lock();
	if (p_realization_) {
		std::string str_temp;
		str_temp.append("[LOG]");
		// Time   [week month day hours:minutes:seconds year]
		str_temp.append(GetFormatValue(GetCurrentISOTime()));
        // Stamp uint is ns
		str_temp.append(GetCurrentLocalTimeStamp());
		// LogLevel
		str_temp.append(GetLevelValue(logInfo_.loglevel));
		// File name
		str_temp.append(GetFormatValue(logInfo_.str_filename));
		// Function name
		str_temp.append(GetFormatValue(logInfo_.str_funcname));
		// Line number
		str_temp.append(GetFormatValue(logInfo_.n_linenumber));

		va_list ptr;
		va_start(ptr, format);
		char c_value[VA_PARAMETER_MAX] = {0};
		vsnprintf(c_value,sizeof(c_value),format,ptr);
		va_end(ptr);

		str_temp.append(GetFormatValue(c_value));

		p_realization_->LogPrintInf(str_temp.c_str());
	}
	UnLock();
}

void LogModule::LogPrintNoLocationInf(const char* format,...) {
	Lock();
	if (p_realization_) {
		std::string str_temp;
		// manufacture
		str_temp.append("[LOG]");
		// time   [week month day hours:minutes:seconds year]
		str_temp.append(GetFormatValue(GetCurrentISOTime()));
        // stamp  uint is ns
		str_temp.append(GetCurrentLocalTimeStamp());
		//LogLevel
		str_temp.append(GetLevelValue(logInfo_.loglevel));

		va_list ptr;
		va_start(ptr, format);
		char c_value[VA_PARAMETER_MAX] = {0};
		vsnprintf(c_value,sizeof(c_value),format,ptr);
		va_end(ptr);

		str_temp.append(GetFormatValue(c_value));

		p_realization_->LogPrintInf(str_temp.c_str());
	}
	UnLock();
}

void LogModule::LogPrintOriginData(const char* format,...) {
	Lock();
	if (p_realization_) {
		std::string str_temp;

		switch (logInfo_.loglevel) {
      case INFO_LEVEL: {
				str_temp.append("[LOG]");
				str_temp.append(GetFormatValue(GetCurrentISOTime()));
				str_temp.append(GetCurrentLocalTimeStamp());
				str_temp.append(GetLevelValue(logInfo_.loglevel));
				break;
			}
			default: {
				break;
			}
		}
		
		va_list ptr;
		va_start(ptr, format);
		char c_value[VA_PARAMETER_MAX] = {0};
		vsnprintf(c_value,sizeof(c_value),format,ptr);
		va_end(ptr);

		str_temp.append(c_value);

		p_realization_->LogPrintData(str_temp.c_str());
	}
	UnLock();
}

void LogModule::InitLock() {
#ifndef __linux__
	InitializeCriticalSection(&mutex_lock_);
#else
  pthread_mutex_init(&mutex_lock_,NULL);
#endif
}

void LogModule::RealseLock() {
#ifndef __linux__
	DeleteCriticalSection(&mutex_lock_);
#else
	pthread_mutex_unlock(&mutex_lock_);
#endif
}

void LogModule::Lock() {
#ifndef __linux__
	EnterCriticalSection(&mutex_lock_);
#else
	pthread_mutex_lock(&mutex_lock_);
#endif
}

void LogModule::UnLock() {
#ifndef __linux__
	LeaveCriticalSection(&mutex_lock_);
#else
	pthread_mutex_unlock(&mutex_lock_);
#endif
}

std::string LogModule::GetCurrentISOTime() {
	std::string curr_time;
#if 0
	//Current date/time based on current time
	time_t now = time(0); 
	// Convert current time to string
	curr_time.assign(ctime(&now));
	// Last charactor of currentTime is "\n", so remove it
	std::string current_time = curr_time.substr(0, curr_time.size()-1);
	return current_time;
#else
  char stdtime_str[50] = {0};
	time_t std_time = 0;
	struct tm* local_time = NULL;
	std_time = time(NULL);
	local_time = localtime(&std_time);
	snprintf(stdtime_str, 50, "%d-%d-%d,%d:%d:%d", 
	local_time->tm_year+1900, local_time->tm_mon+1, local_time->tm_mday,
	local_time->tm_hour, local_time->tm_min, local_time->tm_sec);
	curr_time.assign(stdtime_str);
  return curr_time;
#endif
}

std::string LogModule::GetCurrentLocalTimeStamp() {
	std::string stamp_str;
	std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
		std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
	uint64_t timestamp = (uint64_t)tmp.count();
	char s_stamp[100] = {0};
#ifdef __LP64__
	snprintf(s_stamp, 100, "[%lu.%lu]", (timestamp/1000000000), (timestamp%1000000000));
#else
#ifdef _WIN64
	snprintf(s_stamp, 100, "[%lu.%lu]", (timestamp/1000000000), (timestamp%1000000000));
#else
	snprintf(s_stamp, 100, "[%llu.%llu]", (timestamp/1000000000), (timestamp%1000000000));
#endif
#endif
	stamp_str = s_stamp;

	return stamp_str;
}

std::string LogModule::GetFormatValue(std::string str_value) {
	std::string str_temp;
	str_temp.append("[");
	str_temp.append(str_value);
	str_temp.append("]");
	return str_temp;
}

std::string LogModule::GetFormatValue(int n_value) {
	std::string str_temp;
	str_temp.append("[");
	char c_value[16];
	sprintf(c_value,"%d",n_value);
	str_temp.append(c_value);
	str_temp.append("]");
	return str_temp;
}

std::string  LogModule::GetLevelValue(int level){
	std::string tmp;
	switch (level) {
	case DEBUG_LEVEL:
		tmp = "DEBUG";
		break;
	case WARNING_LEVEL:
		tmp = "WARN";
		break;
	case ERROR_LEVEL:
		tmp = "ERROR";
		break;
	case INFO_LEVEL:
		tmp = "INFO";
		break;
	default:
		tmp = "UnKnown";
		break;
	}
	std::string str_temp;
	str_temp.append("[");
	str_temp.append(tmp);
	str_temp.append("]");
	return str_temp;
}

void LogPrint::Initializion(const char* path) {
	printf("%s", path);
	return ;
}

void LogPrint::free(ILogRealization *plogger) {
	LogPrint* pOutput = static_cast<LogPrint*>(plogger);
	if (pOutput != NULL) {
		delete pOutput;
	}
}

void LogPrint::LogPrintInf(const char* str) {
#ifdef ENABLE_CONSOLE_LOG_DISPALY
	printf("%s\r\n", str);
#endif

#ifdef ENABLE_LOG_WRITE_TO_FILE
  std::string log_filename = GetLogFilePathName();
	FILE *fp = fopen(log_filename.c_str() ,"a");
	if(!fp) {
		printf("%s open filed!\n", log_filename.c_str());
		return ;
	}
	printf_s(fp,str);
	printf_s(fp,"\r\n");
	fclose(fp);
#endif
}

void LogPrint::LogPrintData(const char* str) {
#ifdef ENABLE_CONSOLE_LOG_DISPALY
	printf("%s", str);
#endif

#ifdef ENABLE_LOG_WRITE_TO_FILE
  std::string log_filename = GetOriginDataFilePathName();
	FILE *fp = fopen(log_filename.c_str() ,"a");
	if(!fp) {
		printf("%s open filed!\n", log_filename.c_str());
		return ;
	}
	printf_s(fp,str);
	fclose(fp);
#endif
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/