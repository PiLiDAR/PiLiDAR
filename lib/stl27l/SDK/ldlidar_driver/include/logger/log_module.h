/**
* @file         ldlidar_logger.h
* @author       LDRobot (support@ldrobot.com)
* @brief         
* @version      0.1
* @date         2022.08.10
* @note          
* @copyright    Copyright (c) 2022  SHENZHEN LDROBOT CO., LTD. All rights reserved.
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


#define LINUX

#define ENABLE_LOG_DIS_OUTPUT

#define ENABLE_CONSOLE_LOG_DIS

//#define ENABLE_LOG_WRITE_TO_FILE

#define LOGFILEPATH "./ldlidar-driver.log"

#include <stdio.h>
#include <string>
#include <chrono>
#include <stdlib.h>

#ifndef LINUX
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
};

#ifndef LINUX
class LogOutputString :public ILogRealization {
public:
  virtual void Initializion(const char* path = NULL) {
    return ;
  }

  virtual void LogPrintInf(const char* str) {
    OutputDebugString((LPCTSTR)str);
    OutputDebugString("\r\n");
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
  static  LogModule* GetInstance( __in const char* filename, __in const char* funcname,__in int lineno, LogLevel level, ILogRealization*plog = NULL);
  static  LogModule* GetInstance(LogLevel level, ILogRealization*plog = NULL);

  void LogPrintInf(const char* format,...);
  void LogPrintNoLocationInf(const char* format,...);

private:
  LogModule();

  ~LogModule();

  void InitLock();

  void RealseLock();

  void Lock();

  void UnLock();

  std::string GetCurrentTime();

  inline uint64_t GetCurrentLocalTimeStamp() {
    //// 获取系统时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
      std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
    auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
    return (uint64_t)tmp.count();
  }

  std::string GetFormatValue(std::string str_value);

  std::string  GetFormatValue(int n_value);

  std::string  GetLevelValue(int level);

  static LogModule*  s_plog_module_;

#ifndef LINUX
    CRITICAL_SECTION   mutex_lock_;
#else
    pthread_mutex_t    mutex_lock_;
#endif


};

//// 以下功能支持所处文件、函数、行号信息的打印
#define  LOG(level,format,...)   LogModule::GetInstance(__FILE__, __FUNCTION__, __LINE__,level)->LogPrintInf(format,__VA_ARGS__);
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LD_LOG_DEBUG(format,...)   LOG(LogModule::DEBUG_LEVEL,format,__VA_ARGS__)
#define  LD_LOG_INFO(format,...)    LOG(LogModule::INFO_LEVEL,format,__VA_ARGS__)
#define  LD_LOG_WARN(format,...)    LOG(LogModule::WARNING_LEVEL,format,__VA_ARGS__)
#define  LD_LOG_ERROR(format,...)   LOG(LogModule::ERROR_LEVEL,format,__VA_ARGS__)
#else
#define  LD_LOG_DEBUG(format,...)   do {} while(0)
#define  LD_LOG_INFO(format,...)    do {} while(0)
#define  LD_LOG_WARN(format,...)    do {} while(0)
#define  LD_LOG_ERROR(format,...)   do {} while(0)
#endif
//// 以下功能不支持所处文件、函数、行号信息的打印
#ifdef ENABLE_LOG_DIS_OUTPUT
#define  LOG_NO_DESCRI(level,format,...)   LogModule::GetInstance(level)->LogPrintNoLocationInf(format,__VA_ARGS__);
#define  LDS_LOG_DEBUG(format,...)   LOG_NO_DESCRI(LogModule::DEBUG_LEVEL,format,__VA_ARGS__)       
#define  LDS_LOG_INFO(format,...)    LOG_NO_DESCRI(LogModule::INFO_LEVEL,format,__VA_ARGS__)        
#define  LDS_LOG_WARN(format,...)    LOG_NO_DESCRI(LogModule::WARNING_LEVEL,format,__VA_ARGS__)     
#define  LDS_LOG_ERROR(format,...)   LOG_NO_DESCRI(LogModule::ERROR_LEVEL,format,__VA_ARGS__)       
#else
#define  LDS_LOG_DEBUG(format,...)   do {} while(0)       
#define  LDS_LOG_INFO(format,...)    do {} while(0)        
#define  LDS_LOG_WARN(format,...)    do {} while(0)     
#define  LDS_LOG_ERROR(format,...)   do {} while(0)     
#endif

#endif//__LDLIDAR_LOGGER_H__
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF FILE ********/