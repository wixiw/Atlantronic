/*
 * Logger.hpp
 *
 *  Created on: 29 February 2012
 *      Author: Boris
 */

#ifndef _ARP_CORE_LOGGER_HPP_
#define _ARP_CORE_LOGGER_HPP_


/**
 * Micro wrapper of log4cpp system
 * This is a header-only micro lib to facilitate integration
 */

/*! Sample usage:

  arp_core::log::Logger * logger = getLogger(); // Usually created elsewhere
  //arp_core::log::Logger * logger = new arp_core::log::SimpleLogger(); // For instance

  arp_core::log::Category * cat = logger->create("xcd.cddif"); // Create a category
  arp_core::log::Category& c = *cat; // We can use a reference also (cleaner syntax)
  using namespace arp_core::log;

  // Different ways to invoke the logger
  cat->log(arp_core::log::INFO, "kikooo");
 *cat << arp_core::log::INFO << "kikoo";
  c << INFO << "kikoo " << 21;
  c.log(INFO, "test");
  c.log(WARN, "test %s %d", "pouet", 42);

 */

// TODO: We can include the realtime versions of string and ostringstream from RTT

#include <sstream>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <fstream>

//#ifdef ERROR
//#undef ERROR
//#endif

namespace arp_core
{
namespace log
{

/**
 * Predefined Levels of Priorities. These correspond to the
 * priority levels used by syslog(3).
 **/
typedef enum {
    FATAL,
    EMERG,
    ALERT,
    CRIT,
    ERROR,
    WARN,
    NOTICE,
    INFO,
    DEBUG,
    NOTSET
} PriorityLevel;


struct Priority {
        typedef int Value;
};

class Category;

class CategoryStream
{
    public:
        CategoryStream(Category* category, Priority::Value priority) : _category(category), _priority(priority) {}
        CategoryStream(const CategoryStream & cs) : _category(cs._category), _priority(cs._priority)
        {
            (*this).oss.str(cs.oss.str());
        }
        virtual ~CategoryStream() { flush(); }

        inline void flush();
        inline CategoryStream & eol(CategoryStream& os);

        template<typename T> CategoryStream& operator<<(const T& t)
        {
            if (_priority != NOTSET) { (oss) << t; }
            return *this;
        }
    private:
        Category * _category;
        Priority::Value _priority;
        std::ostringstream oss;
};

class Category
{
    public:
        //static Category& getInstance(const std::string& name);
        //static Category* exists(const std::string& name);
        virtual ~Category() {};
        virtual const std::string& getName() const throw() = 0;
        virtual void setName(const std::string) throw() = 0;
        virtual void setLogLevel(Priority::Value level) throw() = 0;

        /**
         * Log a message with the specified priority.
         * @param priority The priority of this log message.
         * @param stringFormat Format specifier for the string to write
         * in the log file.
         * @param ... The arguments for stringFormat
         **/
        virtual void log(Priority::Value priority, const char* stringFormat,
                ...) throw() = 0;

        /**
         * Log a message with the specified priority.
         * @param priority The priority of this log message.
         * @param message string to write in the log file
         **/
        virtual void log(Priority::Value priority,
                const std::string& message) throw() = 0;

        /**
         * Log a message with the specified priority.
         * @param priority The priority of this log message.
         * @param stringFormat Format specifier for the string to write
         * in the log file.
         * @param va The arguments for stringFormat.
         **/
        virtual void logva(Priority::Value priority,
                const char* stringFormat,
                va_list va) throw() = 0;

        /**
         * Return a CategoryStream with given Priority.
         * @param priority The Priority of the CategoryStream.
         * @returns The requested CategoryStream.
         **/
        virtual CategoryStream getStream(Priority::Value priority) { return CategoryStream(this, priority); }

        /**
         * Return a CategoryStream with given Priority.
         * @param priority The Priority of the CategoryStream.
         * @returns The requested CategoryStream.
         **/
        virtual CategoryStream operator<<(Priority::Value priority) { return CategoryStream(this, priority); }

    protected:
        static const char* priorityLevelNames[10];

};

inline void CategoryStream::flush()
{
    _category->log(_priority, oss.str());
    oss.flush();
}

inline CategoryStream & CategoryStream::eol(CategoryStream& os)
{
    os.flush();
    return os;
}

class NullCategory : public Category
{
    std::string null;
    public:
    virtual ~NullCategory() {};
    virtual const std::string& getName() const throw() { return null; };
    virtual void setName(const std::string) throw() { ; };
    virtual void setLogLevel(Priority::Value level) throw() { ; };

    virtual void log(Priority::Value priority, const char* stringFormat,
            ...) throw() {};
    virtual void log(Priority::Value priority,
            const std::string& message) throw() {};
    virtual void logva(Priority::Value priority,
            const char* stringFormat,
            va_list va) throw() {};
};

class SimpleStreamCategory : public Category
{
        std::ostream & os;
        std::string name;
        Priority::Value logLevel;
    public:
        SimpleStreamCategory(std::ostream &s, const std::string n = "", Priority::Value level = DEBUG)
        : os(s)
        , name(n)
        , logLevel(level)
        {
        };
        virtual ~SimpleStreamCategory() { };
        virtual const std::string& getName() const throw() { return name; };
        virtual void setName(const std::string n) throw() { name = n; };
        virtual void setLogLevel(Priority::Value level) throw() {logLevel = level;};

        void print(int p, const char * m)
        {
            if( p <= logLevel)
            {
                os << name << "(" << priorityLevelNames[p] << ") - " << m << std::endl;
            }
        }

        virtual void log(Priority::Value priority, const char* stringFormat,
                ...) throw()
                {
            va_list vl;
            va_start(vl, stringFormat);
            logva(priority, stringFormat, vl);
                };
        virtual void log(Priority::Value priority,
                const std::string& message) throw()
                {
            print(priority, message.c_str());
                };
        virtual void logva(Priority::Value priority,
                const char* stringFormat,
                va_list va) throw()
                {
            char buffer[256]; // XXX
            vsnprintf(buffer, 255, stringFormat, va);
            print(priority, buffer);
                };
};

class Logger
{
    public:
        virtual Category * create(const std::string & name) = 0;
};

class SimpleLogger : public Logger
{
    public:
        virtual Category * create(const std::string & name) { return new SimpleStreamCategory(std::cout, name); };
};

class FileLogger : public Logger
{
    public:
        virtual Category * create(const std::string & name)
        {
            std::ofstream * myfile = new std::ofstream();
            std::string fileName = "./" + name + ".log";
            myfile->open(fileName.c_str());
            return new SimpleStreamCategory(*myfile, name);
        };
};

class NullLogger : public Logger
{
    public:
        virtual Category * create(const std::string & name) { return new NullCategory(); };
};

}
}

#endif //_ARP_CORE_LOGGER_HPP_
