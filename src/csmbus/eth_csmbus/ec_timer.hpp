#pragma once

#include <rclcpp/rclcpp.hpp>

namespace csmbus
{

class Timer
{
private:
    rclcpp::Clock _clock;
    rclcpp::Time _start;
    
public:
    Timer() : _clock(RCL_ROS_TIME){
        this->_start = this->_clock.now();
    }

    void reset(uint64_t ms = 0){
        this->_start = this->_clock.now() + std::chrono::milliseconds(ms);
    }

    rclcpp::Duration now(void){
        return (this->_clock.now() - this->_start);
    }

    double now_sec(void){
        return (this->_clock.now() - this->_start).seconds();
    }

    uint64_t now_ms(void){
        return (this->_clock.now() - this->_start).nanoseconds() / (uint64_t)1000000;
    }
};

class RealTimer
{
private:
    struct timespec _befor_tim;

    void timespec_now(struct timespec* tim){
        clock_gettime(CLOCK_REALTIME, tim);
    }

    void timespec_sub(const struct timespec* A, const struct timespec* B, struct timespec* res){
        res->tv_sec  = A->tv_sec  - B->tv_sec;
        if(A->tv_nsec < B->tv_nsec)
        {
            res->tv_sec--;
            res->tv_nsec = A->tv_nsec + (uint64_t)1e9;
            res->tv_nsec -= B->tv_nsec;
        }else{
            res->tv_nsec = A->tv_nsec - B->tv_nsec;
        }
    }

public:
    RealTimer(void){}

    void start(){
        timespec_now(&_befor_tim);
    }

    void reset()
    {
        this->start();
    }

    uint64_t getUs(void){
        struct timespec now;
        struct timespec diff;
        timespec_now(&now);
        timespec_sub(&now, &this->_befor_tim, &diff);
        uint64_t now_us = diff.tv_sec * (uint64_t)1e6;
        now_us += (diff.tv_nsec / (uint64_t)1e3);
        return now_us;
    }

    uint64_t getMs(void){
        struct timespec now;
        struct timespec diff;
        timespec_now(&now);
        timespec_sub(&now, &this->_befor_tim, &diff);
        uint64_t now_ms = diff.tv_sec * (uint64_t)1e3;
        now_ms += (diff.tv_nsec / (uint64_t)1e6);
        return now_ms;
    }

    double getSec(void){
        struct timespec now;
        struct timespec diff;
        timespec_now(&now);
        timespec_sub(&now, &this->_befor_tim, &diff);
        double now_sec = (double)diff.tv_sec;
        now_sec += (double)diff.tv_nsec / (double)1e9;
        return now_sec;
    }

    static void sleep_sec(float sec){
        struct timespec ts;
        ts.tv_sec = (time_t)sec;
        ts.tv_nsec = (uint64_t)((sec - (float)ts.tv_sec) * (float)1e9);
        nanosleep(&ts, NULL);
    }

    static void sleep_us(uint64_t us){
        struct timespec ts;
        ts.tv_sec = us / (uint64_t)1e6;
        ts.tv_nsec = (us % (uint64_t)1e6) * (uint64_t)1e3;
        nanosleep(&ts, NULL);
    }

    static void sleep_ms(uint64_t ms){
        struct timespec ts;
        ts.tv_sec = ms / (uint64_t)1e3;
        ts.tv_nsec = (ms % (uint64_t)1e3) * 1e6;
        nanosleep(&ts, NULL);
    }
};

}