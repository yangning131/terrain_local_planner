#ifndef PLANNING_COMMON_THREAD_MUTEX_H_
#define PLANNING_COMMON_THREAD_MUTEX_H_

#include <mutex>

/**
 * @namespace common
 */
namespace common {

/**
 * @class ThreadMutex
 *
 * @brief Encapsulating interface for thread mutex lock
**/
class ThreadMutex {
public:
    inline void lock() {
        mutex_.lock();
    }

    inline void unlock() {
        mutex_.unlock();
    }
private:
    std::mutex mutex_;
};

}       //namespace common

#endif // PLANNING_COMMON_THREAD_MUTEX_H_
