#ifndef MUTEX_H
#define MUTEX_H

#include "semLib.h"

class Mutex {
    SEM_ID sem;
public:
    Mutex();
    void lock();
    void unlock();
};

class MutexLock {
    Mutex &mutex;
    public:
        MutexLock(Mutex &mutex);
        ~MutexLock();
};

#endif