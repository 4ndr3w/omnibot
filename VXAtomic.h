#ifndef VXATOMIC_H
#define VXATOMIC_H

#include "Mutex.h"

template <class type> class VXAtomic {
    Mutex mtx;
    type value;
public:
    void operator= (type val) {
        mtx.lock();
        value = val;
        mtx.unlock();
    }

    operator type() {
        mtx.lock();
        type tmp = value;
        mtx.unlock();
        return tmp;
    }
};

#endif