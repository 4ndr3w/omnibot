#include "Mutex.h"

Mutex::Mutex() {
    sem = semMCreate(SEM_Q_FIFO);
}

void Mutex::lock() {
    semTake(sem, WAIT_FOREVER);
}

void Mutex::unlock() {
    semGive(sem);
}

MutexLock::MutexLock(Mutex &m) 
    : mutex(m) {
    m.lock();
}

MutexLock::~MutexLock() {
    mutex.unlock();
}