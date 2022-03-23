#include "uhallibs/ipc/RobustMutex.hpp"

#include <cstring>
#include "uhal/log/log.hpp"


namespace uhallibs {
namespace ipc {

RobustMutex::RobustMutex() : mCount(0), mSessionActive(false) {
  pthread_mutexattr_t lAttr;

  int s = pthread_mutexattr_init(&lAttr);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", uhal::Integer(s), " (", strerror(s),
        ") returned in mutex attr initialisation");
    throw lExc;
  }

  s = pthread_mutexattr_setpshared(&lAttr, PTHREAD_PROCESS_SHARED);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", uhal::Integer(s), " (", strerror(s),
        ") returned by pthread_mutexattr_setpshared");
    throw lExc;
  }

  s = pthread_mutexattr_setrobust(&lAttr, PTHREAD_MUTEX_ROBUST);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", uhal::Integer(s), " (", strerror(s),
        ") returned by pthread_mutexattr_setrobust");
    throw lExc;
  }

  s = pthread_mutex_init(&mMutex, &lAttr);
  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", uhal::Integer(s), " (", strerror(s),
        ") returned in mutex initialisation");
    throw lExc;
  }
}

RobustMutex::~RobustMutex() {}

void RobustMutex::lock() {
  int s = pthread_mutex_lock(&mMutex);
  bool lLastOwnerDied = (s == EOWNERDEAD);
  if (lLastOwnerDied) s = pthread_mutex_consistent(&mMutex);

  if (s != 0) {
    exception::MutexError lExc;
    log(lExc, "Error code ", uhal::Integer(s), " (", strerror(s), ") returned when ",
        lLastOwnerDied ? "making mutex state consistent" : "locking mutex");
    throw lExc;
  }
}

void RobustMutex::unlock() {
  int s = pthread_mutex_unlock(&mMutex);
  if (s != 0)
    log(uhal::Error(), "Error code ", uhal::Integer(s), " (", strerror(s),
        ") returned when unlocking mutex");
}

uint64_t RobustMutex::getCounter() const { return mCount; }

bool RobustMutex::isActive() const { return mSessionActive; }

void RobustMutex::startSession() {
  mCount++;
  mSessionActive = true;
}

void RobustMutex::endSession() { mSessionActive = false; }

}  // namespace ipc
}  // namespace uhal