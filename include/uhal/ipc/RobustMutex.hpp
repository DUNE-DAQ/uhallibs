
#ifndef _uhal_ipc_RobustMutex_hpp_
#define _uhal_ipc_RobustMutex_hpp_

#include <cstdint>

#include "uhal/ClientInterface.hpp"
#include "uhal/log/exception.hpp"

namespace uhal {
namespace ipc {

namespace exception {
//! Exception class to handle errors from pthread mutex-related functions
UHAL_DEFINE_DERIVED_EXCEPTION_CLASS(
    MutexError, uhal::exception::TransportLayerError,
    "Exception class to handle errors from pthread mutex-related functions.")
}  // namespace exception

class RobustMutex {
 public:
  RobustMutex();
  ~RobustMutex();

  void lock();

  void unlock();

  uint64_t getCounter() const;

  bool isActive() const;

  void startSession();

  void endSession();

 private:
  RobustMutex(const RobustMutex&);

  pthread_mutex_t mMutex;
  uint64_t mCount;
  bool mSessionActive;
};

}  // namespace ipc
}  // namespace uhal

#endif /* _uhal_ipc_RobustMutex_hpp_ */