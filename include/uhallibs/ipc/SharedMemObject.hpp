#ifndef _uhal_ipc_SharedMemObject_hpp_
#define _uhal_ipc_SharedMemObject_hpp_

#include <string>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace uhallibs {
namespace ipc {

template <class T>
class SharedMemObject {
 public:
  SharedMemObject(const SharedMemObject<T>&) = delete;
  SharedMemObject<T>& operator=(const SharedMemObject<T>&) = delete;

  SharedMemObject(const std::string& aName);
  ~SharedMemObject();

  T* operator->();

  T& operator*();

 private:
  std::string mName;
  boost::interprocess::managed_shared_memory mSharedMem;
  T* mObj;
};

// Definition
template <class T>
SharedMemObject<T>::SharedMemObject(const std::string& aName)
    : mName(aName),
      mSharedMem(boost::interprocess::open_or_create, aName.c_str(), 1024, 0x0,
                 boost::interprocess::permissions(0666)),
      mObj(mSharedMem.find_or_construct<T>(
          boost::interprocess::unique_instance)()) {}

template <class T>
SharedMemObject<T>::~SharedMemObject() {
  // boost::interprocess::shared_memory_object::remove(mName.c_str());
}

template <class T>
T* SharedMemObject<T>::operator->() {
  return mObj;
}

template <class T>
T& SharedMemObject<T>::operator*() {
  return *mObj;
}
}  // namespace ipc
}  // namespace uhal

#endif /* _uhal_ipc_SharedMemObject_hpp_ */