#include <boost/interprocess/managed_shared_memory.hpp>

namespace uhal {
namespace ipc {

template <class T>
class SharedObject {
 public:
  SharedObject(const SharedObject<T>&) = delete;
  SharedObject<T>& operator=(const SharedObject<T>&) = delete;

  SharedObject(const std::string& aName);
  ~SharedObject();

  T* operator->();

  T& operator*();

 private:
  std::string mName;
  boost::interprocess::managed_shared_memory mSharedMem;
  T* mObj;
};

// Definition
template <class T>
SharedObject<T>::SharedObject(const std::string& aName)
    : mName(aName),
      mSharedMem(boost::interprocess::open_or_create, aName.c_str(), 1024, 0x0,
                 boost::interprocess::permissions(0666)),
      mObj(mSharedMem.find_or_construct<T>(
          boost::interprocess::unique_instance)()) {}

template <class T>
SharedObject<T>::~SharedObject() {
  // boost::interprocess::shared_memory_object::remove(mName.c_str());
}

template <class T>
T* SharedObject<T>::operator->() {
  return mObj;
}

template <class T>
T& SharedObject<T>::operator*() {
  return *mObj;
}
}  // namespace ipc
}  // namespace uhal