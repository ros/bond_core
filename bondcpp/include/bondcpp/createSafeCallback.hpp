
#ifndef BONDCPP__CREATE_SAFE_CALLBACK_HPP_
#define BONDCPP__CREATE_SAFE_CALLBACK_HPP_

#include <memory>
#include <type_traits>

// Expects obj to inhert from std::enable_shared_from_this
template<typename T, typename MessageT>
auto createSafeSubscriptionMemFuncCallback(
    std::shared_ptr<T> obj,
    void (T::*memberFunc)(const MessageT &)
) {
  static_assert(std::is_base_of_v<std::enable_shared_from_this<T>, T>,
    "Type expected to inherit from std::enable_shared_from_this");

  std::weak_ptr<T> weak_obj = obj;
  return [weak_obj, memberFunc](const MessageT & msg) {
      if (auto shared_obj = weak_obj.lock()) {
          ((*shared_obj).*memberFunc)(msg);
      }
  };
}

#endif // BONDCPP__CREATE_SAFE_CALLBACK_HPP_
