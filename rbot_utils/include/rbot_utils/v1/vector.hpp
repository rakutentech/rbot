#ifndef _RBOT_UTILS_V1_VECTOR_HPP_
#define _RBOT_UTILS_V1_VECTOR_HPP_

#include <vector>

#include <rbot_utils/v1/macro/from_class.hpp>

namespace rbot_utils {
namespace v1 {
template <class T, class Alloc = std::allocator<T>>
struct vector : public std::vector<T, Alloc>
{
  private:
    using Base = std::vector<T, Alloc>;

  public:
    using this_type = vector<T, Alloc>;
    __FROM_CLASS_BRING_TYPE__(Base, value_type);
    __FROM_CLASS_BRING_TYPE__(Base, allocator_type);
    __FROM_CLASS_BRING_TYPE__(Base, size_type);
    __FROM_CLASS_BRING_TYPE__(Base, difference_type);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Base, reference);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Base, pointer);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Base, iterator);
    __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(Base, reverse_iterator);

    using Base::Base;

    vector(const vector& vec) : Base(static_cast<Base>(vec)) {}
    vector(vector&& vec) : Base(static_cast<Base>(vec)) {}
    vector(const vector& vec, const allocator_type& alloc)
        : Base(static_cast<Base>(vec), alloc)
    {}
    vector(vector&& vec, const allocator_type& alloc)
        : Base(static_cast<Base>(vec), alloc)
    {}

    vector& operator=(const vector& vec) { return Base::operator=(vec); }
    vector& operator=(vector&& vec) noexcept(
        Base::operator=(std::declval<Base&&>()))
    {
        return Base::operator=(vec);
    }

    void pop_front() noexcept { this->erase(this->begin()); }

    void push_front(const T& value) { this->insert(this->begin(), value); }

    void push_front(T&& value)
    {
        this->insert(this->begin(), std::forward<T>(value));
    }

    template <class... Args>
    reference emplace_front(Args&&... args)
    {
        *this->emplace(this->begin(), std::forward<Args>(args)...);
    }
};
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_VECTOR_HPP_ */
