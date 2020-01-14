#ifndef _RBOT_UTILS_V1_MACRO_FROM_CLASS_HPP_
#define _RBOT_UTILS_V1_MACRO_FROM_CLASS_HPP_

#include <type_traits>
#include <utility>

namespace rbot_utils {
namespace v1 {
#define __FROM_CLASS_BRING_TYPE__(CLASS, TYPE) using TYPE = typename CLASS::TYPE
#define __FROM_CLASS_BRING_TYPE_AND_CONST_TYPE__(CLASS, TYPE) \
    __FROM_CLASS_BRING_TYPE__(CLASS, TYPE);                   \
    __FROM_CLASS_BRING_TYPE__(CLASS, const_##TYPE)
}  // namespace v1
}  // namespace rbot_utils
#endif /* ifndef _RBOT_UTILS_V1_MACRO_FROM_CLASS_HPP_ */
