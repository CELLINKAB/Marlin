
#include <array>
#include <cstddef>

namespace test {

template<typename T, T val>
struct _static_assert_val{
    constexpr _static_assert_val() = default;
    constexpr _static_assert_val(T v) {};
};

template<typename T, T A, typename U, U B>
constexpr void _static_assert_eq(_static_assert_val<T, A>, _static_assert_val<U, B>){
    static_assert(A == B, "Values not equal!");
}

#define static_assert_eq(A, B) _static_assert_eq(_static_assert_val<decltype(A), A>{}, _static_assert_val<decltype(B), B>{})

template<typename T, size_t N, size_t M>
constexpr auto array_compare(const std::array<T,N>& a, const std::array<T,M>& b) -> bool
{
    bool no_mismatches = true;
    for (size_t i = 0; i < M; ++i)
        no_mismatches &= (a[i]==b[i]);
    return no_mismatches;
};

} // namespace test