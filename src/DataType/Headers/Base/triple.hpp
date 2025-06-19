#pragma once

#include <utility>

namespace iFOC
{

template <typename T1, typename T2>
struct triple
{
    T1 value;
    T1 limit;
    T2 unit;

    constexpr triple(const triple&) = default; ///< Copy constructor
    constexpr triple(triple&&) = default;      ///< Move constructor

    constexpr triple() : value(), limit(), unit() {}

    constexpr triple(T1 v, T1 l, T2 u)
            : value(v), limit(l), unit(u) {}
private:
    /// @cond undocumented
    template<typename _U1, typename _U2>
    static constexpr bool
    _S_assignable()
    {
        if constexpr (std::is_assignable_v<T1&, _U1>)
            return std::is_assignable_v<T2&, _U2>;
        return false;
    }

    template<typename _U1, typename _U2>
    static constexpr bool
    _S_const_assignable()
    {
        if constexpr (std::is_assignable_v<const T1&, _U1>)
            return std::is_assignable_v<const T2&, _U2>;
        return false;
    }

    template<typename _U1, typename _U2>
    static constexpr bool
    _S_nothrow_assignable()
    {
        if constexpr (std::is_nothrow_assignable_v<T1&, _U1>)
            return std::is_nothrow_assignable_v<T2&, _U2>;
        return false;
    }
public:
    // constexpr triple(T1&& v, T1&& l, T2&& u)
    //         : value(std::forward<T1>(v)), limit(std::forward<T1>(l)), unit(std::forward<T2>(u)) {}

    template <class U1, class U2>
    constexpr explicit triple(const std::pair<U1, U2>& p)
            : value(p.first), limit(), unit(p.second) {}

    template <class U1, class U2>
    constexpr explicit triple(std::pair<U1, U2>&& p)
            : value(std::forward<U1>(p.first)), limit(), unit(std::forward<U2>(p.second)) {}

    template <class U1, class U2>
    constexpr explicit triple(const triple<U1, U2>& t)
            : value(t.value), limit(t.limit), unit(t.unit) {}

    template <class U1, class U2>
    constexpr explicit triple(triple<U1, U2>&& t)
            : value(std::forward<U1>(t.value)), limit(std::move(t.limit)), unit(std::forward<U2>(t.unit)) {}

    triple& operator=(const triple&) = delete;

    /// Copy assignment operator
    constexpr triple&
    operator=(const triple& __p)
    noexcept(_S_nothrow_assignable<const T1&, const T2&>())
    requires (_S_assignable<const T1&, const T2&>())
    {
        value = __p.value;
        limit = __p.limit;
        unit = __p.unit;
        return *this;
    }

    /// Move assignment operator
    constexpr triple&
    operator=(triple&& __p)
    noexcept(_S_nothrow_assignable<T1, T2>())
    requires (_S_assignable<T1, T2>())
    {
        value = std::forward<T1>(__p.value);
        limit = std::forward<T1>(__p.limit);
        unit = std::forward<T2>(__p.unit);
        return *this;
    }

    /// Converting assignment from a const `pair<U1, U2>` lvalue
    template<typename _U1, typename _U2>
    constexpr triple&
    operator=(const triple<_U1, _U2>& __p)
    noexcept(_S_nothrow_assignable<const _U1&, const _U2&>())
    requires (_S_assignable<const _U1&, const _U2&>())
    {
        value = __p.value;
        limit = __p.limit;
        unit = __p.unit;
        return *this;
    }

    /// Converting assignment from a non-const `pair<U1, U2>` rvalue
    template<typename _U1, typename _U2>
    constexpr triple&
    operator=(triple<_U1, _U2>&& __p)
    noexcept(_S_nothrow_assignable<_U1, _U2>())
    requires (_S_assignable<_U1, _U2>())
    {
        value = std::forward<T1>(__p.value);
        limit = std::forward<T1>(__p.limit);
        unit = std::forward<T2>(__p.unit);
        return *this;
    }

    template <class U1, class U2>
    triple& operator=(const std::pair<U1, U2>&) = delete;

    template <class U1, class U2>
    triple& operator=(std::pair<U1, U2>&& p)
    {
        value = std::forward<U1>(p.first);
        limit = T1();
        unit = std::forward<U2>(p.second);
        return *this;
    }

    triple& operator=(const std::pair<T1, T2>&) = delete;

    triple& operator=(std::pair<T1, T2>&& p)
    {
        value = std::forward<T1>(p.first);
        limit = T1();
        unit = std::forward<T2>(p.second);
        return *this;
    }

    constexpr void
    swap(triple& __p)
    noexcept(std::__and_<std::__is_nothrow_swappable<T1>,
            std::__is_nothrow_swappable<T2>>::value)
    {
        using std::swap;
        swap(value, __p.value);
        swap(limit, __p.limit);
        swap(unit, __p.unit);
    }
};

template <class T1, class T2>
inline bool operator==(const triple<T1, T2>& x, const triple<T1, T2>& y)
{
    return x.value == y.value && x.limit == y.limit && x.unit == y.unit;
}

template <class T1, class T2>
inline bool operator!=(const triple<T1, T2>& x, const triple<T1, T2>& y)
{
    return !(x == y);
}

// template <class T1, class T2>
// inline bool operator<(const triple<T1, T2>& x, const triple<T1, T2>& y)
// {
//     return x.value < y.value ||
//            (!(y.value < x.value) && x.limit < y.limit) ||
//            (!(y.value < x.value) && !(y.limit < x.limit) && x.unit < y.unit);
// }
//
// template <class T1, class T2>
// inline bool operator<=(const triple<T1, T2>& x, const triple<T1, T2>& y)
// {
//     return !(y < x);
// }
//
// template <class T1, class T2>
// inline bool operator>(const triple<T1, T2>& x, const triple<T1, T2>& y)
// {
//     return y < x;
// }
//
// template <class T1, class T2>
// inline bool operator>=(const triple<T1, T2>& x, const triple<T1, T2>& y)
// {
//     return !(x < y);
// }

template <class T1, class T2>
inline void swap(triple<T1, T2>& x, triple<T1, T2>& y) noexcept(noexcept(x.swap(y)))
{
    x.swap(y);
}

template <class T1, class T2>
inline triple<T1, T2> make_triple(T1 v, T1 l, T2 u)
{
    return triple<T1, T2>(v, l, u);
}
}