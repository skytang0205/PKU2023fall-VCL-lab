// MIT License
// 
// Copyright (c) 2019 nakat-t <armaiti.wizard@gmail.com>
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef NAKATT_SCOPE_HPP_
#define NAKATT_SCOPE_HPP_

#include <cstddef>
#include <exception>
#include <functional>
#include <type_traits>
#include <utility>

#define SCOPE_VERSION_MAJOR 0
#define SCOPE_VERSION_MINOR 9
#define SCOPE_VERSION_PATCH 0

#define SCOPE_STR(v) #v
#define SCOPE_VERSION_(major, minor, patch) SCOPE_STR(major) "." SCOPE_STR(minor) "." SCOPE_STR(patch)
#define SCOPE_VERSION SCOPE_VERSION_(SCOPE_VERSION_MAJOR, SCOPE_VERSION_MINOR, SCOPE_VERSION_PATCH)

#if defined(__cpp_lib_uncaught_exceptions)
#   define SCOPE_USE_SUCCESS_FAIL
#endif

#if defined(__cpp_deduction_guides)
    #define SCOPE_USE_DEDUCTION_GUIDE
#endif

namespace scope {

namespace detail {

template <typename T>
using decay_t = typename std::decay<T>::type;

template <typename T>
using remove_cvref_t = typename std::remove_cv<std::remove_reference<T>>::type;

template <bool B, typename T = void>
using enable_if_t = typename std::enable_if<B, T>::type;

template <bool B, typename T, typename F>
using conditional_t = typename std::conditional<B, T, F>::type;

template <typename T>
using remove_reference_t = typename std::remove_reference<T>::type;

template <typename T, typename U = T>
T exchange(T& obj, U&& new_value)
{
    T old_value = std::move(obj);
    obj = std::forward<U>(new_value);
    return old_value;
}

template <typename T>
constexpr typename std::add_const<T>::type& as_const(T& t) noexcept
{
    return t;
}
template <typename T>
void as_const(const T&&) = delete;

struct strategy_exit
{
    constexpr bool call_when_dtor() const noexcept { return true; }
    constexpr bool call_when_construct_failed() const noexcept { return true; }
};

#if defined(SCOPE_USE_SUCCESS_FAIL)
struct strategy_success
{
    int uncaught_on_creation_{std::uncaught_exceptions()};
    bool           call_when_dtor() const noexcept { return std::uncaught_exceptions() <= uncaught_on_creation_; }
    constexpr bool call_when_construct_failed() const noexcept { return false; }
};

struct strategy_fail
{
    int uncaught_on_creation_{std::uncaught_exceptions()};
    bool           call_when_dtor() const noexcept { return std::uncaught_exceptions() > uncaught_on_creation_; }
    constexpr bool call_when_construct_failed() const noexcept { return true; }
};
#endif // defined(SCOPE_USE_SUCCESS_FAIL)

template <typename EF, typename Strategy>
struct is_dtor_noexcept_t : public std::true_type {};

#if defined(SCOPE_USE_SUCCESS_FAIL)
template <typename EF>
struct is_dtor_noexcept_t<EF, strategy_success>
    : public conditional_t<noexcept(std::declval<EF>()()), std::true_type, std::false_type>
{};
#endif // defined(SCOPE_USE_SUCCESS_FAIL)

template <typename EF, typename Strategy>
class scope_guard
{
public:
    template <
        typename EFP,
        enable_if_t<std::is_constructible<EF, EFP>::value, std::nullptr_t> = nullptr,
        enable_if_t<(!std::is_lvalue_reference<EFP>::value && std::is_nothrow_constructible<EF, EFP>::value), std::nullptr_t> = nullptr
    >
    explicit scope_guard(EFP&& f) noexcept
        : exit_function_{std::forward<EFP>(f)}
    {}

    template <
        typename EFP,
        enable_if_t<std::is_constructible<EF, EFP>::value, std::nullptr_t> = nullptr,
        enable_if_t<!(!std::is_lvalue_reference<EFP>::value && std::is_nothrow_constructible<EF, EFP>::value), std::nullptr_t> = nullptr,
        enable_if_t<std::is_nothrow_constructible<EF, EFP>::value || std::is_nothrow_constructible<EF, EFP&>::value, std::nullptr_t> = nullptr
    >
    explicit scope_guard(EFP&& f) noexcept
        : exit_function_{f}
    {}

    template <
        typename EFP,
        enable_if_t<std::is_constructible<EF, EFP>::value, std::nullptr_t> = nullptr,
        enable_if_t<!(!std::is_lvalue_reference<EFP>::value && std::is_nothrow_constructible<EF, EFP>::value), std::nullptr_t> = nullptr,
        enable_if_t<!(std::is_nothrow_constructible<EF, EFP>::value || std::is_nothrow_constructible<EF, EFP&>::value), std::nullptr_t> = nullptr
    >
    explicit scope_guard(EFP&& f)
    try
        : exit_function_{f}
    {}
    catch(...)
    {
        if(Strategy().call_when_construct_failed()) {
            f();
        }
        throw;
    }

    template <
        typename EFP = EF,
        enable_if_t<std::is_nothrow_move_constructible<EFP>::value, std::nullptr_t> = nullptr
    >
    scope_guard(scope_guard&& rhs) noexcept
        : execute_on_destruction_{rhs.execute_on_destruction_}
        , strategy_{rhs.strategy_}
        , exit_function_{std::forward<EF>(rhs.exit_function_)}
    {
        rhs.release();
    }

    template <
        typename EFP = EF,
        enable_if_t<!std::is_nothrow_move_constructible<EFP>::value, std::nullptr_t> = nullptr,
        enable_if_t<std::is_copy_constructible<EFP>::value, std::nullptr_t> = nullptr
    >
    scope_guard(scope_guard&& rhs) noexcept(std::is_nothrow_copy_constructible<EF>::value)
        : execute_on_destruction_{rhs.execute_on_destruction_}
        , strategy_{rhs.strategy_}
        , exit_function_{rhs.exit_function_}
    {
        rhs.release();
    }

    ~scope_guard() noexcept(is_dtor_noexcept_t<EF, Strategy>::value)
    {
        if(execute_on_destruction_ && strategy_.call_when_dtor()) {
            exit_function_();
        }
    }

    void release() noexcept
    {
        execute_on_destruction_ = false;
    }

    scope_guard(const scope_guard&) = delete;
    scope_guard& operator=(const scope_guard&) = delete;
    scope_guard& operator=(scope_guard&&) = delete;

private:
    bool execute_on_destruction_{true};
    Strategy strategy_{};
    EF exit_function_;
};

template <typename EF>
class scope_exit : public scope_guard<EF, strategy_exit>
{
    using base_type = enable_if_t<!std::is_same<detail::remove_cvref_t<EF>, scope_exit>::value, scope_guard<EF, strategy_exit>>;
public:
    using base_type::base_type;
};

#if defined(SCOPE_USE_DEDUCTION_GUIDE)
template <class EF>
scope_exit(EF) -> scope_exit<EF>;
#endif // defined(SCOPE_USE_DEDUCTION_GUIDE)

#if defined(SCOPE_USE_SUCCESS_FAIL)

template <typename EF>
class scope_fail : public scope_guard<EF, strategy_fail>
{
    using base_type = enable_if_t<!std::is_same<detail::remove_cvref_t<EF>, scope_fail>::value, scope_guard<EF, strategy_fail>>;
public:
    using base_type::base_type;
};

#if defined(SCOPE_USE_DEDUCTION_GUIDE)
template <class EF>
scope_fail(EF) -> scope_fail<EF>;
#endif // defined(SCOPE_USE_DEDUCTION_GUIDE)

template <typename EF>
class scope_success : public scope_guard<EF, strategy_success>
{
    using base_type = enable_if_t<!std::is_same<detail::remove_cvref_t<EF>, scope_success>::value, scope_guard<EF, strategy_success>>;
public:
    using base_type::base_type;
};

#if defined(SCOPE_USE_DEDUCTION_GUIDE)
template <class EF>
scope_success(EF) -> scope_success<EF>;
#endif // defined(SCOPE_USE_DEDUCTION_GUIDE)

#endif // defined(SCOPE_USE_SUCCESS_FAIL)

template <class EF>
scope_exit<EF> make_scope_exit(EF&& f)
{
    return scope_exit<EF>(std::forward<EF>(f));
}

#if defined(SCOPE_USE_SUCCESS_FAIL)
template <class EF>
scope_fail<EF> make_scope_fail(EF&& f)
{
    return scope_fail<EF>(std::forward<EF>(f));
}

template <class EF>
scope_success<EF> make_scope_success(EF&& f)
{
    return scope_success<EF>(std::forward<EF>(f));
}
#endif // defined(SCOPE_USE_SUCCESS_FAIL)

template <typename T, typename U>
constexpr
conditional_t<std::is_nothrow_constructible<T, U>::value, U&&, const U&>
forward_if_nothrow_constructible(U&& value)
{
    return std::forward<U>(value);
}

struct empty_guard
{
    void release() const noexcept {}
};

template <typename T>
class resource_wrapper
{
public:
    template <
        typename Guard, typename TT,
        enable_if_t<std::is_constructible<T, TT>::value, std::nullptr_t> = nullptr
    >
    resource_wrapper(Guard&& g, TT&& value) noexcept(std::is_nothrow_constructible<T, TT>::value)
        : value_(std::forward<TT>(value))
    {
        g.release();
    }

    T&       get() noexcept       { return value_; }
    const T& get() const noexcept { return value_; }

    void reset(const resource_wrapper& other) noexcept(std::is_nothrow_copy_assignable<T>::value)
    {
        value_ = other.value_;
    }
    void reset(resource_wrapper&& other) noexcept(std::is_nothrow_move_assignable<T>::value)
    {
        value_ = std::move(other.value_);
    }
    void reset(const T& new_value) noexcept(std::is_nothrow_copy_assignable<T>::value)
    {
        value_ = new_value;
    }
    void reset(T&& new_value) noexcept(std::is_nothrow_move_assignable<T>::value)
    {
        value_ = std::forward<T>(new_value);
    }

private:
    T value_;
};

template <typename T>
class resource_wrapper<T&>
{
public:
    template <
        typename Guard, typename TT,
        enable_if_t<std::is_convertible<TT, T&>::value, std::nullptr_t> = nullptr
    >
    resource_wrapper(Guard&& g, TT&& value) noexcept(std::is_nothrow_constructible<T, TT&>::value)
        : value_(static_cast<T&>(value))
    {
        g.release();
    }

    T&       get() noexcept       { return value_.get(); }
    const T& get() const noexcept { return value_.get(); }

    void reset(const resource_wrapper& other) noexcept
    {
        value_ = other.value_;
    }
    void reset(resource_wrapper&& other) noexcept
    {
        value_ = std::move(other.value_);
    }
    void reset(const T& new_value) noexcept
    {
        value_ = std::ref(new_value);
    }
    void reset(T&& new_value) noexcept
    {
        value_ = std::ref(new_value);
    }

private:
    std::reference_wrapper<T> value_;
};

template <typename R, typename D>
class unique_resource
{
    using R1 = conditional_t<std::is_reference<R>::value, std::reference_wrapper<remove_reference_t<R>>, R>;

public:
    template <
        typename RR, typename DD,
        enable_if_t<std::is_constructible<R1, RR>::value, std::nullptr_t> = nullptr,
        enable_if_t<std::is_constructible<D , DD>::value, std::nullptr_t> = nullptr,
        enable_if_t<(std::is_nothrow_constructible<R1, RR>::value || std::is_constructible<R1, RR&>::value), std::nullptr_t> = nullptr,
        enable_if_t<(std::is_nothrow_constructible<D , DD>::value || std::is_constructible<D , DD&>::value), std::nullptr_t> = nullptr
    >
    unique_resource(RR&& r, DD&& d, bool e)
            noexcept((std::is_nothrow_constructible<R1, RR>::value || std::is_nothrow_constructible<R1, RR&>::value) &&
                     (std::is_nothrow_constructible<D , DD>::value || std::is_nothrow_constructible<D , DD&>::value))
        : resource_{make_scope_exit([&r, &d, &e]{ if(e) d(r); }), forward_if_nothrow_constructible<R, RR>(std::forward<RR>(r))}
        , deleter_{make_scope_exit([this, &d, &e]{ if(e) d(get()); }), forward_if_nothrow_constructible<D, DD>(std::forward<DD>(d))}
        , execute_on_reset_{e}
    {}

    unique_resource()
        : resource_{empty_guard{}, R{}}
        , deleter_{empty_guard{}, D{}}
        , execute_on_reset_{false}
    {};

    template <
        typename RR, typename DD,
        enable_if_t<std::is_constructible<R1, RR>::value, std::nullptr_t> = nullptr,
        enable_if_t<std::is_constructible<D , DD>::value, std::nullptr_t> = nullptr,
        enable_if_t<(std::is_nothrow_constructible<R1, RR>::value || std::is_constructible<R1, RR&>::value), std::nullptr_t> = nullptr,
        enable_if_t<(std::is_nothrow_constructible<D , DD>::value || std::is_constructible<D , DD&>::value), std::nullptr_t> = nullptr
    >
    unique_resource(RR&& r, DD&& d) noexcept((std::is_nothrow_constructible<R1, RR>::value || std::is_nothrow_constructible<R1, RR&>::value) &&
                                             (std::is_nothrow_constructible<D , DD>::value || std::is_nothrow_constructible<D , DD&>::value))
        : resource_{make_scope_exit([&r, &d]{ d(r); }), forward_if_nothrow_constructible<R, RR>(std::forward<RR>(r))}
        , deleter_{make_scope_exit([this, &d]{ d(get()); }), forward_if_nothrow_constructible<D, DD>(std::forward<DD>(d))}
    {}

    unique_resource(const unique_resource&) = delete;
    unique_resource& operator=(const unique_resource&) = delete;

    unique_resource(unique_resource&& rhs) noexcept(std::is_nothrow_move_constructible<R1>::value && std::is_nothrow_move_constructible<D>::value)
        : resource_{empty_guard{}, std::move_if_noexcept(rhs.get())}
        , deleter_{make_scope_exit([&rhs]{ rhs.reset(); }), std::move_if_noexcept(rhs.deleter_.get())}
        , execute_on_reset_{exchange(rhs.execute_on_reset_, false)}
    {}

    template <
        typename RR1 = R1, typename DD = D,
        enable_if_t<std::is_nothrow_move_assignable<RR1>::value, std::nullptr_t> = nullptr,
        enable_if_t<std::is_nothrow_move_assignable<DD>::value, std::nullptr_t> = nullptr
    >
    unique_resource& operator=(unique_resource&& rhs) noexcept
    {
        if(this != &rhs) {
            reset();
            resource_.reset(std::move(rhs.resource_));
            deleter_.reset(std::move(rhs.deleter_));
            execute_on_reset_ = exchange(rhs.execute_on_reset_, false);
        }
        return *this;
    }

    template <
        typename RR1 = R1, typename DD = D,
        enable_if_t<std::is_nothrow_move_assignable<RR1>::value, std::nullptr_t> = nullptr,
        enable_if_t<!std::is_nothrow_move_assignable<DD>::value, std::nullptr_t> = nullptr
    >
    unique_resource& operator=(unique_resource&& rhs)
    {
        if(this != &rhs) {
            reset();
            deleter_.reset(rhs.deleter_);
            resource_.reset(std::move(rhs.resource_));
            execute_on_reset_ = exchange(rhs.execute_on_reset_, false);
        }
        return *this;
    }

    template <
        typename RR1 = R1, typename DD = D,
        enable_if_t<!std::is_nothrow_move_assignable<RR1>::value, std::nullptr_t> = nullptr,
        enable_if_t<std::is_nothrow_move_assignable<DD>::value, std::nullptr_t> = nullptr
    >
    unique_resource& operator=(unique_resource&& rhs)
    {
        if(this != &rhs) {
            reset();
            resource_.reset(rhs.resource_);
            deleter_.reset(std::move(rhs.deleter_));
            execute_on_reset_ = exchange(rhs.execute_on_reset_, false);
        }
        return *this;
    }

    template <
        typename RR1 = R1, typename DD = D,
        enable_if_t<!std::is_nothrow_move_assignable<RR1>::value, std::nullptr_t> = nullptr,
        enable_if_t<!std::is_nothrow_move_assignable<DD>::value, std::nullptr_t> = nullptr
    >
    unique_resource& operator=(unique_resource&& rhs)
    {
        if(this != &rhs) {
            reset();
            resource_.reset(rhs.resource_);
            deleter_.reset(rhs.deleter_);
            execute_on_reset_ = exchange(rhs.execute_on_reset_, false);
        }
        return *this;
    }

    ~unique_resource()
    {
        reset();
    }

    void reset() noexcept
    {
        if(execute_on_reset_) {
            execute_on_reset_ = false;
            get_deleter()(get());
        }
    }

    template <typename RR, enable_if_t<std::is_nothrow_assignable<R1&, RR>::value, std::nullptr_t> = nullptr>
    void reset(RR&& r)
    {
        reset();
        resource_.reset(std::forward<RR>(r));
        execute_on_reset_ = true;
    }

    template <typename RR, enable_if_t<!std::is_nothrow_assignable<R1&, RR>::value, std::nullptr_t> = nullptr>
    void reset(RR&& r)
    {
        reset();
        resource_.reset(as_const(r));
        execute_on_reset_ = true;
    }

    void release() noexcept
    {
        execute_on_reset_ = false;
    }

    const R& get() const noexcept
    {
        return resource_.get();
    }

    template <
        typename RR = R,
        enable_if_t<std::is_pointer<RR>::value, std::nullptr_t> = nullptr,
        enable_if_t<!std::is_void<typename std::remove_pointer<RR>::type>::value, std::nullptr_t> = nullptr
    >
    typename std::add_lvalue_reference<typename std::remove_pointer<RR>::type>::type operator*() const noexcept
    {
        return *get();
    }

    template <typename RR = R, enable_if_t<std::is_pointer<RR>::value, std::nullptr_t> = nullptr>
    RR operator->() const noexcept
    {
        return get();
    }

    const D& get_deleter() const noexcept
    {
        return deleter_.get();
    }

private:
    resource_wrapper<R1> resource_;
    resource_wrapper<D> deleter_;
    bool execute_on_reset_{true};
};

#if defined(SCOPE_USE_DEDUCTION_GUIDE)
template <typename R, typename D>
unique_resource(R, D) -> unique_resource<R, D>;
#endif // defined(SCOPE_USE_DEDUCTION_GUIDE)

template <typename R, typename D>
unique_resource<decay_t<R>, decay_t<D>>
make_unique_resource(R&& r, D&& d)
        noexcept(std::is_nothrow_constructible<decay_t<R>, R>::value && std::is_nothrow_constructible<decay_t<D>, D>::value)
{
    unique_resource<decay_t<R>, decay_t<D>> ur{std::forward<R>(r), std::forward<D>(d)};
    return ur;
}

template <typename R, typename D, typename S = decay_t<R>>
unique_resource<decay_t<R>, decay_t<D>>
make_unique_resource_checked(R&& resource, const S& invalid, D&& d)
        noexcept(std::is_nothrow_constructible<decay_t<R>, R>::value && std::is_nothrow_constructible<decay_t<D>, D>::value)
{
    unique_resource<decay_t<R>, decay_t<D>> ur{std::forward<R>(resource), std::forward<D>(d), !bool(resource == invalid)};
    return ur;
}

} // namespace detail

using detail::scope_exit;
using detail::make_scope_exit;

#if defined(SCOPE_USE_SUCCESS_FAIL)
using detail::scope_fail;
using detail::make_scope_fail;
using detail::scope_success;
using detail::make_scope_success;
#endif // defined(SCOPE_USE_SUCCESS_FAIL)

using detail::unique_resource;
using detail::make_unique_resource;
using detail::make_unique_resource_checked;

} // namespace scope

#endif // NAKATT_SCOPE_HPP_
