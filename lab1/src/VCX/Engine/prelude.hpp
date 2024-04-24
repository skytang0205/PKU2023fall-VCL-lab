#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <span>
#include <utility>
#include <vector>

namespace VCX::Engine {
    template<typename T>
    std::span<std::byte const>
        make_span_bytes(std::span<T const> const & span) {
        return std::span<std::byte const>(
            (std::byte const *) span.data(),
            span.size_bytes());
    }

    namespace detail {
        template<typename T, size_t ... Indices, typename ... Args>
        auto make_array_impl(std::index_sequence<Indices...>, Args&& ... args) {
            return std::to_array({ std::forward<T>((Indices, T(std::forward<Args>(args)...)))... });
        }
    }

    template<typename T, std::size_t N, typename... Args>
    constexpr auto make_array(Args && ... args) {
        return detail::make_array_impl<T>(std::make_index_sequence<N>{}, std::forward<Args>(args)...);
    }
}
