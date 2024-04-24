#pragma once

#include <cstddef>
#include <type_traits>

#include <glm/glm.hpp>

namespace VCX::Engine {

    template<typename T> struct glm_unpack;

    template<typename T>
        requires std::is_arithmetic_v<T>
    struct glm_unpack<T> {
        using type = T;
        using size = std::integral_constant<std::size_t, 1>;
    };

    template<glm::length_t N, typename T, glm::qualifier Q>
        requires std::is_arithmetic_v<T>
    struct glm_unpack<glm::vec<N, T, Q>> {
        using type = T;
        using size = std::integral_constant<std::size_t, N>;
    };

    template<glm::length_t N, glm::length_t M, typename T, glm::qualifier Q>
        requires std::is_arithmetic_v<T>
    struct glm_unpack<glm::mat<N, M, T, Q>> {
        using type = T;
        using size = std::integral_constant<std::size_t, N * M>;
    };

    template<typename T>
    using glm_type_of = typename glm_unpack<T>::type;

    template<typename T>
    using glm_size_of = typename glm_unpack<T>::size;

    template<typename T>
    constexpr std::size_t glm_size_of_v = glm_size_of<T>::value;
}
