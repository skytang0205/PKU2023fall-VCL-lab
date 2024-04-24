#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>

#include <glm/glm.hpp>

namespace VCX::Engine {
    template<typename T>
    concept TextureFormat = requires(typename T::Encoded e, typename T::Decoded d) {
        { T::Decode(e) } -> std::same_as<typename T::Decoded>;
        { T::Encode(d) } -> std::same_as<typename T::Encoded>;
    };

    namespace Formats {
        struct R8 {
            using Decoded = float;
            using Encoded = unsigned char;

            static Decoded Decode(Encoded const o) {
                return Decoded(o) * _dec;
            }

            static Encoded Encode(Decoded const o) {
                return std::round(std::clamp<float>(o, 0, 1) * _enc);
            }

        private:
            static constexpr float _enc { 255. };
            static constexpr float _dec { 1. / 255. };
        };

        struct RGB8 {
            using Decoded = glm::vec3;
            using Encoded = glm::u8vec3;

            static Decoded Decode(Encoded const & o) {
                return Decoded(o) * _dec;
            }

            static Encoded Encode(Decoded const & o) {
                return {
                    std::round(std::clamp<float>(o.r, 0, 1) * _enc),
                    std::round(std::clamp<float>(o.g, 0, 1) * _enc),
                    std::round(std::clamp<float>(o.b, 0, 1) * _enc),
                };
            }

        private:
            static constexpr float _enc { 255. };
            static constexpr float _dec { 1. / 255. };
        };

        struct RGBA8 {
            using Decoded = glm::vec4;
            using Encoded = glm::u8vec4;

            static Decoded Decode(Encoded const & o) {
                return Decoded(o) * _dec;
            }

            static Encoded Encode(Decoded const & o) {
                return {
                    std::round(std::clamp<float>(o.r, 0, 1) * _enc),
                    std::round(std::clamp<float>(o.g, 0, 1) * _enc),
                    std::round(std::clamp<float>(o.b, 0, 1) * _enc),
                    std::round(std::clamp<float>(o.a, 0, 1) * _enc),
                };
            }

            template<TextureFormat NewFormat>
                requires std::is_same_v<NewFormat, RGB8>
            static typename NewFormat::Encoded Cast(const Encoded & val) {
                if constexpr (std::is_same_v<NewFormat, RGB8>)
                    return { val.r, val.g, val.b };
            }

        private:
            static constexpr float _enc { 255. };
            static constexpr float _dec { 1. / 255. };
        };

        struct R16 {
            using Decoded = float;
            using Encoded = unsigned short;

            static Decoded Decode(Encoded const o) {
                return Decoded(o) * _dec;
            }

            static Encoded Encode(Decoded const o) {
                return std::round(std::clamp<float>(o, 0, 1) * _enc);
            }

        private:
            static constexpr float _enc { 65535. };
            static constexpr float _dec { 1. / 65535. };
        };

        struct D32 {
            using Decoded = float;
            using Encoded = unsigned int;

            static Decoded Decode(Encoded const o) {
                return Decoded(o) * _dec;
            }

            static Encoded Encode(Decoded const o) {
                return std::round(std::clamp<float>(o, 0, 1) * _enc);
            }

        private:
            static constexpr float _enc { 4294967295. };
            static constexpr float _dec { 1. / 4294967295. };
        };

        struct D24S8 {
            using Decoded = std::pair<float, unsigned char>;
            using Encoded = unsigned int;

            static Decoded Decode(Encoded const o) {
                return std::make_pair(float(o & 0x00ffffff) * _dec, o & 0xff000000);
            }

            static Encoded Encode(Decoded const o) {
                return Encoded(std::clamp<float>(o.first, 0, 1) * _enc) | (Encoded(o.second) << 24);
            }

        private:
            static constexpr float _enc { 16777215. };
            static constexpr float _dec { 1. / 16777215. };
        };
    } // namespace Formats

} // namespace VCX::Engine
