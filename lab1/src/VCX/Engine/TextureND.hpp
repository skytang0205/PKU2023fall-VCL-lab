#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "Engine/Formats.hpp"
#include "Engine/prelude.hpp"

namespace VCX::Engine {
    template<std::size_t Dim, TextureFormat Format>
        requires (Dim == 1 || Dim == 2 || Dim == 3)
    class TextureND {
    private:
        std::array<std::size_t, Dim>          _size;
        std::vector<typename Format::Encoded> _data;

        /** Throws std::out_of_range if a invalid index is given */
        constexpr std::size_t GetIndexAt(std::array<std::size_t, Dim> const & o) const {
            // clang-format off
            if constexpr (Dim >= 1) if (o[0] >= _size[0]) throw std::out_of_range("x is out of range.");
            if constexpr (Dim >= 2) if (o[1] >= _size[1]) throw std::out_of_range("y is out of range.");
            if constexpr (Dim >= 3) if (o[2] >= _size[2]) throw std::out_of_range("z is out of range.");
            // clang-format on

            if constexpr (Dim == 1) return o[0];
            if constexpr (Dim == 2) return o[1] * _size[0] + o[0];
            if constexpr (Dim == 3) return (o[2] * _size[1] + o[1]) * _size[0] + o[0];
        }

        class Proxy {
        private:
            typename Format::Encoded & _val;
        
        public:
            Proxy(typename Format::Encoded & val) : _val(val) { }

            Proxy & operator=(typename Format::Decoded const & o) {
                _val = Format::Encode(o);
                return *this;
            }

            operator typename Format::Decoded() const { return Format::Decode(_val); }
        };

    public:
        TextureND() { _size.fill(0); }

        explicit TextureND(std::array<std::size_t, Dim> const & o):
            _size(o) {
            if constexpr (Dim == 1) _data.resize(o[0]);
            if constexpr (Dim == 2) _data.resize(o[0] * o[1]);
            if constexpr (Dim == 3) _data.resize(o[0] * o[1] * o[2]);
        }

        explicit TextureND(std::size_t const sizeX) requires (Dim == 1) :
            _size({ sizeX }), _data(sizeX) { }

        TextureND(std::size_t const sizeX, std::size_t const sizeY) requires (Dim == 2) :
            _size({ sizeX, sizeY }), _data(sizeX * sizeY) { }

        TextureND(std::size_t const sizeX, std::size_t const sizeY, std::size_t const sizeZ) requires (Dim == 3) :
            _size({ sizeX, sizeY, sizeZ }), _data(sizeX * sizeY * sizeZ) { }

        template<TextureFormat NewFormat>
            requires requires (typename Format::Encoded a) { { Format::template Cast<NewFormat>(a) } -> std::same_as<typename NewFormat::Encoded>; }
        TextureND<Dim, NewFormat> Cast() {
            TextureND<Dim, NewFormat> result(_size);
            auto sourceIter = _data.begin();
            auto resultIter = result._data.begin();
            while (sourceIter != _data.end()) {
                *resultIter = Format::template Cast<NewFormat>(*sourceIter);
                ++ sourceIter;
                ++ resultIter;
            }
            return result;
        }

        std::span<std::byte const> GetBytes() const {
            return make_span_bytes<typename Format::Encoded>(_data);
        }

        std::array<std::size_t, Dim> GetSize() const { return _size; }

        // clang-format off
        std::size_t GetSizeX() const requires (Dim >= 1) { return _size[0]; }
        std::size_t GetSizeY() const requires (Dim >= 2) { return _size[1]; }
        std::size_t GetSizeZ() const requires (Dim >= 3) { return _size[2]; }
        // clang-format on

        Proxy At(std::array<std::size_t, Dim> const & o) {
            return Proxy(_data[GetIndexAt(o)]);
        }

        typename Format::Decoded At(std::array<std::size_t, Dim> const & o) const {
            return Format::Decode(_data[GetIndexAt(o)]);
        }

        Proxy At(std::size_t const x) requires (Dim == 1) { return At({ x }); }
        Proxy At(std::size_t const x, std::size_t const y) requires (Dim == 2) { return At({ x, y }); }
        Proxy At(std::size_t const x, std::size_t const y, std::size_t const z) requires (Dim == 3) { return At({ x, y, z }); }

        typename Format::Decoded At(std::size_t const x) const requires (Dim == 1) { return At({ x }); }
        typename Format::Decoded At(std::size_t const x, std::size_t const y) const requires (Dim == 2) { return At({ x, y }); }
        typename Format::Decoded At(std::size_t const x, std::size_t const y, std::size_t const z) const requires (Dim == 3) { return At({ x, y, z }); }
        
        void Fill(typename Format::Decoded const & o) {
            std::fill(_data.begin(), _data.end(), Format::Encode(o));
        }
    };

    // clang-format off
    template<TextureFormat Format> using Texture1D = TextureND<1, Format>;
    template<TextureFormat Format> using Texture2D = TextureND<2, Format>;
    template<TextureFormat Format> using Texture3D = TextureND<3, Format>;
    // clang-format on
} // namespace VCX::Engine
