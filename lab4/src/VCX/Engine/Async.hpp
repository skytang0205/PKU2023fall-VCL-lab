#pragma once

#include <any>
#include <functional>
#include <optional>
#include <stdexcept>
#include <thread>

namespace VCX::Engine {
    // an computationally expensive value that will be asynchronously evaluated.
    // the method names are intendedly aligned with std::optional<T>;
    template<typename T>
    class Async {
    public:
        Async() = default;
        Async(std::function<T()> && func) { Emplace(func); }

        ~Async() { if (_thread.joinable()) _thread.join(); }

        void Reset() {
          if (_thread.joinable()) _thread.join();
          _completed = false;
        }

        void Emplace(std::function<T()> && func) {
            if (_thread.joinable()) _thread.join();
            _completed = false;
            _thread = std::move(std::thread([
                func        = std::move(func),
                & completed = _completed,
                & result    = _result]() {
                result = func();
                completed.store(true);
            }));
        }

        bool HasValue() const { return _completed.load(); }

        T const & Value() const {
            if (HasValue())
                return _result.value();
            else
                throw std::runtime_error("result is not ready.");
        }

        T const & ValueOr(T const & alt) const {
            if (HasValue())
                return _result.value();
            else
                return alt;
        }

        T const & WaitForValue() {
            if (_thread.joinable()) _thread.join();
            return _result.value();
        }

        bool IsCompleted() const {
          return _completed.load();
        }

    private:
        std::thread     _thread;
        std::atomic_bool _completed = false;
        std::optional<T> _result;
    };
}
