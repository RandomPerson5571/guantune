#pragma once
#include <functional>
#include <memory>
#include <new>
#include <type_traits>

namespace autotune {
// The owner grants exclusive access. stop must cancel and join all owned motion
// and must not throw. Copying the PID must not throw after its lifetime ends.
template <typename PID> class IdlePidGuard {
public:
    IdlePidGuard(PID& pid, std::function<void()> stop)
        : pid_(pid), stop_(std::move(stop)), original_(pid) {
        static_assert(std::is_nothrow_copy_constructible<PID>::value,
                      "PID restoration requires a nonthrowing copy constructor");
    }
    IdlePidGuard(const IdlePidGuard&) = delete;
    IdlePidGuard& operator=(const IdlePidGuard&) = delete;
    ~IdlePidGuard() { restore(); }
    void install(const PID& candidate) {
        stop_();
        replace(candidate);
        restored_ = false;
    }
    void restore() {
        if (restored_) return;
        stop_();
        replace(original_);
        restored_ = true;
    }
private:
    void replace(const PID& value) noexcept {
        std::destroy_at(std::addressof(pid_));
        ::new (static_cast<void*>(std::addressof(pid_))) PID(value);
    }
    PID& pid_;
    std::function<void()> stop_;
    PID original_;
    bool restored_ = true;
};
} // namespace autotune
