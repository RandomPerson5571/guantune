#pragma once

#include "autotune/types.hpp"

namespace autotune {

// Hardware-free API proposal. Unimplemented methods are deleted for compile-time diagnostics.
class RelayTuner {
public:
    explicit RelayTuner(Config config) = delete;
    void reset() = delete;
    Step update(const Sample& sample) = delete;
    Result result() const = delete;

private:
    Config config_;
    // TODO: Add relay state and cycle measurements with the core implementation.
};

} // namespace autotune
