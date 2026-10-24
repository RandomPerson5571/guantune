#pragma once

#include "autotune/types.hpp"

namespace autotune {

// Hardware-free API proposal. All methods are declarations only.
class RelayTuner {
public:
    explicit RelayTuner(Config config);
    void reset();
    Step update(const Sample& sample);
    Result result() const;

private:
    Config config_;
    // TODO: Add relay state and cycle measurements with the core implementation.
};

} // namespace autotune
