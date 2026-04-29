#include "loader.h"

#include <stdexcept>

namespace robosim::description {

// Phase B5/B6 stub. Every call throws so test failures are uniformly
// "not implemented" and never tautological. Replaced with the real
// implementation in B7.
std::expected<robot_description, load_error>
load_from_file(const std::filesystem::path& /*path*/) {
  throw std::logic_error("robosim::description::load_from_file: not implemented yet");
}

}  // namespace robosim::description
