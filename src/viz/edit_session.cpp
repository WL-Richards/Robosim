#include "edit_session.h"

#include "description/error.h"
#include "description/loader.h"
#include "description/schema.h"
#include "description/serializer.h"

#include <expected>
#include <filesystem>
#include <utility>

namespace robosim::viz {

namespace desc = robosim::description;

std::expected<edit_session, desc::load_error> load_session(
    const std::filesystem::path& path) {
  auto loaded = desc::load_from_file(path);
  if (!loaded) {
    return std::unexpected(loaded.error());
  }
  edit_session session;
  session.description = std::move(*loaded);
  session.source_path = path;
  session.dirty = false;
  return session;
}

std::expected<void, desc::save_error> save_session(edit_session& session) {
  auto result = desc::save_to_file(session.description, session.source_path);
  if (!result) {
    return std::unexpected(result.error());
  }
  session.dirty = false;
  return {};
}

std::expected<void, desc::load_error> reload_session(edit_session& session) {
  auto loaded = desc::load_from_file(session.source_path);
  if (!loaded) {
    return std::unexpected(loaded.error());
  }
  session.description = std::move(*loaded);
  session.dirty = false;
  return {};
}

}  // namespace robosim::viz
