// Host definitions for firmware/include/filesystem.h. The host's files under MOUNT_POINT come from its
// own file system image (Emscripten preloads the animations there), so directories are never created.

#include <filesystem.h>

namespace FileSystem {

bool mkdirRecursive(const char *) { return true; }

}  // namespace FileSystem
