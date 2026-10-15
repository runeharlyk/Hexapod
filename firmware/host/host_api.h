#pragma once

// Marks a function of the host builds' C interfaces as exported: from the WebAssembly module the app loads and from
// the native library the Python simulation loads.

#if defined(__EMSCRIPTEN__)
#include <emscripten/emscripten.h>
#define API EMSCRIPTEN_KEEPALIVE
#elif defined(_WIN32)
#define API __declspec(dllexport)
#else
#define API __attribute__((visibility("default")))
#endif
