#pragma once
#include <feature_flags.h>
#include <communication/webserver.h>

#if EMBED_WEBAPP
#include "WWWData.h"
#endif

// Serves the embedded web app, or answers every non-API GET with 404 when it is not embedded.
void mountWebApp(WebServer &s);
