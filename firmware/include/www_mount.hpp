#pragma once
#include <features.h>

#if EMBED_WEBAPP
#include <communication/webserver.h>
#include "WWWData.h"

void mountWebApp(WebServer &s);
#endif
