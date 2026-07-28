#pragma once
#include <features.h>

#if EMBED_WEBAPP
#include <communication/webserver.h>
#include "WWWData.h"

void mountStaticAssets(WebServer &s);
void mountSpaFallback(WebServer &s);
#endif
