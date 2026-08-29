#ifndef Features_h
#define Features_h

#define FT_ENABLED(feature) feature

#ifndef USE_CAMERA
#define USE_CAMERA 0
#endif

#ifndef USE_MPU6050
#define USE_MPU6050 1
#endif

#ifndef USE_MAG
#define USE_MAG 0
#endif

#ifndef USE_MDNS
#define USE_MDNS 1
#endif

#ifndef EMBED_WEBAPP
#define EMBED_WEBAPP 0
#endif

#ifndef USE_ESPNOW
#define USE_ESPNOW 0
#endif

#ifndef USE_POLICY
#define USE_POLICY 0
#endif

#ifndef ESPNOW_WIFI_CHANNEL
#define ESPNOW_WIFI_CHANNEL 1
#endif

namespace feature_service {

void printFeatureConfiguration();

} // namespace feature_service

#endif
