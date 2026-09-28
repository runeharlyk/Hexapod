#include "www_mount.hpp"
#include <esp_log.h>

#if EMBED_WEBAPP
#include <cstring>
#include <string>

static const WebAsset *findAsset(const char *uri) {
    for (size_t i = 0; i < WWW_ASSETS_COUNT; i++) {
        if (strcmp(WWW_ASSETS[i].uri, uri) == 0) return &WWW_ASSETS[i];
    }
    return nullptr;
}

static esp_err_t web_send(httpd_req_t *req, const WebAsset &asset) {
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, asset.mime);
    if (asset.gz) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    if (WWW_OPT.add_vary) httpd_resp_set_hdr(req, "Vary", "Accept-Encoding");

    char cc[64];
    snprintf(cc, sizeof(cc), "public, immutable, max-age=%lu", (unsigned long)WWW_OPT.max_age);
    httpd_resp_set_hdr(req, "Cache-Control", cc);

    char et[34];
    snprintf(et, sizeof(et), "\"%08lx\"", (unsigned long)asset.etag);
    httpd_resp_set_hdr(req, "ETag", et);

    return httpd_resp_send(req, (const char *)asset.data, asset.len);
}

// One wildcard handler serves every asset: registering one httpd handler per file would exhaust
// config.max_uri_handlers long before a SvelteKit build runs out of files. It is registered after the
// API routes, so those still match first; unknown non-API paths get index.html for client routing.
void mountWebApp(WebServer &server) {
    const WebAsset *indexAsset = findAsset(WWW_OPT.default_uri);
    server.on("/*", HTTP_GET, [indexAsset](httpd_req_t *req) {
        if (strncmp(req->uri, "/api/", 5) == 0) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
            return ESP_FAIL;
        }
        const char *query = strchr(req->uri, '?');
        const std::string path(req->uri, query ? static_cast<size_t>(query - req->uri) : strlen(req->uri));
        const WebAsset *asset = findAsset(path.c_str());
        if (!asset) asset = indexAsset;
        if (!asset) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
            return ESP_FAIL;
        }
        return web_send(req, *asset);
    });
}
#else
// Without this handler the OPTIONS "/*" route registered for CORS matches a plain GET / and the
// server answers 405 Method Not Allowed, which reads as a broken robot to anyone opening its IP.
void mountWebApp(WebServer &server) {
    ESP_LOGW("www", "Web app not embedded (EMBED_WEBAPP=0), UI routes return 404");
    server.on("/*", HTTP_GET, [](httpd_req_t *req) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
        return ESP_FAIL;
    });
}
#endif
