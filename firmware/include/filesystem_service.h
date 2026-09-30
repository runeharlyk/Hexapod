#pragma once

#include <esp_http_server.h>
#include <esp_log.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <cJSON.h>

#include <filesystem.h>
#include <file_transfer.h>
#include <communication/webserver.h>

// HTTP (not the message bus — file content is large/on-network). Paths reject ".." so a request
// can't escape the mount root.
namespace fs_api {

inline bool resolve(const char *rel, std::string &full) {
    if (!file_transfer::validPath(rel)) return false;
    full = std::string(MOUNT_POINT) + rel;
    return true;
}

inline void walk(const std::string &dirPath, cJSON *node) {
    DIR *dir = opendir(dirPath.c_str());
    if (!dir) return;
    for (struct dirent *e = readdir(dir); e != nullptr; e = readdir(dir)) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        std::string child = dirPath + "/" + e->d_name;
        struct stat st;
        if (stat(child.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            cJSON *sub = cJSON_CreateObject();
            walk(child, sub);
            cJSON_AddItemToObject(node, e->d_name, sub);
        } else {
            cJSON_AddNumberToObject(node, e->d_name, static_cast<double>(st.st_size));
        }
    }
    closedir(dir);
}

inline esp_err_t list(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON *tree = cJSON_CreateObject();
    walk(MOUNT_POINT, tree);
    cJSON_AddItemToObject(root, "root", tree);
    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_sendstr(req, out ? out : "{\"root\":{}}");
    cJSON_free(out);
    cJSON_Delete(root);
    return ret;
}

inline esp_err_t read(httpd_req_t *req) {
    char query[256] = {0};
    char rel[200] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "path", rel, sizeof(rel)) != ESP_OK) {
        return WebServer::sendError(req, 400, "missing path");
    }
    std::string full;
    if (!resolve(rel, full)) return WebServer::sendError(req, 400, "bad path");
    std::string content;
    if (!FileSystem::readFile(full.c_str(), content)) return WebServer::sendError(req, 404, "not found");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, content.data(), content.size());
}

inline bool body(httpd_req_t *req, std::string &out) {
    int len = req->content_len;
    if (len <= 0 || len > 32768) return false;
    out.resize(len);
    int got = 0;
    while (got < len) {
        int r = httpd_req_recv(req, &out[got], len - got);
        if (r <= 0) return false;
        got += r;
    }
    return true;
}

inline esp_err_t edit(httpd_req_t *req) {
    std::string raw;
    if (!body(req, raw)) return WebServer::sendError(req, 400, "bad body");
    cJSON *j = cJSON_Parse(raw.c_str());
    if (!j) return WebServer::sendError(req, 400, "bad json");
    const cJSON *path = cJSON_GetObjectItem(j, "path");
    const cJSON *content = cJSON_GetObjectItem(j, "content");
    std::string full;
    bool ok = cJSON_IsString(path) && cJSON_IsString(content) && resolve(path->valuestring, full) &&
              FileSystem::writeFile(full.c_str(), content->valuestring);
    cJSON_Delete(j);
    if (!ok) return WebServer::sendError(req, 400, "write failed");
    return httpd_resp_sendstr(req, "{}");
}

inline esp_err_t remove(httpd_req_t *req) {
    std::string raw;
    if (!body(req, raw)) return WebServer::sendError(req, 400, "bad body");
    cJSON *j = cJSON_Parse(raw.c_str());
    if (!j) return WebServer::sendError(req, 400, "bad json");
    const cJSON *path = cJSON_GetObjectItem(j, "path");
    std::string full;
    bool ok = cJSON_IsString(path) && resolve(path->valuestring, full) && unlink(full.c_str()) == 0;
    cJSON_Delete(j);
    if (!ok) return WebServer::sendError(req, 400, "delete failed");
    return httpd_resp_sendstr(req, "{}");
}

inline esp_err_t mkdir(httpd_req_t *req) {
    std::string raw;
    if (!body(req, raw)) return WebServer::sendError(req, 400, "bad body");
    cJSON *j = cJSON_Parse(raw.c_str());
    if (!j) return WebServer::sendError(req, 400, "bad json");
    const cJSON *path = cJSON_GetObjectItem(j, "path");
    std::string full;
    bool ok = cJSON_IsString(path) && resolve(path->valuestring, full) && FileSystem::mkdirRecursive(full.c_str());
    cJSON_Delete(j);
    if (!ok) return WebServer::sendError(req, 400, "mkdir failed");
    return httpd_resp_sendstr(req, "{}");
}

inline void registerRoutes(WebServer &server) {
    server.on("/api/files", HTTP_GET, list);
    server.on("/api/files/content", HTTP_GET, read);
    server.on("/api/files/edit", HTTP_POST, edit);
    server.on("/api/files/delete", HTTP_POST, remove);
    server.on("/api/files/mkdir", HTTP_POST, mkdir);
}

} // namespace fs_api
