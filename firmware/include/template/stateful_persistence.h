#pragma once

#include <template/stateful_service.h>
#include <template/state_result.h>
#include <filesystem.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <cstdio>
#include <sys/stat.h>
#include <esp_log.h>
#include <string>
#include <vector>

static const char *TAG_PERSISTENCE = "FSPersistencePB";

template <class T>
class FSPersistencePB {
  public:
    using ProtoStateReader = std::function<void(const T &, T &)>;
    using ProtoStateUpdater = std::function<StateUpdateResult(const T &, T &)>;

    FSPersistencePB(ProtoStateReader stateReader, ProtoStateUpdater stateUpdater, StatefulService<T> *statefulService,
                    const char *filePath, const pb_msgdesc_t *msgDescriptor, size_t maxSize, const T &defaultState)
        : _stateReader(stateReader),
          _stateUpdater(stateUpdater),
          _statefulService(statefulService),
          _filePath(filePath),
          _msgDescriptor(msgDescriptor),
          _maxSize(maxSize),
          _defaultState(defaultState),
          _updateHandlerId(0) {
        enableUpdateHandler();
    }

    // A file that exists but cannot be loaded (a schema change, a torn write) is kept as `.bak`
    // before the defaults replace it, so the WiFi networks or servo calibration in it are not lost.
    void readFromFS() {
        if (loadFromFS()) return;
        struct stat st;
        if (stat(_filePath, &st) == 0) {
            const std::string backup = std::string(_filePath) + ".bak";
            ESP_LOGW(TAG_PERSISTENCE, "Invalid settings in %s, kept as %s, restoring defaults", _filePath,
                     backup.c_str());
            FileSystem::replaceFile(_filePath, backup.c_str());
        }
        applyDefaults();
        writeToFS();
    }

    // Written beside the file and renamed over it, so a reset mid-write leaves the old settings.
    bool writeToFS() {
        std::vector<uint8_t> buffer(_maxSize);
        pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), _maxSize);

        T protoMsg = {};
        _statefulService->read([this, &protoMsg](const T &state) { _stateReader(state, protoMsg); });

        if (!pb_encode(&stream, _msgDescriptor, &protoMsg)) {
            ESP_LOGE(TAG_PERSISTENCE, "Failed to encode %s: %s", _filePath, PB_GET_ERROR(&stream));
            return false;
        }

        mkdirs();

        const std::string temp = std::string(_filePath) + ".tmp";
        FILE *file = fopen(temp.c_str(), "wb");
        if (!file) {
            ESP_LOGE(TAG_PERSISTENCE, "Failed to open file for writing: %s", temp.c_str());
            return false;
        }

        const bool written = fwrite(buffer.data(), 1, stream.bytes_written, file) == stream.bytes_written;
        const bool closed = fclose(file) == 0;
        if (!written || !closed || !FileSystem::replaceFile(temp.c_str(), _filePath)) {
            ESP_LOGE(TAG_PERSISTENCE, "Failed to write %s", _filePath);
            std::remove(temp.c_str());
            return false;
        }
        return true;
    }

    void disableUpdateHandler() {
        if (_updateHandlerId) {
            _statefulService->removeUpdateHandler(_updateHandlerId);
            _updateHandlerId = 0;
        }
    }

    void enableUpdateHandler() {
        if (!_updateHandlerId) {
            _updateHandlerId = _statefulService->addUpdateHandler([&](const std::string &originId) { writeToFS(); });
        }
    }

  private:
    // A file that decodes but fails validation would otherwise leave the state zero-initialised, so it
    // counts as unloadable like an unreadable one.
    bool loadFromFS() {
        FILE *file = fopen(_filePath, "rb");
        if (!file) return false;

        fseek(file, 0, SEEK_END);
        const long fileSize = ftell(file);
        fseek(file, 0, SEEK_SET);
        // Zero bytes is a valid message: every field at its default, as an uncalibrated IMU saves.
        if (fileSize < 0 || static_cast<size_t>(fileSize) > _maxSize) {
            fclose(file);
            return false;
        }

        std::vector<uint8_t> buffer(fileSize);
        const size_t bytesRead = fread(buffer.data(), 1, fileSize, file);
        fclose(file);
        if (bytesRead != static_cast<size_t>(fileSize)) return false;

        T protoMsg = {};
        pb_istream_t stream = pb_istream_from_buffer(buffer.data(), bytesRead);
        return pb_decode(&stream, _msgDescriptor, &protoMsg) &&
               _statefulService->updateWithoutPropagation(
                   [this, &protoMsg](T &state) { return _stateUpdater(protoMsg, state); }) != StateUpdateResult::ERROR;
    }

    ProtoStateReader _stateReader;
    ProtoStateUpdater _stateUpdater;
    StatefulService<T> *_statefulService;
    const char *_filePath;
    const pb_msgdesc_t *_msgDescriptor;
    size_t _maxSize;
    T _defaultState;
    HandlerId _updateHandlerId;

    void mkdirs() {
        std::string path(_filePath);
        size_t index = 0;
        while ((index = path.find('/', index + 1)) != std::string::npos) {
            std::string segment = path.substr(0, index);
            struct stat st;
            if (stat(segment.c_str(), &st) != 0) {
                FileSystem::mkdirRecursive(segment.c_str());
            }
        }
    }

  protected:
    void applyDefaults() {
        _statefulService->updateWithoutPropagation([this](T &state) { return _stateUpdater(_defaultState, state); });
    }
};
