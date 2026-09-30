#pragma once

// Owns the animation state the control task drives: two clip buffers (the player reads one while a
// request loads the other), a third for validation, the player, the puppeteer target and the status
// publisher. Requests arrive on the adapter tasks and the mode worker; they either load into the
// inactive buffer under the load lock and raise a flag the control task consumes under the same lock,
// or copy a pose under a critical section. The control task is the only reader of the active clip and
// the only caller of enter, reset and tick, so a clip is never replaced while it is being evaluated.
//
// Ride height: the evaluated body z is an offset, and the runner adds a base before IK. While a clip
// that sets ride_height plays, the base is that value; otherwise, including puppeteer poses and after
// Exit, it is the STAND ride-height slider, so a clip played on a tall-standing robot stays tall and
// Exit returns to the slider's height. The base eases toward its target with the STAND smoothing.
// The evaluator and the player convert a foot leg to joints at base 0, so a clip with joint legs
// keeps its switching keyframes continuous only with ride_height 0.

#include <atomic>
#include <cstring>
#include <new>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <animation/animation.h>
#include <animation/animation_store.h>
#include <event_bus.h>
#include <message_types.h>
#include <platform_shared/message.pb.h>
#include <utils/timing.h>

// publishStatus casts the player state straight onto the wire enum.
static_assert((int)anim::State::IDLE == socket_message_AnimationState_ANIM_IDLE, "state value mismatch");
static_assert((int)anim::State::ENTRY == socket_message_AnimationState_ANIM_ENTRY, "state value mismatch");
static_assert((int)anim::State::PLAYING == socket_message_AnimationState_ANIM_PLAYING, "state value mismatch");
static_assert((int)anim::State::HOLD == socket_message_AnimationState_ANIM_HOLD, "state value mismatch");
static_assert((int)anim::State::EXIT == socket_message_AnimationState_ANIM_EXIT, "state value mismatch");

class AnimationRunner {
  public:
    static constexpr uint32_t STATUS_PERIOD_MS = 200;

    bool begin(Kinematics &kin, const float (*stance)[4]) {
        kin_ = &kin;
        stance_ = stance;
        player_ = new anim::Player(kin);
        player_->setStance(stance);
        loadMutex_ = xSemaphoreCreateMutex();
        validateMutex_ = xSemaphoreCreateMutex();
        clips_[0] = allocClip();
        clips_[1] = allocClip();
        scratch_ = allocClip();
        if (!clips_[0] || !clips_[1] || !scratch_) {
            ESP_LOGE(TAG, "no PSRAM for the clip buffers");
            return false;
        }
        buffersReady_ = store_.begin();
        return buffersReady_;
    }

    // Adapter task. Loads into the buffer the player is not reading; the control task
    // swaps and starts it on its next tick. A failed load leaves the player untouched, is logged and
    // returns false, so the caller never switches mode for it; the same file fails the same way on a
    // validate request, which is how the app learns why.
    bool requestPlay(const AnimationCommandMsg &cmd) {
        if (!buffersReady_) return false;
        xSemaphoreTake(loadMutex_, portMAX_DELAY);
        const char *error = nullptr;
        const bool ok = store_.load(cmd.name, *clips_[1 - active_], error);
        if (ok) {
            pendingParams_ = cmd;
            pendingPlay_.store(true, std::memory_order_release);
        } else {
            ESP_LOGW(TAG, "play %s refused: %s", cmd.name, error);
        }
        xSemaphoreGive(loadMutex_);
        logStackOnce(playStackLogged_, "load");
        return ok;
    }

    // A stop also cancels a loaded play the control task has not started yet.
    void requestStop() {
        pendingPlay_.store(false, std::memory_order_release);
        pendingStop_.store(true, std::memory_order_release);
    }

    // Mode worker, on a graceful leave: a play exits, and a pose held by an idle player eases home
    // through the puppet path, so no leg meets the next mode away from its stance. A player the stop
    // moves into Exit drops the stance target, as it drops any pose streamed during a play.
    void requestLeave() {
        requestStop();
        setPuppet(PoseMsg {});
    }

    // The last client has gone: a running or held clip exits, and a borrowed mode then hands back.
    void controlLost() {
        ESP_LOGI(TAG, "control lost, stopping the animation");
        requestStop();
    }

    // Mode worker: a refused borrow or an explicit mode other than ANIMATE drops what is queued.
    void cancelPendingPlay() {
        pendingPlay_.store(false, std::memory_order_release);
        pendingStop_.store(false, std::memory_order_release);
    }

    void setPuppet(const PoseMsg &p) {
        portENTER_CRITICAL(&puppetMux_);
        puppet_ = p;
        puppetPending_ = true;
        portEXIT_CRITICAL(&puppetMux_);
    }

    // Validate a file for a request handler. It has its own buffer, so it never overwrites a clip
    // that is loaded and waiting for the control task, and its own lock, so the clamp sweep does not
    // hold up a play.
    void validate(const char *name, socket_message_AnimationReport &report) {
        if (!buffersReady_) {
            report.ok = false;
            strncpy(report.error, "no PSRAM for the animation buffers", sizeof(report.error) - 1);
            return;
        }
        xSemaphoreTake(validateMutex_, portMAX_DELAY);
        xSemaphoreTake(loadMutex_, portMAX_DELAY);
        const char *error = nullptr;
        report.ok = store_.load(name, *scratch_, error);
        xSemaphoreGive(loadMutex_);
        if (!report.ok) {
            strncpy(report.error, error, sizeof(report.error) - 1);
        } else {
            report.clamped_mask = sweepClampMask(*scratch_);
        }
        xSemaphoreGive(validateMutex_);
        logStackOnce(validateStackLogged_, "validate");
    }

    // Control task, on the first ANIMATE tick: the pose the robot holds now is where the first play's
    // entry blend starts. With no play waiting, the pose eases to stance as a puppet target would. The
    // base starts at the slider, and only the height beyond it is captured as an offset.
    void enter(const BodyStateMsg &live, float sliderZm) {
        baseZm_ = sliderZm;
        anim::capturePose(live, stance_, current_);
        current_.body[anim::Z] = live.zm - baseZm_;
        havePuppet_ = !pendingPlay_.load(std::memory_order_acquire);
        if (havePuppet_) puppetTarget_ = anim::Pose {};
    }

    // Control task, on the first tick after ANIMATE: drop any play or pose and return the player to
    // idle at stance.
    void reset() {
        pendingPlay_.store(false, std::memory_order_release);
        pendingStop_.store(false, std::memory_order_release);
        portENTER_CRITICAL(&puppetMux_);
        puppetPending_ = false;
        portEXIT_CRITICAL(&puppetMux_);
        havePuppet_ = false;
        player_->~Player();
        new (player_) anim::Player(*kin_);
        player_->setStance(stance_);
        current_ = anim::Pose {};
        publishStatus(true);
    }

    // Control task, every tick in ANIMATE. sliderZm is the STAND ride-height target. Fills body (the
    // offsets and the base applied to the stance) and the 18 angles.
    void tick(float dt, float sliderZm, BodyStateMsg &body, float angles[18]) {
        const int64_t startUs = esp_timer_get_time();
        // Taken before the requests so a play or stop consumed this tick counts as a change.
        const anim::State before = player_->state();
        const anim::Clip *clipBefore = player_->clip();
        consumeRequests();
        if (player_->state() != anim::State::IDLE) {
            current_ = player_->update(dt);
            // Exit ends at stance, but a leg that blended in joint space still holds it as joint
            // angles; restate it as zero offsets so the pose reads as at stance and the feet follow the base.
            if (player_->state() == anim::State::IDLE) current_ = anim::Pose {};
        } else if (havePuppet_) {
            approachPuppet();
        }
        const bool fixedBase = player_->state() != anim::State::IDLE && player_->clip()->hasRideHeight;
        baseZm_ = lerpf(baseZm_, fixedBase ? player_->clip()->rideHeight : sliderZm, STAND_SMOOTHING);
        anim::Pose out = current_;
        out.body[anim::Z] += baseZm_;
        clampMask_ = anim::poseToAngles(out, *kin_, stance_, angles);
        anim::bodyState(out.body, stance_, body);
        for (int i = 0; i < 6; ++i)
            if (!out.legs[i].joints)
                for (int k = 0; k < 3; ++k) body.feet[i][k] += out.legs[i].v[k];
        publishStatus(before != player_->state() || clipBefore != player_->clip());
        recordTickCost(startUs);
    }

    // Whether leaving ANIMATE now would move a leg: the player is not idle, a play is loaded and
    // waiting, or the pose is away from stance. A borrowed mode is handed back only once this clears,
    // not on a finish edge, so a play cancelled before it started cannot strand the borrow; a swap
    // deferred by lock contention keeps pendingPlay_ set, so the first tick of a borrow does not
    // misfire.
    bool busy() const {
        return player_->state() != anim::State::IDLE || pendingPlay_.load(std::memory_order_acquire) ||
               !anim::atStance(current_);
    }

  private:
    static constexpr const char *TAG = "AnimationRunner";
    static constexpr float STAND_SMOOTHING = 0.06f;
    static constexpr float PUPPET_SWITCH_DEG = 0.5f;
    static constexpr int64_t TICK_COST_WINDOW_US = 5000000;
    Kinematics *kin_ = nullptr;
    const float (*stance_)[4] = nullptr;
    AnimationStore store_;
    anim::Clip *clips_[2] = {nullptr, nullptr};
    anim::Clip *scratch_ = nullptr;
    bool buffersReady_ = false;
    int active_ = 0; // changes only under loadMutex_
    anim::Player *player_ = nullptr;
    SemaphoreHandle_t loadMutex_ = nullptr;
    SemaphoreHandle_t validateMutex_ = nullptr;
    std::atomic<bool> pendingPlay_ {false};
    std::atomic<bool> pendingStop_ {false};
    AnimationCommandMsg pendingParams_ {};
    portMUX_TYPE puppetMux_ = portMUX_INITIALIZER_UNLOCKED;
    PoseMsg puppet_ {};
    bool puppetPending_ = false;
    bool havePuppet_ = false;
    anim::Pose puppetTarget_;
    anim::Pose current_; // offsets; the base is added on output
    float baseZm_ = 0.0f;
    uint32_t clampMask_ = 0;
    unsigned long lastStatusMs_ = 0;
    std::atomic<bool> playStackLogged_ {false};
    std::atomic<bool> validateStackLogged_ {false};
    int64_t tickCostMaxUs_ = 0;
    int64_t tickCostWindowStartUs_ = 0;

    // A load runs on whichever transport task delivered the request; the first of each kind reports
    // its headroom.
    static void logStackOnce(std::atomic<bool> &logged, const char *what) {
        if (logged.exchange(true)) return;
        ESP_LOGI(TAG, "%s ran on %s, stack high water mark %u B", what, pcTaskGetName(nullptr),
                 (unsigned)uxTaskGetStackHighWaterMark(nullptr));
    }

    // The worst tick in each 5 s of playing, so the cost of evaluation, IK and status can be read on
    // hardware. An idle player restarts the window, so a play shorter than the window logs nothing.
    void recordTickCost(int64_t startUs) {
        const int64_t nowUs = esp_timer_get_time();
        if (player_->state() == anim::State::IDLE) {
            tickCostWindowStartUs_ = nowUs;
            tickCostMaxUs_ = 0;
            return;
        }
        if (nowUs - startUs > tickCostMaxUs_) tickCostMaxUs_ = nowUs - startUs;
        if (nowUs - tickCostWindowStartUs_ < TICK_COST_WINDOW_US) return;
        ESP_LOGI(TAG, "tick max %lld us over the last 5 s of playing", (long long)tickCostMaxUs_);
        tickCostWindowStartUs_ = nowUs;
        tickCostMaxUs_ = 0;
    }

    static anim::Clip *allocClip() {
        void *p = heap_caps_malloc(sizeof(anim::Clip), MALLOC_CAP_SPIRAM);
        return p ? new (p) anim::Clip() : nullptr;
    }

    void consumeRequests() {
        // Never block the control loop: if a request holds the lock, the swap waits a tick.
        if (pendingPlay_.load(std::memory_order_acquire) && xSemaphoreTake(loadMutex_, 0) == pdTRUE) {
            if (pendingPlay_.exchange(false, std::memory_order_acq_rel)) {
                active_ = 1 - active_;
                anim::ParamValue values[anim::PARAM_MAX];
                for (int i = 0; i < pendingParams_.paramCount; ++i)
                    values[i] = {pendingParams_.params[i].id, pendingParams_.params[i].value};
                player_->play(clips_[active_], values, pendingParams_.paramCount, &current_);
                havePuppet_ = false;
            }
            xSemaphoreGive(loadMutex_);
        }
        if (pendingStop_.exchange(false, std::memory_order_acq_rel)) player_->stop();
        portENTER_CRITICAL(&puppetMux_);
        // A pose streamed during a play is stale by the time the play ends, so only an idle player
        // takes one.
        if (puppetPending_) {
            puppetPending_ = false;
            if (player_->state() == anim::State::IDLE) {
                for (int a = 0; a < 6; ++a) puppetTarget_.body[a] = puppet_.body[a];
                for (int i = 0; i < 6; ++i) {
                    puppetTarget_.legs[i].joints = puppet_.joints[i];
                    for (int k = 0; k < 3; ++k) puppetTarget_.legs[i].v[k] = puppet_.legs[i][k];
                }
                havePuppet_ = true;
            }
        }
        portEXIT_CRITICAL(&puppetMux_);
    }

    // Lerp toward the puppet target with the STAND smoothing. A leg whose representation changes eases
    // in joint space, as the spec blends a joint-angle leg: a foot leg becoming a joint leg is
    // converted to joints once and lerps toward the puppet joints; a joint leg becoming a foot leg lerps
    // toward the IK of the target foot and switches to the foot offset once every joint is within
    // PUPPET_SWITCH_DEG of it. The joints are solved against the body with the base, the body a foot
    // leg is output with, so the switch is continuous.
    void approachPuppet() {
        for (int a = 0; a < 6; ++a) current_.body[a] = lerpf(current_.body[a], puppetTarget_.body[a], STAND_SMOOTHING);
        float body[6];
        for (int a = 0; a < 6; ++a) body[a] = current_.body[a];
        body[anim::Z] += baseZm_;
        for (int i = 0; i < 6; ++i) {
            anim::LegTarget &leg = current_.legs[i];
            const anim::LegTarget &target = puppetTarget_.legs[i];
            if (!leg.joints && target.joints) {
                float j[3];
                anim::legJointsDeg(*kin_, body, leg.v, i, stance_, j);
                leg = {true, {j[0], j[1], j[2]}};
            }
            if (leg.joints && !target.joints) {
                float j[3];
                anim::legJointsDeg(*kin_, body, target.v, i, stance_, j);
                bool arrived = true;
                for (int k = 0; k < 3; ++k) {
                    leg.v[k] = lerpf(leg.v[k], j[k], STAND_SMOOTHING);
                    arrived = arrived && fabsf(leg.v[k] - j[k]) <= PUPPET_SWITCH_DEG;
                }
                if (arrived) leg = target;
                continue;
            }
            for (int k = 0; k < 3; ++k) leg.v[k] = lerpf(leg.v[k], target.v[k], STAND_SMOOTHING);
        }
    }

    // The validator's clamp sweep: every keyframe plus 32 evenly spaced times per segment.
    uint32_t sweepClampMask(const anim::Clip &clip) {
        float params[anim::PARAM_COUNT];
        anim::resolveParams(clip, nullptr, 0, params);
        uint32_t mask = 0;
        float angles[18];
        anim::Pose pose;
        for (int i = 0; i < clip.keyframeCount; ++i) {
            const float t0 = clip.keyframes[i].time;
            const float t1 = i + 1 < clip.keyframeCount ? clip.keyframes[i + 1].time : t0;
            const int samples = i + 1 < clip.keyframeCount ? 32 : 1;
            for (int s = 0; s < samples; ++s) {
                const float t = t0 + (t1 - t0) * (float)s / 32.0f;
                anim::evaluate(clip, params, t, *kin_, stance_, pose);
                mask |= anim::poseToAngles(pose, *kin_, stance_, angles);
            }
        }
        anim::evaluate(clip, params, clip.duration(), *kin_, stance_, pose);
        mask |= anim::poseToAngles(pose, *kin_, stance_, angles);
        return mask;
    }

    void publishStatus(bool changed) {
        const unsigned long now = millis();
        if (!anim::statusDue(changed, player_->state() == anim::State::IDLE, now, lastStatusMs_, STATUS_PERIOD_MS))
            return;
        lastStatusMs_ = now;
        socket_message_AnimationStatus s = socket_message_AnimationStatus_init_zero;
        if (player_->clip()) strncpy(s.name, player_->clip()->name, sizeof(s.name) - 1);
        s.state = (socket_message_AnimationState)(int)player_->state();
        s.t = player_->t();
        s.clamped_mask = clampMask_;
        EventBus<socket_message_AnimationStatus>::publish(s);
    }
};
