/**
 * @file demo_arm_behaviour.cpp
 * @brief Stationary demo behaviour: arm mimicry and voice, no base movement.
 *
 * @details
 * This file implements `MechelangeloDemoBehaviour`, a separate ROS 2 node used
 * when MECHelangelo should stay still but still interact with visitors. It is
 * useful for exhibitions, bench testing, and arm/voice demonstrations where
 * driving the base is unnecessary or unsafe.
 *
 * This file does **not** use `MechelangeloBehaviour` from `behaviour.hpp`.
 * It defines its own simpler `rclcpp::Node` subclass with only two states.
 *
 * ### What demo mode does
 *
 * | Behaviour | Implementation |
 * |---|---|
 * | Keep the base still | Publishes zero `/cmd_vel` every control tick. |
 * | Wait for a person | Subscribes to `/human_detected` and `/human_tracking`. |
 * | Start interaction | Publishes `/interaction_active = true` when detection is accepted. |
 * | Move arms | Forwards `/arm/mimicry_right_pose` and `/arm/mimicry_left_pose` to `/arm/right_pose` and `/arm/left_pose`. |
 * | Speak | Uses the same background voice queue pattern as the physical behaviour file. |
 * | End interaction | Stops after `kInteractionDurationSeconds` or if the person is lost beyond timeout. |
 * | Avoid immediate retrigger | Uses `kDetectionCooldownSeconds` before accepting a new person. |
 *
 * ### File structure
 *
 * | Section | What it contains |
 * |---|---|
 * | Constants | Control period, voice intervals, arm keepalive, tracking timeout, session/cooldown times. |
 * | `VoiceGroup` | Audio categories used by the demo. |
 * | `DemoState` | Two-state FSM: `SEARCHING` and `INTERACTION`. |
 * | File-scope publishers/subscribers | Arm pose publishers, interaction publisher, mimicry subscribers. |
 * | Voice helpers | Audio discovery, grouping, queueing, background playback, mimicry pose voice triggers. |
 * | Arm/interact helpers | `publishArmsDown()` and `publishInteractionActive()`. |
 * | `MechelangeloDemoBehaviour` | The stationary demo node class, callbacks, control loop, and state transitions. |
 *
 * ### Full control flow
 *
 * @code
 * demo_launch.py
 *   -> starts mechelangelo_demo_behaviour
 *
 * MechelangeloDemoBehaviour()
 *   -> creates arm publishers
 *   -> creates /interaction_active publisher
 *   -> subscribes to human and mimicry topics
 *   -> starts the 100 ms timer
 *
 * controlLoop()
 *   -> always publishes zero /cmd_vel
 *   -> if SEARCHING: republish arm_down and occasionally play ambient voice
 *   -> if INTERACTION: keep mimicry active, forward arm poses, play voice
 *   -> if timeout/lost: return to SEARCHING with cooldown
 * @endcode
 *
 * ### Main functions and how they interact
 *
 * | Function/helper | What it does | Key variables changed |
 * |---|---|---|
 * | `MechelangeloDemoBehaviour()` | Builds the demo node and ROS interfaces. | Publishers, subscribers, timer, voice worker, initial arms-down state. |
 * | `controlLoop()` | Runs the two-state demo FSM every 100 ms. | `state_`, interaction timing, zero `/cmd_vel`, arm-down keepalive. |
 * | `humanDetectedCallback()` | Handles detection edge events from the camera bridge. | Starts interaction if cooldown allows. |
 * | `humanTrackingCallback()` | Stores continuous tracking/grace state. | `human_locked_`, `human_tracking_valid_`, `human_tracking_grace_active_`, `last_human_tracking_time_`. |
 * | `startInteraction()` | Transitions from waiting to interaction. | `state_`, `interaction_start_time_`, `/interaction_active`. |
 * | `returnToSearching()` | Ends interaction and optionally starts cooldown. | `state_`, cooldown timing, arm-down pose, `/interaction_active`. |
 * | `publishArmsDown()` | Keeps both arms in the safe rest pose while waiting. | `/arm/right_pose`, `/arm/left_pose`. |
 * | `publishInteractionActive()` | Opens/closes mimicry for the Pi 4/sim camera. | `/interaction_active`. |
 * | `playVoiceLine()` and helpers | Queue voice audio without blocking control. | `g_voice_queue`, rate-limit maps, phrase group indices. |
 * | `playMimicryPoseVoice()` | Plays special lines for salute, strong-arm, and handshake poses. | Last pose voice state and voice queue. |
 *
 * ### Important variables
 *
 * | Variable | Meaning |
 * |---|---|
 * | `state_` | Current demo FSM state: waiting or interacting. |
 * | `human_locked_` | Whether the camera bridge currently has or is holding a person. |
 * | `human_tracking_valid_` | True only when the camera currently sees the person. |
 * | `human_tracking_grace_active_` | True while the camera is in reacquisition grace period. |
 * | `last_human_tracking_time_` | Used to detect stale/lost camera tracking. |
 * | `interaction_start_time_` | Start of the current timed mimicry session. |
 * | `cooldown_active_`, `cooldown_start_time_` | Suppresses immediate re-entry after a session. |
 * | `g_right_arm_pose_publisher`, `g_left_arm_pose_publisher` | File-scope publishers used by mimicry and arm-down helpers. |
 * | `g_interaction_active_publisher` | File-scope publisher for the mimicry gate. |
 * | `g_voice_queue`, `g_voice_playing` | Background voice playback state. |
 *
 * ### How this differs from the physical behaviour file
 *
 * - No LiDAR, IMU, or safety-zone logic.
 * - No `NavigationState`; demo mode uses its own `DemoState`.
 * - `/cmd_vel` is always zero.
 * - Arm mimicry begins as soon as detection is accepted, instead of after an
 *   approach and settle sequence.
 *
 * Launch it with `demo_launch.py` from `mechelangelo_bringup`.
 */

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <deque>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

// Control loop runs every 100 ms.
static constexpr double kControlPeriodSeconds = 0.1;

// Voice playback intervals (seconds).
static constexpr double kSearchingVoiceIntervalSeconds = 12.0;
static constexpr double kMimicryVoiceIntervalSeconds = 12.0;
static constexpr double kGracePeriodVoiceIntervalSeconds = 12.0;

// Arm keepalive: republish arm_down at this interval while not mimicking.
static constexpr double kArmDownRepublishPeriod = 0.50;

// Human tracking freshness timeout: treat tracking as lost after this long.
static constexpr double kHumanVisualFreshTimeout = 0.60;

// How long to wait after last visual before declaring person lost.
static constexpr double kHumanLostTimeout = 1.5;

// How long each mimicry session lasts before returning to searching.
static constexpr double kInteractionDurationSeconds = 30.0;

// Cooldown after an interaction ends before a new one can start.
static constexpr double kDetectionCooldownSeconds = 10.0;

/**
 * @brief Voice categories for the demo behaviour node.
 *
 * Each value maps to a sub-folder (or combined pool) inside the
 * `resources/` package directory.  The `DEMO_SEARCHING` group pools lines
 * from four folders (`autonomous/`, `centre adjusting/`, `grace period/`,
 * `person in safety zone/`) so the robot has plenty of ambient phrases
 * to cycle through while waiting for a visitor.
 *
 * Audio files can be added to the matching sub-folder without changing
 * any code — they will be discovered at startup.
 */
enum class VoiceGroup
{
    /** @brief Combined ambient pool played while waiting for a person (every 12 s). */
    DEMO_SEARCHING,

    /** @brief Played once when the camera first detects a person. */
    PERSON_DETECTED,

    /** @brief Played once when the camera loses a person. */
    PERSON_LOST,

    /** @brief Played while the camera is reacquiring (grace period). */
    GRACE_PERIOD,

    /** @brief Random mimicry line played during the interaction session. */
    MIMICRY_RANDOM,

    /** @brief Triggered when an arm classifies a "salute" pose. */
    MIMICRY_ATTEN_HUT,

    /** @brief Triggered when an arm classifies an "arm_90_up" or "straight_arm" pose. */
    MIMICRY_STRONG,

    /** @brief First part of the handshake greeting sequence. */
    MIMICRY_HANDSHAKE_GREETING,

    /** @brief Second part of the handshake greeting (robot introduces itself). */
    MIMICRY_HANDSHAKE_NAME
};

/**
 * @brief Top-level states for the demo behaviour state machine.
 *
 * Unlike the full behaviour node, there are only two states because the
 * robot never moves — the approach phase is skipped entirely.
 */
enum class DemoState
{
    /** @brief Waiting for a person.  Ambient voice lines play every 12 s.
     *         Arms are held in the rest pose. */
    SEARCHING,

    /** @brief Person detected and interaction is active.  Arm mimicry
     *         commands are forwarded from the camera.  Times out after
     *         `kInteractionDurationSeconds` (30 s). */
    INTERACTION
};

// ------------------------------------------------------
// File-scope voice state
// ------------------------------------------------------
static std::unordered_map<VoiceGroup, rclcpp::Time> g_last_voice_request_times;
static std::unordered_map<VoiceGroup, bool> g_voice_request_time_initialised;
static std::unordered_map<VoiceGroup, std::size_t> g_voice_phrase_group_indices;
static std::deque<std::string> g_voice_queue;
static std::mutex g_voice_mutex;
static std::condition_variable g_voice_cv;
static bool g_voice_worker_started = false;
static bool g_voice_playing = false;
static std::string g_last_right_mimicry_voice_pose;
static std::string g_last_left_mimicry_voice_pose;

// ------------------------------------------------------
// File-scope publishers / subscribers (set in constructor)
// ------------------------------------------------------
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_right_arm_pose_publisher;
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_left_arm_pose_publisher;
static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr g_interaction_active_publisher;
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_right_mimicry_pose_subscriber;
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_left_mimicry_pose_subscriber;

static rclcpp::Time g_last_arm_down_publish_time;
static bool g_arm_down_publish_time_initialised = false;
static rclcpp::Time g_last_interaction_active_publish_time;
static bool g_interaction_active_publish_time_initialised = false;
static bool g_last_interaction_active_value = false;

// ------------------------------------------------------
// File-scope interaction session / cooldown state
// ------------------------------------------------------
static bool g_interaction_session_active = false;
static rclcpp::Time g_interaction_session_start_time;
static bool g_human_detection_cooldown_active = false;
static rclcpp::Time g_human_detection_cooldown_start_time;

// Last confirmed visual observation (for lost-person decisions).
static bool g_last_visible_human_valid = false;
static rclcpp::Time g_last_visible_human_time;

// Guard so PERSON_LOST voice fires exactly once per loss event.
static bool g_person_lost_voice_played = false;

// ------------------------------------------------------
// Voice utility functions (same logic as behaviour_physical)
// ------------------------------------------------------
static std::string shellQuote(const std::string &value)
{
    std::string quoted = "'";
    for (const char character : value)
    {
        if (character == '\'')
        {
            quoted += "'\\''";
        }
        else
        {
            quoted += character;
        }
    }
    quoted += "'";
    return quoted;
}

static std::string behaviourResourceDirectory()
{
    try
    {
        return ament_index_cpp::get_package_share_directory(
            "mechelangelo_behaviour") + "/resources";
    }
    catch (const std::exception &)
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr)
        {
            return std::string(home) +
                "/ros2_ws/src/MECHelangelo/src/mechelangelo_behaviour/resources";
        }
    }
    return "/home/andy/ros2_ws/src/MECHelangelo/src/mechelangelo_behaviour/resources";
}

static std::string lowerCopy(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char character)
        {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

static std::string phraseKeyForVoiceLine(const std::string &relative_path)
{
    std::string key =
        lowerCopy(std::filesystem::path(relative_path).stem().string());

    while (!key.empty() &&
        std::isdigit(static_cast<unsigned char>(key.back())))
    {
        key.pop_back();
    }

    while (!key.empty() &&
        (std::isspace(static_cast<unsigned char>(key.back())) ||
         key.back() == '_' ||
         key.back() == '-'))
    {
        key.pop_back();
    }

    return key;
}

static bool hasAudioExtension(const std::filesystem::path &path)
{
    const std::string extension = lowerCopy(path.extension().string());
    return extension == ".mp3" ||
        extension == ".wav" ||
        extension == ".ogg" ||
        extension == ".flac" ||
        extension == ".m4a" ||
        extension == ".aac" ||
        extension == ".aiff" ||
        extension == ".aif";
}

static std::vector<std::string> audioFilesInFolder(
    const std::string &folder,
    const std::vector<std::string> &required_substrings = {},
    const std::vector<std::string> &excluded_substrings = {})
{
    std::vector<std::string> files;
    const std::filesystem::path directory =
        std::filesystem::path(behaviourResourceDirectory()) / folder;

    try
    {
        if (!std::filesystem::exists(directory) ||
            !std::filesystem::is_directory(directory))
        {
            return files;
        }

        for (const auto &entry : std::filesystem::directory_iterator(directory))
        {
            if (!entry.is_regular_file() || !hasAudioExtension(entry.path()))
            {
                continue;
            }

            const std::string filename = lowerCopy(entry.path().filename().string());
            bool include = true;

            for (const std::string &substring : required_substrings)
            {
                if (filename.find(lowerCopy(substring)) == std::string::npos)
                {
                    include = false;
                    break;
                }
            }

            if (!include)
            {
                continue;
            }

            for (const std::string &substring : excluded_substrings)
            {
                if (filename.find(lowerCopy(substring)) != std::string::npos)
                {
                    include = false;
                    break;
                }
            }

            if (include)
            {
                files.push_back(folder + "/" + entry.path().filename().string());
            }
        }
    }
    catch (const std::exception &)
    {
        files.clear();
    }

    std::sort(files.begin(), files.end());
    return files;
}

// Build the combined searching pool from 4 folders.
static std::vector<std::string> buildDemoSearchingLines()
{
    const auto autonomous = audioFilesInFolder("autonomous");
    const auto centering  = audioFilesInFolder("centre adjusting");
    const auto grace      = audioFilesInFolder("grace period");
    const auto safety     = audioFilesInFolder("person in safety zone");

    std::vector<std::string> combined;
    combined.reserve(
        autonomous.size() + centering.size() + grace.size() + safety.size());
    combined.insert(combined.end(), autonomous.begin(), autonomous.end());
    combined.insert(combined.end(), centering.begin(),  centering.end());
    combined.insert(combined.end(), grace.begin(),      grace.end());
    combined.insert(combined.end(), safety.begin(),     safety.end());
    std::sort(combined.begin(), combined.end());
    return combined;
}

static const std::vector<std::string> &voiceLinesForGroup(VoiceGroup group)
{
    static const std::vector<std::string> demo_searching_lines =
        buildDemoSearchingLines();
    static const std::vector<std::string> person_detected_lines =
        audioFilesInFolder("person detected");
    static const std::vector<std::string> person_lost_lines =
        audioFilesInFolder("person lost");
    static const std::vector<std::string> grace_period_lines =
        audioFilesInFolder("grace period");
    static const std::vector<std::string> mimicry_random_lines =
        audioFilesInFolder("mimicing", {}, {"atten hut", "i am strong"});
    static const std::vector<std::string> mimicry_atten_hut_lines =
        audioFilesInFolder("mimicing", {"atten hut"});
    static const std::vector<std::string> mimicry_strong_lines =
        audioFilesInFolder("mimicing", {"i am strong"});
    static const std::vector<std::string> mimicry_handshake_greeting_lines =
        audioFilesInFolder("person detected", {}, {"mechelangelo", "target", "found", "interesting"});
    static const std::vector<std::string> mimicry_handshake_name_lines =
        audioFilesInFolder("person detected", {"my name"});

    switch (group)
    {
    case VoiceGroup::PERSON_DETECTED:          return person_detected_lines;
    case VoiceGroup::PERSON_LOST:              return person_lost_lines;
    case VoiceGroup::GRACE_PERIOD:             return grace_period_lines;
    case VoiceGroup::MIMICRY_RANDOM:           return mimicry_random_lines;
    case VoiceGroup::MIMICRY_ATTEN_HUT:        return mimicry_atten_hut_lines;
    case VoiceGroup::MIMICRY_STRONG:           return mimicry_strong_lines;
    case VoiceGroup::MIMICRY_HANDSHAKE_GREETING: return mimicry_handshake_greeting_lines;
    case VoiceGroup::MIMICRY_HANDSHAKE_NAME:   return mimicry_handshake_name_lines;
    case VoiceGroup::DEMO_SEARCHING:
    default:                                   return demo_searching_lines;
    }
}

static std::vector<std::vector<std::string>> buildPhraseGroups(
    const std::vector<std::string> &lines)
{
    std::unordered_map<std::string, std::vector<std::string>> grouped_lines;
    std::vector<std::string> phrase_keys;

    for (const std::string &line : lines)
    {
        const std::string key = phraseKeyForVoiceLine(line);
        if (grouped_lines.find(key) == grouped_lines.end())
        {
            phrase_keys.push_back(key);
        }
        grouped_lines[key].push_back(line);
    }

    std::sort(phrase_keys.begin(), phrase_keys.end());

    std::vector<std::vector<std::string>> phrase_groups;
    phrase_groups.reserve(phrase_keys.size());
    for (const std::string &key : phrase_keys)
    {
        std::vector<std::string> &group = grouped_lines[key];
        std::sort(group.begin(), group.end());
        phrase_groups.push_back(group);
    }

    return phrase_groups;
}

static const std::vector<std::vector<std::string>> &voicePhraseGroupsForGroup(
    VoiceGroup group)
{
    static const std::vector<std::vector<std::string>> demo_searching_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::DEMO_SEARCHING));
    static const std::vector<std::vector<std::string>> person_detected_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::PERSON_DETECTED));
    static const std::vector<std::vector<std::string>> person_lost_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::PERSON_LOST));
    static const std::vector<std::vector<std::string>> grace_period_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::GRACE_PERIOD));
    static const std::vector<std::vector<std::string>> mimicry_random_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::MIMICRY_RANDOM));
    static const std::vector<std::vector<std::string>> mimicry_atten_hut_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::MIMICRY_ATTEN_HUT));
    static const std::vector<std::vector<std::string>> mimicry_strong_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::MIMICRY_STRONG));
    static const std::vector<std::vector<std::string>> mimicry_handshake_greeting_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::MIMICRY_HANDSHAKE_GREETING));
    static const std::vector<std::vector<std::string>> mimicry_handshake_name_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::MIMICRY_HANDSHAKE_NAME));

    switch (group)
    {
    case VoiceGroup::PERSON_DETECTED:          return person_detected_groups;
    case VoiceGroup::PERSON_LOST:              return person_lost_groups;
    case VoiceGroup::GRACE_PERIOD:             return grace_period_groups;
    case VoiceGroup::MIMICRY_RANDOM:           return mimicry_random_groups;
    case VoiceGroup::MIMICRY_ATTEN_HUT:        return mimicry_atten_hut_groups;
    case VoiceGroup::MIMICRY_STRONG:           return mimicry_strong_groups;
    case VoiceGroup::MIMICRY_HANDSHAKE_GREETING: return mimicry_handshake_greeting_groups;
    case VoiceGroup::MIMICRY_HANDSHAKE_NAME:   return mimicry_handshake_name_groups;
    case VoiceGroup::DEMO_SEARCHING:
    default:                                   return demo_searching_groups;
    }
}

// Cycle to the next phrase group, pick a random version within it.
static std::string voiceLineForNextPhraseCycle(VoiceGroup group)
{
    const std::vector<std::vector<std::string>> &phrase_groups =
        voicePhraseGroupsForGroup(group);
    if (phrase_groups.empty())
    {
        return "";
    }

    std::size_t &phrase_index = g_voice_phrase_group_indices[group];
    const std::vector<std::string> &versions =
        phrase_groups[phrase_index % phrase_groups.size()];
    phrase_index = (phrase_index + 1) % phrase_groups.size();

    if (versions.empty())
    {
        return "";
    }

    static std::mt19937 engine{std::random_device{}()};
    std::uniform_int_distribution<std::size_t> distribution(0, versions.size() - 1);
    return versions[distribution(engine)];
}

static void runVoicePlayerCommand(const std::string &full_path)
{
    const std::string quoted_path = shellQuote(full_path);

    std::ostringstream command;
    command
        << "sh -c \""
        << "if command -v mpg123 >/dev/null 2>&1; then "
        << "mpg123 -q " << quoted_path << "; "
        << "elif command -v ffplay >/dev/null 2>&1; then "
        << "ffplay -nodisp -autoexit -loglevel quiet " << quoted_path << "; "
        << "elif command -v cvlc >/dev/null 2>&1; then "
        << "cvlc --play-and-exit --quiet " << quoted_path << "; "
        << "fi"
        << "\" >/dev/null 2>&1";

    const int play_result = std::system(command.str().c_str());
    (void)play_result;
}

static void ensureVoiceWorkerStarted()
{
    std::lock_guard<std::mutex> lock(g_voice_mutex);
    if (g_voice_worker_started)
    {
        return;
    }

    g_voice_worker_started = true;
    std::thread(
        []
        {
            while (rclcpp::ok())
            {
                std::string full_path;
                {
                    std::unique_lock<std::mutex> lock(g_voice_mutex);
                    g_voice_cv.wait(
                        lock,
                        []
                        {
                            return !g_voice_queue.empty() || !rclcpp::ok();
                        });

                    if (!rclcpp::ok())
                    {
                        return;
                    }

                    full_path = g_voice_queue.front();
                    g_voice_queue.pop_front();
                    g_voice_playing = true;
                }

                runVoicePlayerCommand(full_path);

                {
                    std::lock_guard<std::mutex> lock(g_voice_mutex);
                    g_voice_playing = false;
                }
            }
        }).detach();
}

static bool voiceBusyOrQueued()
{
    std::unique_lock<std::mutex> lock(g_voice_mutex, std::try_to_lock);
    if (!lock.owns_lock())
    {
        return true;
    }
    return g_voice_playing || !g_voice_queue.empty();
}

static std::string fullVoicePath(const std::string &relative_path)
{
    return behaviourResourceDirectory() + "/" + relative_path;
}

static bool enqueueVoiceLine(
    const std::string &relative_path,
    bool priority = false)
{
    if (relative_path.empty())
    {
        return false;
    }

    const std::string full_path = fullVoicePath(relative_path);
    ensureVoiceWorkerStarted();

    {
        std::lock_guard<std::mutex> lock(g_voice_mutex);
        if (priority)
        {
            g_voice_queue.push_front(full_path);
        }
        else
        {
            g_voice_queue.push_back(full_path);
        }
    }
    g_voice_cv.notify_one();
    return true;
}

static bool enqueueVoiceSequence(
    const std::vector<std::string> &relative_paths,
    bool priority = false)
{
    std::vector<std::string> full_paths;
    for (const std::string &relative_path : relative_paths)
    {
        if (!relative_path.empty())
        {
            full_paths.push_back(fullVoicePath(relative_path));
        }
    }

    if (full_paths.empty())
    {
        return false;
    }

    ensureVoiceWorkerStarted();

    {
        std::lock_guard<std::mutex> lock(g_voice_mutex);
        if (priority)
        {
            g_voice_queue.insert(
                g_voice_queue.begin(),
                full_paths.begin(),
                full_paths.end());
        }
        else
        {
            g_voice_queue.insert(
                g_voice_queue.end(),
                full_paths.begin(),
                full_paths.end());
        }
    }
    g_voice_cv.notify_one();
    return true;
}

static bool playVoiceLine(
    VoiceGroup group,
    const rclcpp::Time &now,
    double min_interval_seconds = 6.0,
    bool force = false,
    bool queue_when_busy = false,
    bool priority = false)
{
    if (!force && !queue_when_busy && voiceBusyOrQueued())
    {
        return false;
    }

    const bool initialised = g_voice_request_time_initialised[group];
    if (!force && initialised &&
        (now - g_last_voice_request_times[group]).seconds() < min_interval_seconds)
    {
        return false;
    }

    const std::string relative_path = voiceLineForNextPhraseCycle(group);
    if (relative_path.empty())
    {
        return false;
    }

    if (!enqueueVoiceLine(relative_path, priority))
    {
        return false;
    }

    g_last_voice_request_times[group] = now;
    g_voice_request_time_initialised[group] = true;
    return true;
}

static bool playVoiceSequence(
    const std::vector<VoiceGroup> &groups,
    const rclcpp::Time &now,
    double min_interval_seconds = 1.0,
    bool force = true,
    bool queue_when_busy = false,
    bool priority = false)
{
    if (groups.empty())
    {
        return false;
    }

    if (!force && !queue_when_busy && voiceBusyOrQueued())
    {
        return false;
    }

    const VoiceGroup timing_group = groups.front();
    const bool initialised = g_voice_request_time_initialised[timing_group];
    if (!force && initialised &&
        (now - g_last_voice_request_times[timing_group]).seconds() < min_interval_seconds)
    {
        return false;
    }

    std::vector<std::string> relative_paths;
    for (const VoiceGroup group : groups)
    {
        const std::string relative_path = voiceLineForNextPhraseCycle(group);
        if (!relative_path.empty())
        {
            relative_paths.push_back(relative_path);
        }
    }

    const bool queued_any = enqueueVoiceSequence(relative_paths, priority);

    if (queued_any)
    {
        g_last_voice_request_times[timing_group] = now;
        g_voice_request_time_initialised[timing_group] = true;
    }

    return queued_any;
}

// Trigger the pose-specific voice line that matches an arm movement.
static bool playMimicryPoseVoice(
    const std::string &pose_name,
    const rclcpp::Time &now)
{
    if (pose_name == "salute")
    {
        return playVoiceLine(
            VoiceGroup::MIMICRY_ATTEN_HUT, now, 1.0, false, true, true);
    }
    else if (pose_name == "arm_90_up" || pose_name == "straight_arm")
    {
        return playVoiceLine(
            VoiceGroup::MIMICRY_STRONG, now, 1.0, false, true, true);
    }
    else if (pose_name == "handshake")
    {
        return playVoiceSequence(
            {VoiceGroup::MIMICRY_HANDSHAKE_GREETING, VoiceGroup::MIMICRY_HANDSHAKE_NAME},
            now, 1.0, false, true, true);
    }
    return false;
}

static void preloadVoiceLines()
{
    (void)voicePhraseGroupsForGroup(VoiceGroup::DEMO_SEARCHING);
    (void)voicePhraseGroupsForGroup(VoiceGroup::PERSON_DETECTED);
    (void)voicePhraseGroupsForGroup(VoiceGroup::PERSON_LOST);
    (void)voicePhraseGroupsForGroup(VoiceGroup::GRACE_PERIOD);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_RANDOM);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_ATTEN_HUT);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_STRONG);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_HANDSHAKE_GREETING);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_HANDSHAKE_NAME);
}

// ------------------------------------------------------
// Arm and interaction_active keepalive helpers
// ------------------------------------------------------
static void publishArmsDown(const rclcpp::Time &now, bool force = false)
{
    if (!g_right_arm_pose_publisher || !g_left_arm_pose_publisher)
    {
        return;
    }

    const bool period_elapsed = !g_arm_down_publish_time_initialised ||
        (now - g_last_arm_down_publish_time).seconds() >= kArmDownRepublishPeriod;

    if (!force && !period_elapsed)
    {
        return;
    }

    std_msgs::msg::String pose;
    pose.data = "arm_down";
    g_right_arm_pose_publisher->publish(pose);
    g_left_arm_pose_publisher->publish(pose);
    g_last_arm_down_publish_time = now;
    g_arm_down_publish_time_initialised = true;
}

static void publishInteractionActive(
    const rclcpp::Time &now,
    bool active,
    bool force = false)
{
    if (!g_interaction_active_publisher)
    {
        return;
    }

    const bool value_changed = active != g_last_interaction_active_value;
    const bool period_elapsed =
        !g_interaction_active_publish_time_initialised ||
        (now - g_last_interaction_active_publish_time).seconds() >= 0.50;

    if (!force && !value_changed && !period_elapsed)
    {
        return;
    }

    std_msgs::msg::Bool msg;
    msg.data = active;
    g_interaction_active_publisher->publish(msg);
    g_last_interaction_active_publish_time = now;
    g_interaction_active_publish_time_initialised = true;
    g_last_interaction_active_value = active;
}

/**
 * @brief ROS 2 node implementing the MECHelangelo demo-mode behaviour.
 *
 * @details
 * Runs at **10 Hz** (100 ms wall-clock timer).  The robot base never moves;
 * this node only controls `/interaction_active`, `/arm/right_pose`,
 * `/arm/left_pose`, and voice playback.
 *
 * Typical use: launch with `demo_launch.py` when the robot is displayed
 * on a stand.  The Pi 4 bridge still needs to be running so that
 * `/human_detected` and `/human_tracking` are published.
 *
 * Subscribed topics:
 * | Topic | Type | Description |
 * |-------|------|-------------|
 * | `/human_detected` | `std_msgs/Bool` | Edge-trigger from Pi 4 bridge |
 * | `/human_tracking` | `std_msgs/Float32MultiArray` | Continuous camera state |
 * | `/arm/mimicry_right_pose` | `std_msgs/String` | Classified right-arm pose |
 * | `/arm/mimicry_left_pose` | `std_msgs/String` | Classified left-arm pose |
 *
 * Published topics:
 * | Topic | Type | Description |
 * |-------|------|-------------|
 * | `/cmd_vel` | `geometry_msgs/Twist` | Always zero (robot does not move) |
 * | `/arm/right_pose` | `std_msgs/String` | Named rest pose or forwarded mimicry pose |
 * | `/arm/left_pose` | `std_msgs/String` | Named rest pose or forwarded mimicry pose |
 * | `/interaction_active` | `std_msgs/Bool` | `true` while mimicry session is active |
 */
class MechelangeloDemoBehaviour : public rclcpp::Node
{
public:
    /** @brief Construct the node, create publishers/subscribers, start the 100 ms timer. */
    MechelangeloDemoBehaviour();

    /** @brief Stop the node cleanly and log shutdown. */
    ~MechelangeloDemoBehaviour();

private:
    /** @brief 10 Hz control loop — manages session timing, voice, and arm keepalive. */
    void controlLoop();

    /**
     * @brief Edge-trigger callback for human lock acquire events.
     *
     * @param msg  `true` = camera acquired a person; `false` = ignored (loss
     *             is handled in `controlLoop()` via the tracking timeout).
     *
     * Transitions from SEARCHING to INTERACTION if not in cooldown.
     */
    void humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief Continuous tracking heartbeat from the Pi 4 bridge.
     *
     * @param msg  `[detected, centre_offset, dist_m, tracking_valid, grace_active]`.
     *             Updates `human_locked_`, `human_tracking_valid_`, and
     *             `human_tracking_grace_active_`.
     *
     * Also transitions from SEARCHING to INTERACTION on a rising edge, so that
     * the `/human_tracking` path alone (without `/human_detected`) is sufficient
     * to start an interaction.
     */
    void humanTrackingCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief Transition into INTERACTION state.
     * @param now  Current ROS time.
     *
     * Publishes `interaction_active = true`, plays a greeting voice line,
     * and sets `demo_state_` to `DemoState::INTERACTION`.
     */
    void startInteraction(const rclcpp::Time &now);

    /**
     * @brief Return to SEARCHING from INTERACTION.
     * @param now           Current ROS time.
     * @param with_cooldown If `true`, starts a 10-second detection cooldown
     *                      so the robot does not immediately re-enter INTERACTION.
     *
     * Publishes `interaction_active = false`, holds arms in rest pose, and
     * resets all session state.
     */
    void endInteraction(const rclcpp::Time &now, bool with_cooldown);

    /** @brief Publisher for `/cmd_vel` — always publishes zero in demo mode. */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    /** @brief Subscriber for the human lock edge-trigger on `/human_detected`. */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_detected_subscriber_;

    /** @brief Subscriber for the continuous tracking heartbeat on `/human_tracking`. */
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr human_tracking_subscriber_;

    /** @brief 100 ms wall-clock timer driving `controlLoop()`. */
    rclcpp::TimerBase::SharedPtr control_timer_;

    /** @brief Current demo FSM state (SEARCHING or INTERACTION). */
    DemoState demo_state_;

    /** @brief `true` while the camera reports a detected / locked human. */
    bool human_locked_;

    /** @brief `true` only when the camera has live visual tracking data
     *         (i.e. the skeleton is visible in the current frame). */
    bool human_tracking_valid_;

    /** @brief `true` while the camera is in grace-period reacquisition
     *         (the person was briefly lost but the lock is still held). */
    bool human_tracking_grace_active_;

    /** @brief Normalised horizontal image offset of the detected human.
     *         −0.5 = left edge, 0 = centred, +0.5 = right edge.
     *         Not used for steering in demo mode, but kept for consistency
     *         with the full behaviour node interface. */
    double human_centre_offset_;

    /** @brief ROS time of the most recent `/human_tracking` message.
     *         Used to detect tracking timeout in the INTERACTION state. */
    rclcpp::Time last_human_tracking_time_;
};

// Helper: transition into INTERACTION state.
void MechelangeloDemoBehaviour::startInteraction(const rclcpp::Time &now)
{
    playVoiceLine(VoiceGroup::PERSON_DETECTED, now, 1.0, true, false, true);
    g_interaction_session_active = true;
    g_interaction_session_start_time = now;
    g_person_lost_voice_played = false;
    g_last_right_mimicry_voice_pose.clear();
    g_last_left_mimicry_voice_pose.clear();
    demo_state_ = DemoState::INTERACTION;
    publishInteractionActive(now, true, true);
    RCLCPP_INFO(this->get_logger(), "DEMO: Human detected — starting interaction.");
}

// Helper: return to SEARCHING from INTERACTION.
void MechelangeloDemoBehaviour::endInteraction(
    const rclcpp::Time &now, bool with_cooldown)
{
    g_interaction_session_active = false;
    if (with_cooldown)
    {
        g_human_detection_cooldown_active = true;
        g_human_detection_cooldown_start_time = now;
    }
    human_locked_ = false;
    human_tracking_valid_ = false;
    human_tracking_grace_active_ = false;
    g_last_visible_human_valid = false;
    g_last_right_mimicry_voice_pose.clear();
    g_last_left_mimicry_voice_pose.clear();
    demo_state_ = DemoState::SEARCHING;
    publishInteractionActive(now, false, true);
    publishArmsDown(now, true);
}

MechelangeloDemoBehaviour::MechelangeloDemoBehaviour()
: Node("mechelangelo_demo_behaviour"),
  demo_state_(DemoState::SEARCHING),
  human_locked_(false),
  human_tracking_valid_(false),
  human_tracking_grace_active_(false),
  human_centre_offset_(0.0)
{
    g_right_arm_pose_publisher =
        this->create_publisher<std_msgs::msg::String>("/arm/right_pose", 10);
    g_left_arm_pose_publisher =
        this->create_publisher<std_msgs::msg::String>("/arm/left_pose", 10);
    g_interaction_active_publisher =
        this->create_publisher<std_msgs::msg::Bool>(
            "/interaction_active",
            rclcpp::QoS(1).reliable().transient_local());

    // Forward mimicry poses to the arms only while actively interacting.
    g_right_mimicry_pose_subscriber =
        this->create_subscription<std_msgs::msg::String>(
            "/arm/mimicry_right_pose",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active &&
                    human_tracking_valid_ &&
                    g_right_arm_pose_publisher)
                {
                    g_right_arm_pose_publisher->publish(*msg);
                    if (!msg->data.empty() &&
                        msg->data != g_last_right_mimicry_voice_pose)
                    {
                        if (playMimicryPoseVoice(msg->data, this->now()))
                        {
                            g_last_right_mimicry_voice_pose = msg->data;
                        }
                    }
                }
                else
                {
                    g_last_right_mimicry_voice_pose.clear();
                }
            });

    g_left_mimicry_pose_subscriber =
        this->create_subscription<std_msgs::msg::String>(
            "/arm/mimicry_left_pose",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active &&
                    human_tracking_valid_ &&
                    g_left_arm_pose_publisher)
                {
                    g_left_arm_pose_publisher->publish(*msg);
                    if (!msg->data.empty() &&
                        msg->data != g_last_left_mimicry_voice_pose)
                    {
                        if (playMimicryPoseVoice(msg->data, this->now()))
                        {
                            g_last_left_mimicry_voice_pose = msg->data;
                        }
                    }
                }
                else
                {
                    g_last_left_mimicry_voice_pose.clear();
                }
            });

    cmd_vel_publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    human_detected_subscriber_ =
        this->create_subscription<std_msgs::msg::Bool>(
            "/human_detected",
            10,
            std::bind(
                &MechelangeloDemoBehaviour::humanDetectedCallback,
                this,
                std::placeholders::_1));

    human_tracking_subscriber_ =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/human_tracking",
            10,
            std::bind(
                &MechelangeloDemoBehaviour::humanTrackingCallback,
                this,
                std::placeholders::_1));

    last_human_tracking_time_       = this->now();
    g_last_visible_human_time       = this->now();
    g_last_arm_down_publish_time    = this->now();
    g_arm_down_publish_time_initialised         = false;
    g_last_interaction_active_publish_time      = this->now();
    g_interaction_active_publish_time_initialised = false;
    g_last_interaction_active_value             = false;

    preloadVoiceLines();
    ensureVoiceWorkerStarted();

    control_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&MechelangeloDemoBehaviour::controlLoop, this));

    publishInteractionActive(this->now(), false, true);
    publishArmsDown(this->now(), true);

    RCLCPP_INFO(
        this->get_logger(),
        "Mechelangelo Demo Behaviour started — stationary, arm-only mode.");
}

MechelangeloDemoBehaviour::~MechelangeloDemoBehaviour()
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Demo Behaviour stopped.");
}

void MechelangeloDemoBehaviour::controlLoop()
{
    const rclcpp::Time now = this->now();

    // Robot never moves in demo mode — always publish zero velocity.
    geometry_msgs::msg::Twist zero_twist;
    cmd_vel_publisher_->publish(zero_twist);

    // Expire detection cooldown.
    if (g_human_detection_cooldown_active &&
        (now - g_human_detection_cooldown_start_time).seconds() >=
            kDetectionCooldownSeconds)
    {
        g_human_detection_cooldown_active = false;
    }

    // Publish interaction_active gate: true only while actively mimicking.
    const bool mimicking_now =
        g_interaction_session_active && human_tracking_valid_;
    publishInteractionActive(now, mimicking_now);

    // Keep arms down whenever we are not actively mimicking.
    if (!mimicking_now)
    {
        publishArmsDown(now);
    }

    switch (demo_state_)
    {
    case DemoState::SEARCHING:
    {
        // Cycle through searching ambient voice lines every 12 seconds.
        playVoiceLine(
            VoiceGroup::DEMO_SEARCHING, now, kSearchingVoiceIntervalSeconds);
        break;
    }

    case DemoState::INTERACTION:
    {
        const double time_since_tracking =
            (now - last_human_tracking_time_).seconds();
        const double time_since_visible = g_last_visible_human_valid
            ? (now - g_last_visible_human_time).seconds()
            : std::numeric_limits<double>::infinity();

        const bool visual_fresh =
            human_locked_ && human_tracking_valid_ &&
            time_since_tracking <= kHumanVisualFreshTimeout;

        const bool grace_fresh =
            human_locked_ && human_tracking_grace_active_ &&
            time_since_tracking <= kHumanVisualFreshTimeout;

        // Interaction timer: end session after the configured duration.
        const double interaction_elapsed =
            (now - g_interaction_session_start_time).seconds();

        if (interaction_elapsed >= kInteractionDurationSeconds)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "DEMO: Interaction complete (%.0f s). Returning to searching.",
                kInteractionDurationSeconds);
            endInteraction(now, true);
            break;
        }

        if (visual_fresh)
        {
            // Camera has the person — play random mimicry line every 12 s.
            playVoiceLine(
                VoiceGroup::MIMICRY_RANDOM, now, kMimicryVoiceIntervalSeconds);
        }
        else if (grace_fresh)
        {
            // Camera is reacquiring — hold arms down, play grace period lines.
            playVoiceLine(
                VoiceGroup::GRACE_PERIOD, now, kGracePeriodVoiceIntervalSeconds);
            g_last_right_mimicry_voice_pose.clear();
            g_last_left_mimicry_voice_pose.clear();

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "DEMO: Camera reacquiring person — holding arms.");
        }
        else if (time_since_visible > kHumanLostTimeout)
        {
            // Person is truly gone — play lost line once, then return to searching.
            if (!g_person_lost_voice_played)
            {
                playVoiceLine(VoiceGroup::PERSON_LOST, now, 1.0, true);
                g_person_lost_voice_played = true;
            }
            RCLCPP_WARN(
                this->get_logger(),
                "DEMO: Person lost (%.1f s). Returning to searching.",
                time_since_visible);
            endInteraction(now, true);
        }
        else
        {
            // Briefly lost — hold and wait for reacquisition.
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "DEMO: Camera target temporarily lost — waiting for reacquisition.");
        }
        break;
    }

    default:
        break;
    }
}

void MechelangeloDemoBehaviour::humanDetectedCallback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
    {
        return;
    }

    if (g_human_detection_cooldown_active)
    {
        const double elapsed =
            (this->now() - g_human_detection_cooldown_start_time).seconds();
        if (elapsed < kDetectionCooldownSeconds)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "DEMO: Human detection ignored — cooldown %.1f s remaining.",
                kDetectionCooldownSeconds - elapsed);
            return;
        }
        g_human_detection_cooldown_active = false;
    }

    if (demo_state_ != DemoState::INTERACTION)
    {
        startInteraction(this->now());
    }
}

void MechelangeloDemoBehaviour::humanTrackingCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Invalid /human_tracking message — expected [detected, offset, dist].");
        return;
    }

    const rclcpp::Time now = this->now();
    const bool detected = msg->data[0] > 0.5F;
    const bool tracking_valid =
        msg->data.size() >= 4 ? msg->data[3] > 0.5F : detected;
    const bool grace_active =
        msg->data.size() >= 5 ? msg->data[4] > 0.5F : false;

    if (detected && g_human_detection_cooldown_active)
    {
        const double elapsed =
            (now - g_human_detection_cooldown_start_time).seconds();
        if (elapsed < kDetectionCooldownSeconds)
        {
            human_locked_ = false;
            human_tracking_valid_ = false;
            human_tracking_grace_active_ = false;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "DEMO: Tracking detection ignored — cooldown %.1f s remaining.",
                kDetectionCooldownSeconds - elapsed);
            return;
        }
        g_human_detection_cooldown_active = false;
    }

    human_locked_                = detected;
    human_tracking_valid_        = detected && tracking_valid;
    human_tracking_grace_active_ = detected && grace_active;
    human_centre_offset_         =
        human_tracking_valid_ ? static_cast<double>(msg->data[1]) : 0.0;
    last_human_tracking_time_    = now;

    if (!detected)
    {
        human_tracking_valid_        = false;
        human_tracking_grace_active_ = false;
        return;
    }

    // Update last confirmed visual time.
    if (tracking_valid)
    {
        g_last_visible_human_valid = true;
        g_last_visible_human_time  = now;
    }

    // Trigger interaction if not already active.
    if (demo_state_ != DemoState::INTERACTION)
    {
        startInteraction(now);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MechelangeloDemoBehaviour>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
