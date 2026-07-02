/**
 * @file behaviour_physical.cpp
 * @brief Physical robot implementation of `MechelangeloBehaviour`.
 *
 * @details
 * This is the behaviour file used by the real robot through
 * `mechelangelo_behaviour_physical`. It keeps the same public class as
 * `behaviour.hpp`, but the internal logic is tuned for the actual base,
 * sensors, Pi 4 camera bridge, arm servos, and voice playback.
 *
 * Read this file when changing how the real robot moves around people or
 * walls. Read `behaviour.cpp` when working on the Gazebo/simulation planner.
 *
 * ### Why the physical file is different
 *
 * | Physical constraint | Code response |
 * |---|---|
 * | The base cannot reliably turn below about 0.375 rad/s. | Human centring uses short on/off turn pulses instead of tiny continuous turns. |
 * | The camera has bearing but no trustworthy depth. | LiDAR, not camera distance, decides when to slow and stop. |
 * | Gallery surfaces can create noisy LiDAR speckles. | `filterLaserScan()` uses local-neighbour, segment, and temporal-vote filtering. |
 * | The robot can get stuck spinning near walls. | `kAutonomousMaximumTurnDuration` reverses timed DVD turns after too long. |
 * | Arms should not move freely while driving. | `publishArmsDown()` keeps both arms in the rest pose until interaction starts. |
 * | Audio playback must not block the 10 Hz control loop. | Voice files are queued and played on a background worker thread. |
 *
 * ### File structure
 *
 * | Section | What it contains |
 * |---|---|
 * | Constants | Physical speeds, LiDAR filter values, human approach distances, timing, voice intervals. |
 * | File-scope state | Human approach phase, pulse timers, cooldown/session flags, arm publishers, voice queue. |
 * | Enums | `HumanMotionPhase` for approach/interact/recovery and `VoiceGroup` for audio categories. |
 * | Voice helpers | Audio discovery, phrase grouping, shell-safe playback command, worker thread, rate limiting. |
 * | Arm/interact helpers | `publishArmsDown()`, `publishInteractionActive()`, mimicry pose forwarding. |
 * | Controller helpers | `resetHumanMotionController()`, cooldown checks, phase-name helpers. |
 * | Class methods | Constructor, `run()`, `blindAutonomous()`, `controlLoop()`, callbacks, LiDAR helpers, RViz markers. |
 *
 * ### Full control flow
 *
 * @code
 * physical_autonomous.launch.py
 *   -> starts lidar_driver, imu_driver, base_driver, pi4_bridge
 *   -> starts mechelangelo_behaviour_physical
 *
 * MechelangeloBehaviour()
 *   -> creates ROS interfaces
 *   -> creates arm and interaction publishers
 *   -> subscribes to mimicry pose topics
 *   -> starts the 100 ms timer
 *
 * controlLoop()
 *   -> SEARCHING: choose a safe DVD-bounce direction
 *   -> ALIGNING: rotate until the front LiDAR cone is clear
 *   -> MOVING: drive forward until an obstacle enters stop_distance_m_
 *   -> STOPPED: hold still briefly, then search again
 *   -> HUMAN_DETECTED: centre, approach, settle, interact, recover, or cooldown
 * @endcode
 *
 * ### Main functions and how they interact
 *
 * | Function/helper | What it does | Key variables changed |
 * |---|---|---|
 * | `MechelangeloBehaviour()` | Builds the physical ROS node. | Publishers/subscribers, `stop_distance_m_`, `g_*_publisher` helpers. |
 * | `run(false)` | Starts physical autonomous mode. | Calls `blindAutonomous()` and spins. |
 * | `blindAutonomous()` | Returns to DVD exploration. | `current_state_`, `blind_autonomous_active_`, safety flags, arm-down state. |
 * | `controlLoop()` | Main 10 Hz state machine. | `current_state_`, `current_twist_`, human phase/session flags. |
 * | `laserScanCallback()` | Filters and stores LiDAR scans. | `latest_scan_`, `latest_segments_`, `scan_history_`, `/scan_filtered`. |
 * | `humanTrackingCallback()` | Stores Pi 4 camera state. | `human_locked_`, `human_centre_offset_`, `human_tracking_valid_`, grace flags. |
 * | `humanDetectedCallback()` | Handles human detection edge events. | Enters `HUMAN_DETECTED` if cooldown allows. |
 * | `filterLaserScan()` | Removes isolated, tiny, and intermittent LiDAR returns. | Returns the scan used by all obstacle decisions. |
 * | `getHumanLidarRange()` | Finds range near the camera-estimated human bearing. | Used by approach stop/slowdown logic. |
 * | `publishArmsDown()` | Keeps arms safe outside interaction. | Publishes `/arm/right_pose` and `/arm/left_pose`. |
 * | `publishInteractionActive()` | Opens/closes mimicry. | Publishes `/interaction_active`; updates last published value/time. |
 * | `playVoiceLine()` and helpers | Queue audio without blocking control. | `g_voice_queue`, voice timing maps, `g_voice_playing`. |
 *
 * ### Human interaction phases
 *
 * `current_state_ == NavigationState::HUMAN_DETECTED` is the top-level state.
 * Inside that state, `g_human_motion_phase` tracks the smaller physical
 * approach state machine:
 *
 * | Phase | What the robot does | Exit condition |
 * |---|---|---|
 * | `APPROACH` | Uses camera offset to turn in pulses, then drives forward using LiDAR range. | Person is centred and at stop distance. |
 * | `SETTLE` | Holds still and checks several good samples. | Enough centred/stopped samples are collected. |
 * | `INTERACTION` | Publishes `/interaction_active = true` and forwards arm mimicry poses. | 30 second interaction timer ends. |
 * | `RECOVERY` | Holds still while the camera reacquires a lost person. | Camera reacquires or timeout returns to exploration. |
 *
 * ### Important physical variables
 *
 * | Variable | Meaning | Usually changed by |
 * |---|---|---|
 * | `g_human_motion_phase` | Current phase inside human interaction. | `controlLoop()`, `resetHumanMotionController()`. |
 * | `g_human_centre_locked` | Whether the camera offset is centred enough to drive forward. | Human approach branch in `controlLoop()`. |
 * | `g_human_turn_cycle_start` | Start time for the current on/off turn pulse. | Human centring logic. |
 * | `g_human_forward_cycle_start` | Start time for pulsed forward slowdown near the person. | Human approach slowdown logic. |
 * | `g_interaction_session_active` | Whether mimicry is currently open. | SETTLE/INTERACTION branches. |
 * | `g_human_detection_cooldown_active` | Whether new detections are temporarily ignored. | Interaction end/recovery logic. |
 * | `g_last_visible_human_*` | Last valid camera observation used for recovery. | `humanTrackingCallback()`. |
 * | `g_right_arm_pose_publisher`, `g_left_arm_pose_publisher` | File-scope arm publishers used by helper functions. | Constructor initialises them; helpers publish through them. |
 * | `g_voice_queue`, `g_voice_playing`, `g_last_voice_request_times` | Voice playback state. | Voice helper functions and worker thread. |
 *
 * ### Tuning constants that matter most
 *
 * | Constant | Effect |
 * |---|---|
 * | `kForwardSpeed` | DVD-bounce cruise speed. |
 * | `kTurnSpeed` | Exploration turn command. |
 * | `kAutonomousMaximumTurnDuration` | Maximum time spent turning before reversing direction. |
 * | `kHumanApproachSpeed` | Forward speed while approaching a person. |
 * | `kHumanApproachMaxTurnSpeed` | Physical turn pulse speed. |
 * | `kHumanLidarStopDistance` | LiDAR range where the robot stops approaching. |
 * | `kHumanLidarSlowdownMargin` | Distance band before stop where forward motion becomes pulsed. |
 * | `kHumanInteractionDurationSeconds` | Length of the mimicry session. |
 * | `kHumanDetectionCooldownSeconds` | Time after interaction before another detection is accepted. |
 *
 * @see behaviour_code_guide for the package-level behaviour guide.
 */

#include "behaviour.hpp"

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
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------
// Behaviour constants
// ------------------------------------------------------

// Control loop runs every 100 ms.
static constexpr double kControlPeriodSeconds = 0.1;

// Physical movement tuning.
// The base was tested and 0.375 rad/s is the slowest reliable in-place turn.
// Autonomous DVD turns therefore use that fixed command instead of requesting
// smaller values that leave the motors stalled.
static constexpr double kForwardSpeed = 0.15;       // m/s, physical DVD cruise
static constexpr double kTurnSpeed = 0.425;         // rad/s, below the 0.45 driver cap
static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees

// Smooth commanded stops so the physical base does not snap from motion to zero.
static constexpr double kStopLinearDecel = 0.16;  // m/s^2
static constexpr double kStopAngularDecel = 0.45; // rad/s^2

// Ignore returns too close to the robot body / lidar blind spot.
static constexpr double kMinValidRange = 0.5; // m

// Front scan window used while moving forward.
static constexpr double kFrontCheckAngle = 30 * M_PI / 180.0; // +/- 30 degrees

// When several neighbouring beams are effectively tied for the longest
// distance, steer toward the middle of that opening instead of whichever beam
// happens to appear first in the scan array.
static constexpr double kLongestRangeTieTolerance = 0.05; // m

// ------------------------------------------------------
// DVD-style bounce exploration tuning
// ------------------------------------------------------
// The autonomous gallery behaviour is intended to move like a DVD screensaver:
// drive straight until the front is blocked, stop at a safe distance, then pick
// a new safe side-bounce angle instead of always chasing the longest wall.
//
// A beam is considered open if it is infinity OR farther than this clearance.
// Infinity is useful in large rooms because it means the LiDAR did not hit
// anything within range.
static constexpr double kDvdOpenClearanceDistance = 4.5; // m

// Ignore very narrow open gaps. This prevents the robot from aiming through
// thin laser cracks between obstacles.
static constexpr double kDvdMinSectorWidth = 18.0 * M_PI / 180.0; // radians

// Trim candidate angles away from the edge of an open sector.
static constexpr double kDvdSectorEdgeMargin = 8.0 * M_PI / 180.0; // radians

// Preferred bounce band after the robot stops at a wall.
// These angles are relative to the robot's current forward direction:
//   0 deg   = back into the wall/obstacle it just stopped for
//   +/-90   = side-bounce
//   +/-180  = drive exactly back along the previous path
// The robot randomly chooses inside this safe side band when possible.
static constexpr double kDvdPreferredMinTurnAngle = 55.0 * M_PI / 180.0;  // radians
static constexpr double kDvdPreferredMaxTurnAngle = 150.0 * M_PI / 180.0; // radians

// Fallback band used if the preferred side-bounce band has no safe candidates.
// This still avoids the front wall and avoids exact reverse.
static constexpr double kDvdAvoidFrontAngle = 35.0 * M_PI / 180.0;  // radians
static constexpr double kDvdAvoidReverseAngle = 165.0 * M_PI / 180.0; // radians

static constexpr double kAutonomousMaximumTurnDuration = 7.0; // hard anti-circle limit
static rclcpp::Time g_autonomous_timed_turn_start_time;
static int g_autonomous_last_dvd_turn_sign = 1;

// ------------------------------------------------------
// LaserScan noise suppression constants
// ------------------------------------------------------
// The filter is based on the previous LaserProcessing::countSegments()
// approach: valid objects form segments of neighbouring points. Random
// dots are usually isolated or only one/two points, so they get replaced
// with infinity and will not stop the robot.

// Stage 1: local neighbour test.
static constexpr int kNoiseNeighbourWindow = 5;          // check +/- 4 beams
static constexpr int kNoiseMinNeighbourCount = 1;        // require at least 2 close neighbours
static constexpr double kNoiseNeighbourDistance = 0.25;  // m in local XY space

// Stage 2: segment extraction test.
static constexpr double kSegmentJoinDistance = 0.28;     // m max gap between consecutive points
static constexpr int kSegmentMinPoints = 3;              // reject tiny speckle clusters
static constexpr double kSegmentMinLength = 0.03;        // m reject near-zero length segments

// Stage 3: temporal majority vote across a ring buffer of recent scans.
static constexpr int kTemporalBufferSize = 4;            // number of scans to accumulate
static constexpr int kTemporalVoteThreshold = 2;         // minimum votes (out of 5) to keep a beam

// ------------------------------------------------------
// Human tracking tuning
// ------------------------------------------------------
// /human_tracking message format:
// data[0] = detected, data[1] = centre_offset, data[2] = distance_m
// centre_offset is normalised image offset from centre: -0.5 left, 0 centre, +0.5 right.
static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
static constexpr double kHumanLostTimeout = 1.5;        // seconds, wait stationary before returning to DVD exploration

// ------------------------------------------------------
// Physical human approach: camera bearing + LiDAR range
// ------------------------------------------------------
// Camera centre_offset controls steering; LiDAR controls forward speed and stop.
static constexpr double kHumanApproachSpeed = 0.12;          // m/s, tested reliable approach speed
static constexpr double kHumanApproachMaxTurnSpeed = 0.375;  // rad/s, slowest reliable physical turn
static constexpr double kHumanCentreReleaseZone = 0.11;      // re-centre if target drifts beyond this
static constexpr double kHumanTurnPulseOnSeconds = 0.22;     // short correction pulse
static constexpr double kHumanTurnPulseOffSeconds = 0.28;    // stationary camera update period
static constexpr double kHumanForwardPulseOnSeconds = 0.20;  // slowdown band: 0.12 m/s pulse on
static constexpr double kHumanForwardPulseOffSeconds = 0.40; // slowdown band: decel pause
static constexpr double kHumanInteractionMaxOffset = 0.08;   // must be centred before interaction
static constexpr int kHumanInteractionSettleSamples = 3;     // 0.3 s at 10 Hz
static constexpr double kHumanInteractionMaxLinearSpeed = 0.03; // m/s
// If the base is stopped at distance but camera alignment never becomes valid,
// abandon the attempt instead of waiting indefinitely.
static constexpr double kHumanFailedStopTimeoutSeconds = 10.0;

// LiDAR validation for human distance.
static constexpr double kCameraHorizontalFov = 60.0 * M_PI / 180.0;
static constexpr double kHumanLidarWindow = 10.0 * M_PI / 180.0;
// Stop distance raised from 1.65m to 1.80m to account for:
//   - Deceleration overshoot (~30mm at 0.20 m/s², ~66mm at the old 0.10 m/s²)
//   - Temporal LiDAR filter lag (~10–20mm at 0.12 m/s closing speed)
//   - Physical motor/wheel inertia (~10–20mm additional coast after command)
// Combined worst-case pushes the actual stop point ~50–60mm past the commanded
// threshold, so 1.80m commanded gives ~1.72–1.75m physical stopping distance —
// comfortably above the 1.50m ultrasonic emergency threshold.
static constexpr double kHumanLidarStopDistance = 1.80;
// LiDAR is the primary human-approach stopping sensor. kHumanLidarSlowdownMargin
// is added to kHumanLidarStopDistance to define the pulsed-slowdown band start.
static constexpr double kHumanLidarSlowdownMargin = 0.25;  // m above stop to begin slowdown pulses
// Deceleration rate when the stop condition is reached. Increased from 0.10 to
// 0.20 m/s² to halve the commanded overshoot (0.030m vs 0.066m at 0.12 m/s).
static constexpr double kHumanApproachDecelRate = 0.20;    // m/s²

// Human interaction session timing. Once a valid interaction pose is reached,
// hold interaction for this long, then return to DVD exploration and ignore
// camera-triggered human interrupts for a short cooldown period.
static constexpr double kHumanInteractionDurationSeconds = 30.0;
static constexpr double kHumanDetectionCooldownSeconds = 10.0;

// Camera tracking freshness. The camera supplies bearing/visibility; LiDAR
// supplies all range decisions.
static constexpr double kHumanVisualFreshTimeout = 0.60;

// Keep both arms in the named arm_down pose whenever the robot is exploring,
// approaching or settling. Mimicry takes control only after the internal
// interaction state has started.
static constexpr double kArmDownRepublishPeriod = 0.50;

// Last confirmed visual human observation. Kept file-scope so this patch does not
// require behaviour.hpp changes. Used to pause/reacquire instead of immediately
// dropping back to DVD exploration when the camera briefly loses the person.
static bool g_last_visible_human_valid = false;
static double g_last_visible_human_centre_offset = 0.0;
static double g_last_visible_human_distance_m = -1.0;
static rclcpp::Time g_last_visible_human_time;

// File-scope interaction session/cooldown and long-run stats. Keeping these
// outside the class avoids requiring behaviour.hpp changes for this test.
static bool g_interaction_session_active = false;
static rclcpp::Time g_interaction_session_start_time;
static bool g_human_detection_cooldown_active = false;
static rclcpp::Time g_human_detection_cooldown_start_time;

/**
 * @brief Sub-states used inside the `HUMAN_DETECTED` FSM state.
 *
 * When the camera reports a person, the node enters HUMAN_DETECTED and
 * progresses through these phases in order.  Only INTERACTION causes
 * `/interaction_active` to be published as `true`, which gates the mimicry
 * arm commands from the camera.
 */
enum class HumanMotionPhase
{
    /** @brief Robot is turning and/or driving toward the detected person.
     *         Centering pulses and forward pulses are both active here. */
    APPROACH,

    /** @brief Robot is at the correct distance and offset, accumulating
     *         confirmation samples (`kHumanInteractionSettleSamples` = 3)
     *         before committing to interaction. */
    SETTLE,

    /** @brief Interaction session is active.  Base is stationary.
     *         Arm mimicry commands are forwarded from the camera.
     *         Duration is `kHumanInteractionDurationSeconds` (30 s). */
    INTERACTION,

    /** @brief Camera briefly lost the person or is in grace-period reacquisition.
     *         Robot holds still and waits up to `kHumanLostTimeout` (1.5 s)
     *         before returning to DVD exploration. */
    RECOVERY
};

static HumanMotionPhase g_human_motion_phase = HumanMotionPhase::APPROACH;

// Named arm-pose keepalive. These are file-scope so behaviour.hpp does not need
// new publisher members for this test.
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_right_arm_pose_publisher;
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_left_arm_pose_publisher;
static rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr g_interaction_active_publisher;
static rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_voice_prompt_publisher;
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_right_mimicry_pose_subscriber;
static rclcpp::Subscription<std_msgs::msg::String>::SharedPtr g_left_mimicry_pose_subscriber;
static int g_arm_mimicry_forward_count = 0;
static int g_arm_mimicry_ignored_count = 0;
static rclcpp::Time g_last_arm_down_publish_time;
static bool g_arm_down_publish_time_initialised = false;
static rclcpp::Time g_last_interaction_active_publish_time;
static bool g_interaction_active_publish_time_initialised = false;
static bool g_last_interaction_active_value = false;
static std::string g_approach_arm_pose_name = "arm_down";

static int g_human_settle_good_samples = 0;
static bool g_human_stop_wait_active = false;
static rclcpp::Time g_human_stop_wait_start;

// Simple physical human-centering state. The base cannot turn below 0.375 rad/s,
// so small visual corrections are issued as short pulses separated by a
// stationary camera-update period.
static bool g_human_centre_locked = false;
static bool g_human_turn_cycle_initialised = false;
static rclcpp::Time g_human_turn_cycle_start;
static bool g_human_forward_cycle_initialised = false;
static rclcpp::Time g_human_forward_cycle_start;

/**
 * @brief Categories of voice lines used by the voice playback system.
 *
 * Each group maps to a sub-folder inside the `resources/` directory.
 * Audio files are pre-loaded at startup into phrase groups (groups of
 * variations on the same phrase).  `playVoiceLine()` cycles through
 * phrase groups in order and picks a random variation within each group,
 * so the robot does not repeat the exact same audio file back-to-back.
 *
 * New audio files can be added by dropping `.mp3` / `.wav` files into
 * the matching sub-folder — no code changes required.
 */
enum class VoiceGroup
{
    /** @brief Played during DVD-bounce exploration (every 30 s). */
    AUTONOMOUS,

    /** @brief Played once when the camera first locks onto a person. */
    PERSON_DETECTED,

    /** @brief Played once when the camera loses a person. */
    PERSON_LOST,

    /** @brief Played while the camera is in grace-period reacquisition. */
    GRACE_PERIOD,

    /** @brief Played while the robot is re-centering on the person during approach. */
    CENTERING,

    /** @brief Played when an unexpected object enters the safety zone. */
    SAFETY_ZONE,

    /** @brief Random mimicry line played during the interaction session. */
    MIMICRY_RANDOM,

    /** @brief Triggered when the right arm classifies a "salute" pose. */
    MIMICRY_ATTEN_HUT,

    /** @brief Triggered when the arm classifies an "arm_90_up" or "straight_arm" pose. */
    MIMICRY_STRONG,

    /** @brief First part of the handshake greeting sequence. */
    MIMICRY_HANDSHAKE_GREETING,

    /** @brief Second part of the handshake greeting ("My name is MECHelangelo"). */
    MIMICRY_HANDSHAKE_NAME
};

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

/**
 * @brief Wrap a string in POSIX single-quotes, escaping any embedded
 *        single-quote characters, so the result is safe to embed in a
 *        shell command string passed to `std::system()`.
 *
 * @param value  The string to quote (typically a file path).
 * @return       The quoted string, e.g. `hello 'world'` → `'hello '\''world'\'''`.
 */
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

/**
 * @brief Return the absolute path to the `resources/` folder inside the
 *        installed `mechelangelo_behaviour` package share directory.
 *
 * @details
 * Tries `ament_index_cpp::get_package_share_directory()` first (works after
 * `colcon build` and sourcing the workspace).  Falls back to a hard-coded
 * path relative to `$HOME` and then an absolute fallback for development
 * environments where the package is not installed.
 *
 * The resources folder contains sub-folders of audio files for each
 * `VoiceGroup`, e.g. `resources/autonomous/`, `resources/person detected/`.
 *
 * @return Absolute path to the `resources/` directory as a std::string.
 */
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

static const std::vector<std::string> &voiceLinesForGroup(VoiceGroup group)
{
    static const std::vector<std::string> autonomous_lines =
        audioFilesInFolder("autonomous");
    static const std::vector<std::string> person_detected_lines =
        audioFilesInFolder("person detected");
    static const std::vector<std::string> person_lost_lines =
        audioFilesInFolder("person lost");
    static const std::vector<std::string> grace_period_lines =
        audioFilesInFolder("grace period");
    static const std::vector<std::string> centering_lines =
        audioFilesInFolder("centre adjusting");
    static const std::vector<std::string> safety_zone_lines =
        audioFilesInFolder("person in safety zone");
    static const std::vector<std::string> mimicry_random_lines =
        audioFilesInFolder(
            "mimicing",
            {},
            {"atten hut", "i am strong"});
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
    case VoiceGroup::PERSON_DETECTED:
        return person_detected_lines;
    case VoiceGroup::PERSON_LOST:
        return person_lost_lines;
    case VoiceGroup::GRACE_PERIOD:
        return grace_period_lines;
    case VoiceGroup::CENTERING:
        return centering_lines;
    case VoiceGroup::SAFETY_ZONE:
        return safety_zone_lines;
    case VoiceGroup::MIMICRY_RANDOM:
        return mimicry_random_lines;
    case VoiceGroup::MIMICRY_ATTEN_HUT:
        return mimicry_atten_hut_lines;
    case VoiceGroup::MIMICRY_STRONG:
        return mimicry_strong_lines;
    case VoiceGroup::MIMICRY_HANDSHAKE_GREETING:
        return mimicry_handshake_greeting_lines;
    case VoiceGroup::MIMICRY_HANDSHAKE_NAME:
        return mimicry_handshake_name_lines;
    case VoiceGroup::AUTONOMOUS:
    default:
        return autonomous_lines;
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
    static const std::vector<std::vector<std::string>> autonomous_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::AUTONOMOUS));
    static const std::vector<std::vector<std::string>> person_detected_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::PERSON_DETECTED));
    static const std::vector<std::vector<std::string>> person_lost_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::PERSON_LOST));
    static const std::vector<std::vector<std::string>> grace_period_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::GRACE_PERIOD));
    static const std::vector<std::vector<std::string>> centering_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::CENTERING));
    static const std::vector<std::vector<std::string>> safety_zone_groups =
        buildPhraseGroups(voiceLinesForGroup(VoiceGroup::SAFETY_ZONE));
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
    case VoiceGroup::PERSON_DETECTED:
        return person_detected_groups;
    case VoiceGroup::PERSON_LOST:
        return person_lost_groups;
    case VoiceGroup::GRACE_PERIOD:
        return grace_period_groups;
    case VoiceGroup::CENTERING:
        return centering_groups;
    case VoiceGroup::SAFETY_ZONE:
        return safety_zone_groups;
    case VoiceGroup::MIMICRY_RANDOM:
        return mimicry_random_groups;
    case VoiceGroup::MIMICRY_ATTEN_HUT:
        return mimicry_atten_hut_groups;
    case VoiceGroup::MIMICRY_STRONG:
        return mimicry_strong_groups;
    case VoiceGroup::MIMICRY_HANDSHAKE_GREETING:
        return mimicry_handshake_greeting_groups;
    case VoiceGroup::MIMICRY_HANDSHAKE_NAME:
        return mimicry_handshake_name_groups;
    case VoiceGroup::AUTONOMOUS:
    default:
        return autonomous_groups;
    }
}

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
    std::uniform_int_distribution<std::size_t> distribution(
        0,
        versions.size() - 1);

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

static bool playVoiceLine(VoiceGroup group, const rclcpp::Time &now,
    double min_interval_seconds = 6.0, bool force = false,
    bool queue_when_busy = false, bool priority = false)
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
        (now - g_last_voice_request_times[timing_group]).seconds() <
            min_interval_seconds)
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

static bool playMimicryPoseVoice(
    const std::string &pose_name,
    const rclcpp::Time &now)
{
    if (pose_name == "salute")
    {
        return playVoiceLine(
            VoiceGroup::MIMICRY_ATTEN_HUT,
            now,
            1.0,
            false,
            true,
            true);
    }
    else if (pose_name == "arm_90_up" || pose_name == "straight_arm")
    {
        return playVoiceLine(
            VoiceGroup::MIMICRY_STRONG,
            now,
            1.0,
            false,
            true,
            true);
    }
    else if (pose_name == "handshake")
    {
        return playVoiceSequence(
            {
                VoiceGroup::MIMICRY_HANDSHAKE_GREETING,
                VoiceGroup::MIMICRY_HANDSHAKE_NAME,
            },
            now,
            1.0,
            false,
            true,
            true);
    }

    return false;
}

static void preloadVoiceLines()
{
    (void)voicePhraseGroupsForGroup(VoiceGroup::AUTONOMOUS);
    (void)voicePhraseGroupsForGroup(VoiceGroup::PERSON_DETECTED);
    (void)voicePhraseGroupsForGroup(VoiceGroup::PERSON_LOST);
    (void)voicePhraseGroupsForGroup(VoiceGroup::GRACE_PERIOD);
    (void)voicePhraseGroupsForGroup(VoiceGroup::CENTERING);
    (void)voicePhraseGroupsForGroup(VoiceGroup::SAFETY_ZONE);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_RANDOM);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_ATTEN_HUT);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_STRONG);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_HANDSHAKE_GREETING);
    (void)voicePhraseGroupsForGroup(VoiceGroup::MIMICRY_HANDSHAKE_NAME);
}

static const char *humanMotionPhaseName(HumanMotionPhase phase)
{
    switch (phase)
    {
    case HumanMotionPhase::APPROACH: return "APPROACH";
    case HumanMotionPhase::SETTLE: return "SETTLE";
    case HumanMotionPhase::INTERACTION: return "INTERACTION";
    case HumanMotionPhase::RECOVERY: return "RECOVERY";
    default: return "UNKNOWN";
    }
}

/**
 * @brief Publish the rest-pose name to both arm topics at most once every
 *        `kArmDownRepublishPeriod` seconds (or immediately if `force` is true).
 *
 * @details
 * This is called continuously during exploration, approach, and settle so
 * that the arm controller always has a fresh target even if messages are
 * dropped.  The rate limit avoids flooding the arm topic while still
 * guaranteeing a fresh command every 500 ms.
 *
 * The rest-pose name is set from the `approach_arm_pose_name` ROS parameter
 * (default: `"arm_down"`).
 *
 * @param now    Current ROS time (used to check the republish interval).
 * @param force  If `true`, publish immediately regardless of the interval.
 */
static void publishArmsDown(const rclcpp::Time &now, bool force = false)
{
    if (!g_right_arm_pose_publisher || !g_left_arm_pose_publisher)
    {
        return;
    }

    const bool period_elapsed = !g_arm_down_publish_time_initialised ||
        (now - g_last_arm_down_publish_time).seconds() >=
            kArmDownRepublishPeriod;

    if (!force && !period_elapsed)
    {
        return;
    }

    std_msgs::msg::String pose;
    pose.data = g_approach_arm_pose_name;
    g_right_arm_pose_publisher->publish(pose);
    g_left_arm_pose_publisher->publish(pose);
    g_last_arm_down_publish_time = now;
    g_arm_down_publish_time_initialised = true;
}

/**
 * @brief Publish the `/interaction_active` gate topic, rate-limited to
 *        once every 500 ms unless the value changes or `force` is true.
 *
 * @details
 * The `/interaction_active` topic is published with
 * `rclcpp::QoS(1).reliable().transient_local()` (latched), so new
 * subscribers always receive the current value on first connect.  The
 * 500 ms keepalive is still needed for nodes that miss the initial latched
 * message due to start-up timing.
 *
 * `true` = mimicry session is active → perception camera may forward arm poses.
 * `false` = arms should be in the rest pose.
 *
 * @param now     Current ROS time.
 * @param active  The value to publish.
 * @param force   If `true`, publish immediately regardless of rate limit.
 */
static void publishInteractionActive(
    const rclcpp::Time &now,
    bool active,
    bool force = false)
{
    if (!g_interaction_active_publisher)
    {
        return;
    }

    const bool value_changed =
        active != g_last_interaction_active_value;
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
 * @brief Reset all file-scope human-approach sub-state to its initial values.
 *
 * @details
 * Called when:
 * - A new human detection begins (rising edge on `/human_detected` or
 *   `/human_tracking`).
 * - An interaction session ends (time limit reached or person lost).
 * - The robot abandons an approach due to `kHumanFailedStopTimeoutSeconds`.
 *
 * Does NOT reset `g_interaction_session_active` or cooldown state — those
 * are managed separately by the callers.
 */
static void resetHumanMotionController()
{
    g_human_motion_phase = HumanMotionPhase::APPROACH;
    g_human_centre_locked = false;
    g_human_turn_cycle_initialised = false;
    g_human_forward_cycle_initialised = false;
    g_human_stop_wait_active = false;
    g_human_settle_good_samples = 0;
    g_last_right_mimicry_voice_pose.clear();
    g_last_left_mimicry_voice_pose.clear();
}

static bool humanDetectionCooldownActive(const rclcpp::Time &now)
{
    if (!g_human_detection_cooldown_active)
    {
        return false;
    }

    return (now - g_human_detection_cooldown_start_time).seconds() <
        kHumanDetectionCooldownSeconds;
}

static double humanDetectionCooldownRemaining(const rclcpp::Time &now)
{
    if (!g_human_detection_cooldown_active)
    {
        return 0.0;
    }

    const double elapsed = (now - g_human_detection_cooldown_start_time).seconds();
    return std::max(0.0, kHumanDetectionCooldownSeconds - elapsed);
}

MechelangeloBehaviour::MechelangeloBehaviour()
: Node("mechelangelo_behaviour"),
  human_locked_(false),
  human_tracking_valid_(false),
  human_tracking_grace_active_(false),
  human_centre_offset_(0.0),
  human_distance_m_(-1.0),
  blind_autonomous_active_(false),
  current_state_(NavigationState::SEARCHING),
  target_angle_(0.0),
  target_range_(0.0),
  stop_distance_m_(1.5),
  align_yaw_initialised_(false)
{
    this->declare_parameter("stop_distance_m", 1.5);
    stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();

    this->declare_parameter("approach_arm_pose_name", "arm_down");
    g_approach_arm_pose_name =
        this->get_parameter("approach_arm_pose_name").as_string();

    RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_m_);
    RCLCPP_INFO(this->get_logger(),
        "Non-interaction arm hold pose: %s",
        g_approach_arm_pose_name.c_str());

    g_right_arm_pose_publisher =
        this->create_publisher<std_msgs::msg::String>("/arm/right_pose", 10);
    g_left_arm_pose_publisher =
        this->create_publisher<std_msgs::msg::String>("/arm/left_pose", 10);
    g_interaction_active_publisher =
        this->create_publisher<std_msgs::msg::Bool>(
            "/interaction_active",
            rclcpp::QoS(1).reliable().transient_local());
    // Placeholder text topic for the future voice system. It is only used when
    // the human is too close and the robot cannot safely reverse.
    g_voice_prompt_publisher =
        this->create_publisher<std_msgs::msg::String>("/voice_prompt", 10);

    // The launch file remaps the state bridge's mimicry outputs to these raw
    // topics. Only verified interaction poses are forwarded to the real named
    // pose topics; inactive/default zero poses are ignored.
    g_right_mimicry_pose_subscriber =
        this->create_subscription<std_msgs::msg::String>(
            "/arm/mimicry_right_pose",
            10,
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active &&
                    g_human_motion_phase == HumanMotionPhase::INTERACTION &&
                    g_right_arm_pose_publisher)
                {
                    g_right_arm_pose_publisher->publish(*msg);
                    g_arm_mimicry_forward_count++;
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
                    g_arm_mimicry_ignored_count++;
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
                    g_human_motion_phase == HumanMotionPhase::INTERACTION &&
                    g_left_arm_pose_publisher)
                {
                    g_left_arm_pose_publisher->publish(*msg);
                    g_arm_mimicry_forward_count++;
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
                    g_arm_mimicry_ignored_count++;
                    g_last_left_mimicry_voice_pose.clear();
                }
            });

    g_last_arm_down_publish_time = this->now();
    g_arm_down_publish_time_initialised = false;
    g_last_interaction_active_publish_time = this->now();
    g_interaction_active_publish_time_initialised = false;
    g_last_interaction_active_value = false;
    preloadVoiceLines();
    ensureVoiceWorkerStarted();

    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&MechelangeloBehaviour::laserScanCallback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu",
        rclcpp::SensorDataQoS(),
        std::bind(&MechelangeloBehaviour::imuCallback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10);

    filtered_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_filtered",
        rclcpp::SensorDataQoS());

    obstacle_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/behaviour_obstacle_markers",
        10);

    human_detected_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/human_detected",
        10,
        std::bind(&MechelangeloBehaviour::humanDetectedCallback, this, std::placeholders::_1));

    human_tracking_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/human_tracking",
        10,
        std::bind(&MechelangeloBehaviour::humanTrackingCallback, this, std::placeholders::_1));

    last_human_tracking_time_ = this->now();
    g_last_visible_human_time = this->now();
    g_last_visible_human_valid = false;
    g_last_visible_human_centre_offset = 0.0;
    g_last_visible_human_distance_m = -1.0;

    control_timer_ = this->create_wall_timer(
        100ms,
        std::bind(&MechelangeloBehaviour::controlLoop, this));

    publishInteractionActive(this->now(), false, true);
    publishArmsDown(this->now(), true);
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been started.");
}

MechelangeloBehaviour::~MechelangeloBehaviour()
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node has been stopped.");
}

void MechelangeloBehaviour::run(bool sim_mode)
{
    RCLCPP_INFO(this->get_logger(), "Mechelangelo Behaviour Node is running.");

    if (sim_mode)
    {
        RCLCPP_INFO(this->get_logger(), "Running in simulation mode.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Running in real robot mode.");
    }

    blindAutonomous();
    rclcpp::spin(shared_from_this());
}

void MechelangeloBehaviour::blindAutonomous()
{
    RCLCPP_INFO(this->get_logger(), "Executing blind autonomous behaviour.");

    blind_autonomous_active_ = true;
    // DVD behaviour begins by driving straight. A bounce heading is selected
    // only after the filtered LiDAR reports a wall in front.
    current_state_ = NavigationState::MOVING;
    target_angle_ = 0.0;
    target_range_ = 0.0;
    align_yaw_initialised_ = false;
    g_human_settle_good_samples = 0;
    g_human_centre_locked = false;
    g_human_turn_cycle_initialised = false;
    resetHumanMotionController();
    clearObstacleMarkers();
}

void MechelangeloBehaviour::controlLoop()
{
    if (!blind_autonomous_active_)
    {
        return;
    }

    geometry_msgs::msg::Twist twist;

    // The behaviour node is the authoritative interaction gate. The perception
    // mimicry camera is only activated after the fused pose and arm-clearance
    // checks have entered the verified interaction session.
    publishInteractionActive(
        this->now(),
        g_interaction_session_active &&
            g_human_motion_phase == HumanMotionPhase::INTERACTION);

    // Keep the arms down until the verified interaction session starts.
    if (!g_interaction_session_active &&
        g_human_motion_phase != HumanMotionPhase::INTERACTION)
    {
        publishArmsDown(this->now());
    }

    // DVD exploration requires the heavily filtered LaserScan. Human approach
    // is deliberately camera + ultrasonic only and must not be blocked by LiDAR.
    if (current_state_ != NavigationState::HUMAN_DETECTED &&
        (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Waiting for valid filtered LaserScan data...");

        stopRobot(twist);
        current_twist_ = twist;
        cmd_vel_publisher_->publish(twist);
        return;
    }

    switch (current_state_)
    {
    case NavigationState::SEARCHING:
    {
        playVoiceLine(VoiceGroup::AUTONOMOUS, this->now(), 30.0);
        stopRobot(twist);
        clearObstacleMarkers();

        double longest_angle = 0.0;
        double longest_range = 0.0;

        if (!getLongestRange(longest_angle, longest_range))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "SEARCHING: No valid filtered LaserScan range found.");
            break;
        }

        target_angle_ = longest_angle;
        target_range_ = longest_range;
        align_yaw_initialised_ = false;
        g_autonomous_timed_turn_start_time = this->now();

        RCLCPP_INFO(
            this->get_logger(),
            "SEARCHING: Selected exploration heading at %.2f deg, representative range %.2f m",
            target_angle_ * 180.0 / M_PI,
            target_range_);

        current_state_ = NavigationState::ALIGNING;
        break;
    }

    case NavigationState::ALIGNING:
    {
        clearObstacleMarkers();
        twist.linear.x = 0.0;

        const rclcpp::Time now = this->now();

        if (!align_yaw_initialised_)
        {
            align_yaw_initialised_ = true;
            g_autonomous_timed_turn_start_time = now;
            if (std::fabs(target_angle_) <= kAlignmentTolerance)
            {
                target_angle_ = g_autonomous_last_dvd_turn_sign < 0
                    ? M_PI / 2.0
                    : -M_PI / 2.0;
            }
            g_autonomous_last_dvd_turn_sign = target_angle_ >= 0.0 ? 1 : -1;
        }

        std::vector<LaserSegment> blocking_segments;
        const bool blocked_by_segment =
            findBlockingObstaclesInFront(blocking_segments);
        const double front_range = getFrontRange();
        const bool lidar_front_open =
            !blocked_by_segment &&
            (std::isinf(front_range) || front_range > stop_distance_m_);

        if (lidar_front_open)
        {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            current_twist_ = twist;
            align_yaw_initialised_ = false;
            current_state_ = NavigationState::MOVING;

            RCLCPP_INFO(
                this->get_logger(),
                "ALIGNING: Front is open. Starting DVD forward movement. lidar=%.2f.",
                front_range);
            break;
        }

        const double elapsed =
            (now - g_autonomous_timed_turn_start_time).seconds();
        if (elapsed >= kAutonomousMaximumTurnDuration)
        {
            target_angle_ = -target_angle_;
            g_autonomous_last_dvd_turn_sign =
                target_angle_ >= 0.0 ? 1 : -1;
            g_autonomous_timed_turn_start_time = now;

            RCLCPP_WARN(
                this->get_logger(),
                "ALIGNING: Turned for %.1f s without finding open front. Reversing scan direction.",
                kAutonomousMaximumTurnDuration);
        }

        const double turn_cmd =
            (g_autonomous_last_dvd_turn_sign >= 0 ? kTurnSpeed : -kTurnSpeed);
        twist.angular.z = turn_cmd;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "ALIGNING: Rotating until front opens. direction=%s, lidar %.2f m, cmd %.3f rad/s.",
            g_autonomous_last_dvd_turn_sign >= 0 ? "left" : "right",
            front_range,
            twist.angular.z);
        break;
    }

    case NavigationState::MOVING:
    {
        std::vector<LaserSegment> blocking_segments;
        const bool blocked_by_segment = findBlockingObstaclesInFront(blocking_segments);
        const double front_range = getFrontRange();

        // Segment-based blocking is the main decision. This prevents a single
        // random dot from stopping the robot because the dot will not survive
        // the neighbour + segment filter.
        if (blocked_by_segment || front_range <= stop_distance_m_)
        {
            playVoiceLine(VoiceGroup::SAFETY_ZONE, this->now(), 15.0);
            if (blocking_segments.empty())
            {
                // Fallback marker if the range check caught something but no
                // segment was available. This should be rare after filtering.
                LaserSegment fallback;
                fallback.point_count = 1;
                fallback.min_range = front_range;
                fallback.midpoint.x = std::isfinite(front_range) ? front_range : stop_distance_m_;
                fallback.midpoint.y = 0.0;
                fallback.midpoint.z = 0.0;
                blocking_segments.push_back(fallback);
            }

            publishObstacleMarkers(blocking_segments);

            RCLCPP_WARN(
                this->get_logger(),
                "MOVING: Blocking obstacle detected in front. Front range = %.2f m. Turning to find open space.",
                front_range);

            stopRobot(twist);
            if (std::fabs(twist.linear.x) <= 1e-6 &&
                std::fabs(twist.angular.z) <= 1e-6)
            {
                double longest_angle = 0.0;
                double longest_range = 0.0;
                if (getLongestRange(longest_angle, longest_range))
                {
                    target_angle_ = longest_angle;
                    target_range_ = longest_range;
                }
                else
                {
                    target_angle_ = g_autonomous_last_dvd_turn_sign < 0
                        ? M_PI / 2.0
                        : -M_PI / 2.0;
                    target_range_ = front_range;
                }

                align_yaw_initialised_ = false;
                current_state_ = NavigationState::ALIGNING;
            }
            break;
        }

        clearObstacleMarkers();

        if (std::isinf(front_range))
        {
            playVoiceLine(VoiceGroup::AUTONOMOUS, this->now(), 30.0);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Front is clear after filtering. Driving forward.");
        }
        else
        {
            playVoiceLine(VoiceGroup::AUTONOMOUS, this->now(), 30.0);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Driving forward. Filtered front range = %.2f m",
                front_range);
        }

        twist.linear.x = kForwardSpeed;
        twist.angular.z = 0.0;
        break;
    }

    case NavigationState::HUMAN_DETECTED:
    {
        const rclcpp::Time now = this->now();
        const double time_since_tracking =
            (now - last_human_tracking_time_).seconds();
        const double time_since_visible_human = g_last_visible_human_valid
            ? (now - g_last_visible_human_time).seconds()
            : std::numeric_limits<double>::infinity();

        // Interaction is deliberately simple: hold the base still, enable the
        // interaction topic for the configured duration, then resume DVD travel.
        if (g_interaction_session_active)
        {
            g_human_motion_phase = HumanMotionPhase::INTERACTION;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            playVoiceLine(VoiceGroup::MIMICRY_RANDOM, now, 12.0);

            const double interaction_elapsed =
                (now - g_interaction_session_start_time).seconds();

            if (interaction_elapsed >= kHumanInteractionDurationSeconds)
            {
                g_interaction_session_active = false;
                            g_human_detection_cooldown_active = true;
                g_human_detection_cooldown_start_time = now;
                human_locked_ = false;
                g_human_settle_good_samples = 0;
                g_human_centre_locked = false;
                g_human_turn_cycle_initialised = false;
                resetHumanMotionController();
                clearObstacleMarkers();
                current_state_ = NavigationState::SEARCHING;
                publishInteractionActive(now, false, true);
                publishArmsDown(now, true);

                RCLCPP_INFO(
                    this->get_logger(),
                    "HUMAN: Interaction complete. Resuming filtered-LiDAR DVD exploration.");
            }
            else
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "HUMAN_INTERACTION: Base stationary %.1f/%.1f s.",
                    interaction_elapsed,
                    kHumanInteractionDurationSeconds);
            }
            break;
        }

        const bool visual_fresh =
            human_locked_ &&
            human_tracking_valid_ &&
            time_since_tracking <= kHumanVisualFreshTimeout;

        const bool reacquire_fresh =
            human_locked_ &&
            human_tracking_grace_active_ &&
            time_since_tracking <= kHumanVisualFreshTimeout;

        if (reacquire_fresh)
        {
            playVoiceLine(VoiceGroup::GRACE_PERIOD, now, 12.0);
            g_human_motion_phase = HumanMotionPhase::RECOVERY;
            g_human_settle_good_samples = 0;
            g_human_centre_locked = false;
            g_human_turn_cycle_initialised = false;
            g_human_forward_cycle_initialised = false;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "HUMAN: Camera target in grace. Holding still for reacquisition.");
            break;
        }

        // Do not spin blindly from a stale camera offset. Stop immediately,
        // wait briefly for the camera to reacquire, then return to DVD travel.
        if (!visual_fresh)
        {
            g_human_motion_phase = HumanMotionPhase::RECOVERY;
            g_human_settle_good_samples = 0;
            g_human_centre_locked = false;
            g_human_turn_cycle_initialised = false;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            if (time_since_visible_human > kHumanLostTimeout)
            {
                playVoiceLine(VoiceGroup::PERSON_LOST, now, 1.0, true);
                human_locked_ = false;
                human_tracking_valid_ = false;
                human_tracking_grace_active_ = false;
                            resetHumanMotionController();
                clearObstacleMarkers();
                current_state_ = NavigationState::SEARCHING;

                RCLCPP_WARN(
                    this->get_logger(),
                    "HUMAN: Camera target lost for %.1f s. Resuming DVD exploration.",
                    kHumanLostTimeout);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "HUMAN: Camera target temporarily lost. Holding still for reacquisition.");
            }
            break;
        }

        const double absolute_offset =
            std::fabs(human_centre_offset_);

        // Centre first. Once centred, tolerate a small amount of drift while
        // driving. If the target leaves the release zone, stop and re-centre.
        if (absolute_offset <= kHumanCentreDeadZone)
        {
            g_human_centre_locked = true;
        }
        else if (absolute_offset >=
            kHumanCentreReleaseZone)
        {
            g_human_centre_locked = false;
        }

        double desired_angular = 0.0;

        if (!g_human_centre_locked)
        {
            playVoiceLine(VoiceGroup::CENTERING, now, 12.0);
            const double pulse_period =
                kHumanTurnPulseOnSeconds +
                kHumanTurnPulseOffSeconds;

            if (!g_human_turn_cycle_initialised)
            {
                g_human_turn_cycle_start = now;
                g_human_turn_cycle_initialised = true;
            }

            double pulse_elapsed =
                (now - g_human_turn_cycle_start).seconds();

            if (pulse_elapsed >= pulse_period)
            {
                g_human_turn_cycle_start = now;
                pulse_elapsed = 0.0;
            }

            if (pulse_elapsed <
                kHumanTurnPulseOnSeconds)
            {
                desired_angular =
                    std::copysign(
                        kHumanApproachMaxTurnSpeed,
                        -human_centre_offset_);
            }
        }
        else
        {
            g_human_turn_cycle_initialised = false;
        }

        double desired_linear = 0.0;

        // The physical base does not move reliably below 0.12 m/s, so every
        // nonzero forward command is exactly 0.12 m/s. Between
        // (kHumanLidarStopDistance + kHumanLidarSlowdownMargin) and
        // kHumanLidarStopDistance, short 0.12 m/s pulses are used instead
        // of requesting unusable lower velocities.
        if (!g_human_centre_locked)
        {
            g_human_forward_cycle_initialised = false;
        }
        else
        {
            // Use the minimum of the temporal median and the minimum of recent
            // buffered scans. The median can report a large distance while the
            // person is already close (intermittent through-body reflections
            // push the median above the true distance). The min-recent catches
            // any genuine close return and triggers the slowdown band earlier.
            const double lidar_range_median = getHumanLidarRange(human_centre_offset_);
            const double lidar_range_min_recent = getHumanLidarMinRecentRange(human_centre_offset_);
            const double lidar_range = std::min(lidar_range_median, lidar_range_min_recent);
            const bool lidar_range_valid =
                std::isfinite(lidar_range) && lidar_range > kMinValidRange;

            if (lidar_range_valid)
            {
                if (lidar_range <= kHumanLidarStopDistance)
                {
                    g_human_forward_cycle_initialised = false;
                    desired_linear = 0.0;
                }
                else if (lidar_range <=
                    kHumanLidarStopDistance + kHumanLidarSlowdownMargin)
                {
                    // Pulsed slowdown band.
                    const double pulse_period =
                        kHumanForwardPulseOnSeconds +
                        kHumanForwardPulseOffSeconds;

                    if (!g_human_forward_cycle_initialised)
                    {
                        g_human_forward_cycle_start = now;
                        g_human_forward_cycle_initialised = true;
                    }

                    double pulse_elapsed =
                        (now - g_human_forward_cycle_start).seconds();

                    if (pulse_elapsed >= pulse_period)
                    {
                        g_human_forward_cycle_start = now;
                        pulse_elapsed = 0.0;
                    }

                    desired_linear =
                        pulse_elapsed < kHumanForwardPulseOnSeconds
                            ? kHumanApproachSpeed
                            : 0.0;
                }
                else
                {
                    g_human_forward_cycle_initialised = false;
                    desired_linear = kHumanApproachSpeed;
                }
            }
            else
            {
                // No valid LiDAR return in the human cone — hold position.
                g_human_forward_cycle_initialised = false;
                desired_linear = 0.0;

                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    1000,
                    "HUMAN: No LiDAR return in human cone. Holding position.");
            }
        }

        // Interaction distance is now LiDAR-driven. Use the same min(median,
        // min_recent) logic so the AT-STOP check fires as soon as any recent
        // scan placed the human inside the threshold, not just the median.
        const double lidar_range_interact_median = getHumanLidarRange(human_centre_offset_);
        const double lidar_range_interact_min = getHumanLidarMinRecentRange(human_centre_offset_);
        const double lidar_range_interact = std::min(lidar_range_interact_median, lidar_range_interact_min);
        const bool at_interaction_distance =
            std::isfinite(lidar_range_interact) &&
            lidar_range_interact > kMinValidRange &&
            lidar_range_interact <= kHumanLidarStopDistance;
        const bool interaction_alignment_good =
            g_human_centre_locked &&
            absolute_offset <=
                kHumanInteractionMaxOffset;
        const bool base_nearly_stopped =
            std::fabs(current_twist_.linear.x) <=
                kHumanInteractionMaxLinearSpeed;

        // A close ultrasonic reading can stop the base even when the camera
        // alignment does not agree. Do not remain trapped in that state.
        if (at_interaction_distance)
        {
            if (!g_human_stop_wait_active)
            {
                g_human_stop_wait_active = true;
                g_human_stop_wait_start = now;
            }

            const double stop_wait_elapsed =
                (now - g_human_stop_wait_start).seconds();

            if (stop_wait_elapsed >=
                kHumanFailedStopTimeoutSeconds)
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                current_twist_ = twist;

                g_human_detection_cooldown_active = true;
                g_human_detection_cooldown_start_time = now;
                human_locked_ = false;
                            resetHumanMotionController();
                clearObstacleMarkers();
                current_state_ = NavigationState::SEARCHING;
                publishInteractionActive(now, false, true);
                publishArmsDown(now, true);

                RCLCPP_WARN(
                    this->get_logger(),
                    "HUMAN: Ultrasonic kept the base stopped for %.1f s "
                    "without a valid centred interaction. Abandoning attempt "
                    "and returning to DVD exploration.",
                    kHumanFailedStopTimeoutSeconds);
                break;
            }
        }
        else
        {
            g_human_stop_wait_active = false;
        }

        if (at_interaction_distance &&
            interaction_alignment_good)
        {
            g_human_forward_cycle_initialised = false;
            desired_linear = 0.0;
            desired_angular = 0.0;
            g_human_motion_phase =
                HumanMotionPhase::SETTLE;

            if (base_nearly_stopped)
            {
                g_human_settle_good_samples++;
            }
        }
        else
        {
            g_human_settle_good_samples = 0;
            g_human_motion_phase =
                HumanMotionPhase::APPROACH;
        }

        if (g_human_settle_good_samples >=
            kHumanInteractionSettleSamples)
        {
            playVoiceLine(VoiceGroup::MIMICRY_RANDOM, now, 12.0);
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            current_twist_ = twist;
            g_human_motion_phase =
                HumanMotionPhase::INTERACTION;
            g_interaction_session_active = true;
            g_interaction_session_start_time = now;
            g_human_stop_wait_active = false;
            g_human_settle_good_samples = 0;
            publishInteractionActive(now, true, true);

            RCLCPP_INFO(
                this->get_logger(),
                "HUMAN: Centred and stopped at %.2f m (LiDAR). Beginning interaction.",
                lidar_range_interact);
            break;
        }

        // When stopping, ramp toward zero at kHumanApproachDecelRate rather than
        // cutting to zero immediately, so the base does not lurch to a halt.
        if (desired_linear > 0.0)
        {
            twist.linear.x = kHumanApproachSpeed;
        }
        else
        {
            const double decel_step =
                kHumanApproachDecelRate * kControlPeriodSeconds;
            twist.linear.x =
                std::max(0.0, current_twist_.linear.x - decel_step);
        }
        twist.angular.z = desired_angular;

        // Per-cycle approach trace. Shows median and min-recent separately so
        // filter divergence is visible when the LiDAR intermittently misses the
        // human (median stays high; min-recent catches genuine close returns).
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "HUMAN [%s]: centred=%s offset=%.3f | "
            "LiDAR med=%.2f min=%.2f m stop@%.2f -> %s | "
            "settle=%d/%d  cmd v=%.3f w=%.3f",
            humanMotionPhaseName(g_human_motion_phase),
            g_human_centre_locked ? "yes" : "no",
            human_centre_offset_,
            lidar_range_interact_median,
            lidar_range_interact_min,
            kHumanLidarStopDistance,
            at_interaction_distance ? "AT-STOP" :
                ((std::isfinite(lidar_range_interact) && lidar_range_interact > kMinValidRange)
                    ? "approach" : "blind"),
            g_human_settle_good_samples,
            kHumanInteractionSettleSamples,
            twist.linear.x,
            twist.angular.z);

        break;
    }

    default:
    {
        RCLCPP_WARN(this->get_logger(), "Unknown navigation state. Returning to SEARCHING.");
        stopRobot(twist);
        clearObstacleMarkers();
        current_state_ = NavigationState::SEARCHING;
        break;
    }
    }

    current_twist_ = twist;
    cmd_vel_publisher_->publish(twist);
}

void MechelangeloBehaviour::stopRobot(geometry_msgs::msg::Twist &twist)
{
    const double linear_step = kStopLinearDecel * kControlPeriodSeconds;
    const double angular_step = kStopAngularDecel * kControlPeriodSeconds;

    auto rampTowardZero = [](double value, double max_step)
    {
        if (std::fabs(value) <= max_step)
        {
            return 0.0;
        }

        return value - std::copysign(max_step, value);
    };

    twist.linear.x = rampTowardZero(current_twist_.linear.x, linear_step);
    twist.linear.y = rampTowardZero(current_twist_.linear.y, linear_step);
    twist.linear.z = rampTowardZero(current_twist_.linear.z, linear_step);

    twist.angular.x = rampTowardZero(current_twist_.angular.x, angular_step);
    twist.angular.y = rampTowardZero(current_twist_.angular.y, angular_step);
    twist.angular.z = rampTowardZero(current_twist_.angular.z, angular_step);
}

void MechelangeloBehaviour::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const sensor_msgs::msg::LaserScan filtered_scan = filterLaserScan(*msg);

    latest_scan_ = filtered_scan;
    latest_segments_ = buildLaserSegments(filtered_scan);

    filtered_scan_publisher_->publish(filtered_scan);
}

void MechelangeloBehaviour::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    latest_imu_ = *msg;
    imu_available_ = true;
}

void MechelangeloBehaviour::humanDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (!msg->data)
    {
        return;
    }

    if (humanDetectionCooldownActive(this->now()))
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Human detection ignored during cooldown. Remaining %.1f s.",
            humanDetectionCooldownRemaining(this->now()));
        return;
    }

    if (g_human_detection_cooldown_active)
    {
        g_human_detection_cooldown_active = false;
    }

    if (current_state_ != NavigationState::HUMAN_DETECTED)
    {
        playVoiceLine(VoiceGroup::PERSON_DETECTED, this->now(), 1.0, true);
        resetHumanMotionController();
        g_human_centre_locked = false;
        g_human_turn_cycle_initialised = false;
        g_human_settle_good_samples = 0;

        // Break cleanly out of an autonomous bounce before camera alignment.
        geometry_msgs::msg::Twist stop;
        current_twist_ = stop;
        cmd_vel_publisher_->publish(stop);

        RCLCPP_WARN(
            this->get_logger(),
            "Human detected. Stopping DVD motion and waiting for camera centring.");
    }

    current_state_ = NavigationState::HUMAN_DETECTED;
}

void MechelangeloBehaviour::humanTrackingCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Received invalid /human_tracking message. "
            "Expected [detected, centre_offset, distance_m].");
        return;
    }

    const rclcpp::Time now = this->now();
    const bool detected = msg->data[0] > 0.5F;
    const bool tracking_valid =
        msg->data.size() >= 4
            ? msg->data[3] > 0.5F
            : detected;
    const bool grace_active =
        msg->data.size() >= 5
            ? msg->data[4] > 0.5F
            : (detected && !tracking_valid);

    if (detected && humanDetectionCooldownActive(now))
    {
        human_locked_ = false;
        human_tracking_valid_ = false;
        human_tracking_grace_active_ = false;

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Camera human detection ignored during cooldown. Remaining %.1f s.",
            humanDetectionCooldownRemaining(now));
        return;
    }

    if (g_human_detection_cooldown_active &&
        !humanDetectionCooldownActive(now))
    {
        g_human_detection_cooldown_active = false;
    }

    human_locked_ = detected;
    human_tracking_valid_ = detected && tracking_valid;
    human_tracking_grace_active_ = detected && grace_active;
    human_centre_offset_ =
        human_tracking_valid_
            ? static_cast<double>(msg->data[1])
            : 0.0;

    // Preserve the third field for message compatibility only. Ultrasonic
    // Range messages are the sole human-distance source on the physical robot.
    human_distance_m_ =
        static_cast<double>(msg->data[2]);
    last_human_tracking_time_ = now;

    if (!detected)
    {
        human_tracking_valid_ = false;
        human_tracking_grace_active_ = false;
        return;
    }

    if (current_state_ != NavigationState::HUMAN_DETECTED)
    {
        playVoiceLine(VoiceGroup::PERSON_DETECTED, now, 1.0, true);
        resetHumanMotionController();
        g_human_centre_locked = false;
        g_human_turn_cycle_initialised = false;
        g_human_settle_good_samples = 0;

        // Cancel any in-progress DVD turn immediately. The next control cycle
        // begins the centre-first visual alignment from a stationary base.
        geometry_msgs::msg::Twist stop;
        current_twist_ = stop;
        cmd_vel_publisher_->publish(stop);

        const double lidar_at_detection = getHumanLidarRange(human_centre_offset_);
        const bool lidar_at_detection_valid =
            std::isfinite(lidar_at_detection) && lidar_at_detection > kMinValidRange;

        RCLCPP_INFO(
            this->get_logger(),
            "HUMAN SPOTTED: camera offset=%.3f  LiDAR=%.2f m (%s)  "
            "stop threshold=%.2f m  Approaching.",
            human_centre_offset_,
            lidar_at_detection,
            lidar_at_detection_valid ? "valid" : "no-return",
            kHumanLidarStopDistance);
    }

    g_last_visible_human_valid = true;
    g_last_visible_human_centre_offset =
        human_centre_offset_;
    g_last_visible_human_distance_m =
        human_distance_m_;
    g_last_visible_human_time = now;
    current_state_ = NavigationState::HUMAN_DETECTED;
}

sensor_msgs::msg::LaserScan MechelangeloBehaviour::filterLaserScan(
    const sensor_msgs::msg::LaserScan &raw_scan)
{
    sensor_msgs::msg::LaserScan neighbour_filtered = raw_scan;
    sensor_msgs::msg::LaserScan final_filtered = raw_scan;

    if (raw_scan.ranges.empty() || raw_scan.angle_increment == 0.0)
    {
        return final_filtered;
    }

    const int scan_count = static_cast<int>(raw_scan.ranges.size());
    std::vector<double> x_points(scan_count, std::numeric_limits<double>::quiet_NaN());
    std::vector<double> y_points(scan_count, std::numeric_limits<double>::quiet_NaN());
    std::vector<bool> usable(scan_count, false);

    // Convert valid polar points to local Cartesian points.
    for (int i = 0; i < scan_count; ++i)
    {
        const double range = raw_scan.ranges[i];

        if (!isRangeUsableForFiltering(raw_scan, range))
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            final_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            continue;
        }

        const double angle = raw_scan.angle_min + static_cast<double>(i) * raw_scan.angle_increment;
        x_points[i] = range * std::cos(angle);
        y_points[i] = range * std::sin(angle);
        usable[i] = true;
    }

    // Stage 1: suppress isolated points that do not have nearby neighbours.
    for (int i = 0; i < scan_count; ++i)
    {
        if (!usable[i])
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
            continue;
        }

        int close_neighbour_count = 0;
        const int start_index = std::max(0, i - kNoiseNeighbourWindow);
        const int end_index = std::min(scan_count - 1, i + kNoiseNeighbourWindow);

        for (int j = start_index; j <= end_index; ++j)
        {
            if (j == i || !usable[j])
            {
                continue;
            }

            const double distance = std::hypot(x_points[i] - x_points[j], y_points[i] - y_points[j]);

            if (distance <= kNoiseNeighbourDistance)
            {
                close_neighbour_count++;
            }
        }

        if (close_neighbour_count < kNoiseMinNeighbourCount)
        {
            neighbour_filtered.ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    // Stage 2: build LaserProcessing-style segments and only keep points
    // that belong to a real segment.
    const std::vector<LaserSegment> accepted_segments = buildLaserSegments(neighbour_filtered);

    std::fill(final_filtered.ranges.begin(), final_filtered.ranges.end(), std::numeric_limits<float>::infinity());

    for (const LaserSegment &segment : accepted_segments)
    {
        for (int i = segment.start_index; i <= segment.end_index; ++i)
        {
            if (i >= 0 && i < scan_count && std::isfinite(neighbour_filtered.ranges[i]))
            {
                final_filtered.ranges[i] = neighbour_filtered.ranges[i];
            }
        }
    }

    // Stage 3: temporal majority vote.
    // Push the spatially-filtered ranges into the ring buffer, then only
    // keep a beam if it was finite in at least kTemporalVoteThreshold of the
    // last kTemporalBufferSize scans. The emitted range is the median of the
    // valid readings, which is more stable than any single frame's value.
    scan_history_.push_back(final_filtered.ranges);
    if (static_cast<int>(scan_history_.size()) > kTemporalBufferSize)
    {
        scan_history_.pop_front();
    }

    if (static_cast<int>(scan_history_.size()) < kTemporalBufferSize)
    {
        // Not enough history yet — return spatial-only result so the robot
        // is not blind for the first few scans after startup.
        return final_filtered;
    }

    sensor_msgs::msg::LaserScan temporal_filtered = final_filtered;
    std::fill(temporal_filtered.ranges.begin(),
              temporal_filtered.ranges.end(),
              std::numeric_limits<float>::infinity());

    for (int i = 0; i < scan_count; ++i)
    {
        std::vector<float> valid_readings;
        valid_readings.reserve(kTemporalBufferSize);

        for (const auto &hist : scan_history_)
        {
            if (i < static_cast<int>(hist.size()) && std::isfinite(hist[i]))
            {
                valid_readings.push_back(hist[i]);
            }
        }

        if (static_cast<int>(valid_readings.size()) >= kTemporalVoteThreshold)
        {
            const std::size_t mid = valid_readings.size() / 2;
            std::nth_element(valid_readings.begin(),
                             valid_readings.begin() + static_cast<std::ptrdiff_t>(mid),
                             valid_readings.end());
            temporal_filtered.ranges[i] = valid_readings[mid];
        }
    }

    return temporal_filtered;
}

std::vector<LaserSegment> MechelangeloBehaviour::buildLaserSegments(
    const sensor_msgs::msg::LaserScan &scan) const
{
    std::vector<LaserSegment> segments;

    if (scan.ranges.empty() || scan.angle_increment == 0.0)
    {
        return segments;
    }

    const int scan_count = static_cast<int>(scan.ranges.size());

    bool segment_active = false;
    LaserSegment current_segment;
    geometry_msgs::msg::Point previous_point;

    auto finish_segment = [&]()
    {
        if (!segment_active)
        {
            return;
        }

        current_segment.midpoint.x = 0.5 * (current_segment.start_point.x + current_segment.end_point.x);
        current_segment.midpoint.y = 0.5 * (current_segment.start_point.y + current_segment.end_point.y);
        current_segment.midpoint.z = 0.0;
        current_segment.length = std::hypot(
            current_segment.end_point.x - current_segment.start_point.x,
            current_segment.end_point.y - current_segment.start_point.y);
        current_segment.midpoint_angle = std::atan2(
            current_segment.midpoint.y,
            current_segment.midpoint.x);

        if (current_segment.point_count >= kSegmentMinPoints &&
            current_segment.length >= kSegmentMinLength)
        {
            segments.push_back(current_segment);
        }

        segment_active = false;
        current_segment = LaserSegment();
    };

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = scan.ranges[i];

        if (!isRangeUsableForFiltering(scan, range))
        {
            finish_segment();
            continue;
        }

        const geometry_msgs::msg::Point point = polarToPoint(scan, i);

        if (!segment_active)
        {
            segment_active = true;
            current_segment = LaserSegment();
            current_segment.start_index = i;
            current_segment.end_index = i;
            current_segment.point_count = 1;
            current_segment.start_point = point;
            current_segment.end_point = point;
            current_segment.min_range = range;
            previous_point = point;
            continue;
        }

        const double gap = std::hypot(point.x - previous_point.x, point.y - previous_point.y);

        if (gap <= kSegmentJoinDistance)
        {
            current_segment.end_index = i;
            current_segment.end_point = point;
            current_segment.point_count++;
            current_segment.min_range = std::min(current_segment.min_range, range);
            previous_point = point;
        }
        else
        {
            finish_segment();

            segment_active = true;
            current_segment = LaserSegment();
            current_segment.start_index = i;
            current_segment.end_index = i;
            current_segment.point_count = 1;
            current_segment.start_point = point;
            current_segment.end_point = point;
            current_segment.min_range = range;
            previous_point = point;
        }
    }

    finish_segment();
    return segments;
}

geometry_msgs::msg::Point MechelangeloBehaviour::polarToPoint(
    const sensor_msgs::msg::LaserScan &scan,
    int index) const
{
    geometry_msgs::msg::Point point;

    if (index < 0 || index >= static_cast<int>(scan.ranges.size()))
    {
        return point;
    }

    const double angle = scan.angle_min + static_cast<double>(index) * scan.angle_increment;
    const double range = scan.ranges[index];

    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.z = 0.0;

    return point;
}

bool MechelangeloBehaviour::isRangeUsableForFiltering(
    const sensor_msgs::msg::LaserScan &scan,
    double range) const
{
    if (!std::isfinite(range))
    {
        return false;
    }

    if (range <= kMinValidRange)
    {
        return false;
    }

    if (scan.range_min > 0.0 && range < scan.range_min)
    {
        return false;
    }

    if (scan.range_max > 0.0 && range > scan.range_max)
    {
        return false;
    }

    return true;
}

bool MechelangeloBehaviour::isRangeValid(double range) const
{
    return std::isfinite(range) && range > kMinValidRange;
}

double MechelangeloBehaviour::getMinimumRange(double start_angle, double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    double min_range = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

        if (!angleInsideWindow(angle, start_angle, end_angle))
        {
            continue;
        }

        const double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range < min_range)
        {
            min_range = range;
        }
    }

    return min_range;
}

double MechelangeloBehaviour::getFrontRange() const
{
    return getMinimumRange(-kFrontCheckAngle, kFrontCheckAngle);
}

bool MechelangeloBehaviour::getLongestRange(double &out_angle, double &out_range) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return false;
    }

    const int scan_count = static_cast<int>(latest_scan_.ranges.size());
    const double angle_step = std::fabs(latest_scan_.angle_increment);

    if (scan_count <= 0 || angle_step <= 0.0)
    {
        return false;
    }

    // ------------------------------------------------------
    // Step 1: classify beams as open or blocked.
    //
    // Open means either:
    //   - infinity: the LiDAR did not hit anything within range, or
    //   - a finite return farther than the clearance distance.
    // ------------------------------------------------------
    std::vector<bool> open_beam(scan_count, false);

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];

        if (std::isinf(range))
        {
            open_beam[i] = true;
        }
        else if (std::isfinite(range) && range >= kDvdOpenClearanceDistance)
        {
            open_beam[i] = true;
        }
    }

    auto angleForIndex = [&](int index)
    {
        return normaliseAngle(
            latest_scan_.angle_min + static_cast<double>(index) * latest_scan_.angle_increment);
    };

    auto absoluteAngleForIndex = [&](int index)
    {
        return std::fabs(angleForIndex(index));
    };

    auto rangeForLog = [&](int index)
    {
        const double range = latest_scan_.ranges[index];

        if (std::isinf(range))
        {
            return std::numeric_limits<double>::infinity();
        }

        if (std::isfinite(range))
        {
            return range;
        }

        return 0.0;
    };

    auto isPreferredBounceAngle = [&](int index)
    {
        const double abs_angle = absoluteAngleForIndex(index);
        return abs_angle >= kDvdPreferredMinTurnAngle &&
               abs_angle <= kDvdPreferredMaxTurnAngle;
    };

    auto isFallbackBounceAngle = [&](int index)
    {
        const double abs_angle = absoluteAngleForIndex(index);
        return abs_angle >= kDvdAvoidFrontAngle &&
               abs_angle <= kDvdAvoidReverseAngle;
    };

    // ------------------------------------------------------
    // Step 2: extract open sectors and collect safe candidate beams.
    //
    // Preferred candidates are side-bounce angles, roughly +/-55 to +/-150 deg.
    // General fallback candidates simply avoid the front wall and exact reverse.
    // Left and right candidates are stored separately so one long side of the
    // room does not dominate every decision.
    // ------------------------------------------------------
    std::vector<int> preferred_left_candidates;
    std::vector<int> preferred_right_candidates;
    std::vector<int> fallback_left_candidates;
    std::vector<int> fallback_right_candidates;
    std::vector<int> fallback_all_candidates;

    int accepted_sector_count = 0;
    int widest_sector_count = 0;

    auto collectCandidatesFromSector = [&](int start_index, int count)
    {
        if (count <= 0)
        {
            return;
        }

        const double sector_width = static_cast<double>(count) * angle_step;
        widest_sector_count = std::max(widest_sector_count, count);

        if (sector_width < kDvdMinSectorWidth)
        {
            return;
        }

        accepted_sector_count++;

        int edge_margin_count = static_cast<int>(std::ceil(kDvdSectorEdgeMargin / angle_step));

        // Never let the edge margin remove the entire sector.
        edge_margin_count = std::min(edge_margin_count, std::max(0, (count - 1) / 2));

        for (int offset = edge_margin_count; offset < count - edge_margin_count; ++offset)
        {
            const int index = (start_index + offset) % scan_count;
            const double angle = angleForIndex(index);

            if (isPreferredBounceAngle(index))
            {
                if (angle >= 0.0)
                {
                    preferred_left_candidates.push_back(index);
                }
                else
                {
                    preferred_right_candidates.push_back(index);
                }
            }

            if (isFallbackBounceAngle(index))
            {
                fallback_all_candidates.push_back(index);

                if (angle >= 0.0)
                {
                    fallback_left_candidates.push_back(index);
                }
                else
                {
                    fallback_right_candidates.push_back(index);
                }
            }
        }
    };

    const bool all_open = std::all_of(open_beam.begin(), open_beam.end(),
        [](bool value) { return value; });

    if (all_open)
    {
        collectCandidatesFromSector(0, scan_count);
    }
    else
    {
        for (int start_index = 0; start_index < scan_count; ++start_index)
        {
            if (!open_beam[start_index])
            {
                continue;
            }

            const int previous_index = (start_index - 1 + scan_count) % scan_count;
            if (open_beam[previous_index])
            {
                continue;
            }

            int count = 0;
            while (count < scan_count && open_beam[(start_index + count) % scan_count])
            {
                count++;
            }

            collectCandidatesFromSector(start_index, count);
        }
    }

    // ------------------------------------------------------
    // Step 3: choose a random safe bounce angle.
    //
    // Preferred behaviour:
    //   - use the side-bounce band if possible;
    //   - choose left/right with a 50/50 coin flip when both are available;
    //   - choose a random beam inside that side's safe candidate list.
    // This produces a DVD-screensaver style movement instead of always going
    // down the longest wall/side of the room.
    // ------------------------------------------------------
    static thread_local std::mt19937 rng(std::random_device{}());

    auto chooseFromCandidates = [&](const std::vector<int> &candidates)
    {
        std::uniform_int_distribution<int> index_dist(
            0, static_cast<int>(candidates.size()) - 1);
        return candidates[index_dist(rng)];
    };

    auto chooseBalancedSide = [&](const std::vector<int> &left_candidates,
                                  const std::vector<int> &right_candidates,
                                  bool &used_left_side,
                                  bool &used_right_side)
    {
        used_left_side = false;
        used_right_side = false;

        if (!left_candidates.empty() && !right_candidates.empty())
        {
            std::uniform_int_distribution<int> side_dist(0, 1);

            if (side_dist(rng) == 0)
            {
                used_left_side = true;
                return chooseFromCandidates(left_candidates);
            }

            used_right_side = true;
            return chooseFromCandidates(right_candidates);
        }

        if (!left_candidates.empty())
        {
            used_left_side = true;
            return chooseFromCandidates(left_candidates);
        }

        used_right_side = true;
        return chooseFromCandidates(right_candidates);
    };

    int selected_index = -1;
    bool used_left_side = false;
    bool used_right_side = false;

    if (!preferred_left_candidates.empty() || !preferred_right_candidates.empty())
    {
        selected_index = chooseBalancedSide(
            preferred_left_candidates,
            preferred_right_candidates,
            used_left_side,
            used_right_side);
    }
    else if (!fallback_left_candidates.empty() || !fallback_right_candidates.empty())
    {
        selected_index = chooseBalancedSide(
            fallback_left_candidates,
            fallback_right_candidates,
            used_left_side,
            used_right_side);
    }
    else if (!fallback_all_candidates.empty())
    {
        selected_index = chooseFromCandidates(fallback_all_candidates);
        used_left_side = angleForIndex(selected_index) >= 0.0;
        used_right_side = !used_left_side;
    }

    if (selected_index >= 0)
    {
        out_angle = angleForIndex(selected_index);
        out_range = rangeForLog(selected_index);

        return true;
    }

    // ------------------------------------------------------
    // Step 4: fallback to the old longest finite scan if the room is too
    // cluttered for a safe random bounce heading.
    // ------------------------------------------------------
    double max_finite_range = 0.0;
    int max_finite_index = -1;

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];

        if (isRangeValid(range) && range > max_finite_range)
        {
            max_finite_range = range;
            max_finite_index = i;
        }
    }

    if (max_finite_index < 0)
    {
        return false;
    }

    const double tied_range_threshold =
        std::max(kMinValidRange, max_finite_range - kLongestRangeTieTolerance);

    std::vector<bool> near_longest(scan_count, false);

    for (int i = 0; i < scan_count; ++i)
    {
        const double range = latest_scan_.ranges[i];
        near_longest[i] = isRangeValid(range) && range >= tied_range_threshold;
    }

    int best_start_index = max_finite_index;
    int best_count = 0;

    for (int start_index = 0; start_index < scan_count; ++start_index)
    {
        if (!near_longest[start_index])
        {
            continue;
        }

        const int previous_index = (start_index - 1 + scan_count) % scan_count;
        if (near_longest[previous_index])
        {
            continue;
        }

        int count = 0;
        while (count < scan_count && near_longest[(start_index + count) % scan_count])
        {
            count++;
        }

        if (count > best_count)
        {
            best_count = count;
            best_start_index = start_index;
        }
    }

    if (best_count == 0)
    {
        best_count = 1;
        best_start_index = max_finite_index;
    }

    const double best_mid_index = std::fmod(
        static_cast<double>(best_start_index) +
            0.5 * static_cast<double>(std::max(0, best_count - 1)),
        static_cast<double>(scan_count));

    out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
    out_range = max_finite_range;

    return true;
}

double MechelangeloBehaviour::normaliseAngle(double angle_rad) const
{
    while (angle_rad > M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }

    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }

    return angle_rad;
}

bool MechelangeloBehaviour::angleInsideWindow(
    double angle_rad,
    double start_angle,
    double end_angle) const
{
    const double angle = normaliseAngle(angle_rad);
    const double start = normaliseAngle(start_angle);
    const double end = normaliseAngle(end_angle);

    if (start <= end)
    {
        return angle >= start && angle <= end;
    }

    return angle >= start || angle <= end;
}

bool MechelangeloBehaviour::segmentOverlapsAngleWindow(
    const LaserSegment &segment,
    double start_angle,
    double end_angle) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return false;
    }

    for (int i = segment.start_index; i <= segment.end_index; ++i)
    {
        if (i < 0 || i >= static_cast<int>(latest_scan_.ranges.size()))
        {
            continue;
        }

        const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;

        if (angleInsideWindow(angle, start_angle, end_angle))
        {
            return true;
        }
    }

    return false;
}

bool MechelangeloBehaviour::findBlockingObstaclesInFront(
    std::vector<LaserSegment> &blocking_segments) const
{
    blocking_segments.clear();

    for (const LaserSegment &segment : latest_segments_)
    {
        if (segment.min_range <= stop_distance_m_ &&
            segmentOverlapsAngleWindow(segment, -kFrontCheckAngle, kFrontCheckAngle))
        {
            blocking_segments.push_back(segment);
        }
    }

    return !blocking_segments.empty();
}

void MechelangeloBehaviour::publishObstacleMarkers(
    const std::vector<LaserSegment> &blocking_segments)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.ns = "behaviour_blocking_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int marker_id = 1;

    for (const LaserSegment &segment : blocking_segments)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "behaviour_blocking_obstacles";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = segment.midpoint.x;
        marker.pose.position.y = segment.midpoint.y;
        marker.pose.position.z = 0.15;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Make marker size scale slightly with the observed segment length.
        const double marker_width = std::clamp(segment.length + 0.15, 0.20, 0.80);
        marker.scale.x = marker_width;
        marker.scale.y = marker_width;
        marker.scale.z = 0.30;

        // Red/orange transparent marker for blocking obstacle.
        marker.color.a = 0.75F;
        marker.color.r = 1.0F;
        marker.color.g = 0.15F;
        marker.color.b = 0.0F;

        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 400000000; // 0.4 s

        marker_array.markers.push_back(marker);
    }

    obstacle_marker_publisher_->publish(marker_array);
}

void MechelangeloBehaviour::clearObstacleMarkers()
{
    if (!obstacle_marker_publisher_)
    {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = latest_scan_.header.frame_id.empty() ? "base_link" : latest_scan_.header.frame_id;
    clear_marker.header.stamp = this->get_clock()->now();
    clear_marker.ns = "behaviour_blocking_obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    obstacle_marker_publisher_->publish(marker_array);
}

double MechelangeloBehaviour::getHumanLidarRange(double centre_offset) const
{
    const double estimated_human_angle = -centre_offset * kCameraHorizontalFov;
    const double start_angle = estimated_human_angle - kHumanLidarWindow;
    const double end_angle = estimated_human_angle + kHumanLidarWindow;

    return getMinimumRange(start_angle, end_angle);
}

double MechelangeloBehaviour::getHumanLidarMinRecentRange(double centre_offset) const
{
    if (scan_history_.empty() ||
        latest_scan_.ranges.empty() ||
        latest_scan_.angle_increment == 0.0)
    {
        return std::numeric_limits<double>::infinity();
    }

    const double estimated_human_angle = -centre_offset * kCameraHorizontalFov;
    const double start_angle = estimated_human_angle - kHumanLidarWindow;
    const double end_angle = estimated_human_angle + kHumanLidarWindow;

    const int scan_count = static_cast<int>(latest_scan_.ranges.size());
    double min_range = std::numeric_limits<double>::infinity();

    for (const auto &hist_ranges : scan_history_)
    {
        for (int i = 0; i < scan_count && i < static_cast<int>(hist_ranges.size()); ++i)
        {
            const double angle = latest_scan_.angle_min +
                static_cast<double>(i) * latest_scan_.angle_increment;
            if (angle < start_angle || angle > end_angle)
                continue;
            const float r = hist_ranges[i];
            if (std::isfinite(r) && r > kMinValidRange)
                min_range = std::min(min_range, static_cast<double>(r));
        }
    }
    return min_range;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MechelangeloBehaviour>();

    // This physical build uses camera bearing and dual ultrasonic ranging.
    node->run(false);

    rclcpp::shutdown();
    return 0;
}
