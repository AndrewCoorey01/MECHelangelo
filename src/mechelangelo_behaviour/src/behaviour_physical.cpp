/////////////////////////////////////////////////////////////////////////
/// DVD bounce LiDAR exploration + camera/dual-ultrasonic human approach

#include "behaviour.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <functional>
#include <memory>
#include <random>
#include <sstream>
#include <string>
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
static constexpr double kAngleGain = 0.55;          // retained for compatibility
static constexpr double kAlignmentTolerance = 0.10; // radians, about 5.7 degrees
static constexpr double kTurnAngularAcceleration = 0.45; // retained for compatibility

// Smooth commanded stops so the physical base does not snap from motion to zero.
static constexpr double kStopLinearDecel = 0.16;  // m/s^2
static constexpr double kStopAngularDecel = 0.45; // rad/s^2

// Stop this far before a real obstacle/wall.
// Loaded from the ROS parameter 'stop_distance_m'.
// Default: 1.5 m (simulation). Physical robot: set to 0.75 in the launch file.

// 5 loops x 0.1 s = 0.5 seconds. The reactive turn loop below keeps the robot
// moving out of corners instead of sitting through long stop/search cycles.
static constexpr int kStopDurationLoops = 5;

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
static constexpr double kHumanTargetDistance = 1.75;     // m
static constexpr double kHumanDistanceTolerance = 0.15; // m
static constexpr double kHumanMaxForwardSpeed = 0.16;   // m/s, slower approach helps keep person in camera
static constexpr double kHumanMaxReverseSpeed = 0.12;   // m/s
static constexpr double kHumanMaxTurnSpeed = 0.50;      // rad/s, gentler human tracking so the camera does not swing off target
static constexpr double kHumanTurnGain = 1.8;           // image offset to angular speed
static constexpr double kHumanForwardGain = 0.35;       // distance error to linear speed
static constexpr double kHumanCentreDeadZone = 0.06;    // normalised image width
static constexpr double kHumanLostTimeout = 1.5;        // seconds, wait stationary before returning to DVD exploration
static constexpr double kHumanRecoveryTurnSpeed = 0.375; // retained for compatibility
static constexpr double kHumanRecoveryCreepSpeed = 0.00; // never move blindly after camera loss


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
static constexpr double kLidarCameraMaxDisagreement = 0.4;
// Stop distance raised from 1.65m to 1.80m to account for:
//   - Decel overshoot (~30mm at 0.20 m/s², ~66mm at the old 0.10 m/s²)
//   - Temporal LiDAR filter lag (~10–20mm at 0.12 m/s closing speed)
//   - Physical motor/wheel inertia (~10–20mm additional coast after command)
// Combined worst-case pushes the actual stop point ~50–60mm past the commanded
// threshold, so 1.80m commanded gives ~1.72–1.75m physical stopping distance —
// comfortably above the 1.50m ultrasonic emergency threshold.
static constexpr double kHumanLidarStopDistance = 1.80;
static constexpr double kHumanLidarStopTolerance = 0.20;
// LiDAR is the primary human-approach stopping sensor. kHumanLidarSlowdownMargin
// is added to kHumanLidarStopDistance to define the pulsed-slowdown band start.
static constexpr double kHumanLidarSlowdownMargin = 0.25;  // m above stop to begin slowdown pulses
// Deceleration rate when the stop condition is reached. Increased from 0.10 to
// 0.20 m/s² to halve the commanded overshoot (0.030m vs 0.066m at 0.12 m/s).
static constexpr double kHumanApproachDecelRate = 0.20;    // m/s²

// Once the robot reaches a usable interaction pose, hold it instead of
// continuing to creep forward or dropping straight back into DVD exploration
// when the detector flickers for a moment.
static constexpr double kHumanInteractionHoldTimeout = 5.0;       // seconds to hold pose through brief detector loss
static constexpr double kHumanInteractionHoldMaxOffset = 0.18;    // human must be reasonably centred to latch interaction hold
static constexpr double kHumanRecoveryRotateOffsetThreshold = 0.15; // do not rotate during recovery if human was basically centred
static constexpr double kHumanInteractionHoldRangeSlack = 0.25;   // extra distance slack while staying latched

// Human interaction session timing. Once a valid interaction pose is reached,
// hold interaction for this long, then return to DVD exploration and ignore
// camera-triggered human interrupts for a short cooldown period.
static constexpr double kHumanInteractionDurationSeconds = 30.0;
static constexpr double kHumanDetectionCooldownSeconds = 10.0;

// Safety zone.
static constexpr double kSafetyZoneRadius = 1.5;
static constexpr double kSafetyZoneIntruderThreshold = 0.3;
static constexpr double kSafetyZoneHumanExclusionAngle = 25.0 * M_PI / 180.0;

// During HUMAN_DETECTED, the robot is allowed to have a closer wall behind it
// because the arms and human interaction are mainly front/side constrained.
// This avoids false blocking when the base turns imperfectly and the rear of
// the LiDAR/base drifts slightly toward the wall it just stopped near.
static constexpr double kHumanRearSafetyRadius = 1.0; // m, rear-only during human approach/interaction
static constexpr double kHumanRearSectorHalfAngle = 45.0 * M_PI / 180.0; // rear cone around +/-180 deg

// In human reposition mode, the 1.5 m bubble is a requirement for arm interaction,
// not a hard stop for base motion. These smaller radii are the true emergency
// stop distances while the robot is trying to move out of a bad interaction pose.
static constexpr double kHumanModeFrontSideHardStopRadius = 0.85; // m
static constexpr double kHumanModeRearHardStopRadius = 1.0;       // m, keep rear at 1 m as requested

// ------------------------------------------------------
// Human interaction repositioning tuning
// ------------------------------------------------------
// The robot needs a clear space around itself before arm interaction.
// If the human is visible but the 1.5 m arm bubble is blocked by a wall or
// obstacle, the robot does not drive straight at the human. Instead it samples
// short forward/arc moves and chooses the one predicted to clear the bubble
// while still keeping the human in view.
static constexpr double kInteractionBubbleRadius = 1.5;                 // m, arm movement clearance around robot
static constexpr double kInteractionBubbleSafetyMargin = 0.10;          // m, extra buffer added to the bubble check
static constexpr double kInteractionHumanExclusionAngle = 25.0 * M_PI / 180.0; // ignore tracked human cone
static constexpr int kInteractionAllowedBlockedBeams = 3;               // tolerate a few filtered/noisy beams
static constexpr double kInteractionRepositionLookahead = 0.65;         // m, predicted short move distance
static constexpr double kInteractionRepositionSpeed = 0.11;             // m/s, enough movement to escape a bad pose
static constexpr double kInteractionRepositionTurnGain = 0.85;          // rad/s per rad target heading
static constexpr double kInteractionRepositionMaxAngularSpeed = 0.45;   // rad/s, still below normal exploration turning
static constexpr double kInteractionPathHalfWidth = 0.45;               // m, collision corridor half-width
static constexpr double kInteractionPathForwardBuffer = 0.25;           // m, extra forward collision buffer
static constexpr double kInteractionMaxCandidateAngle = 60.0 * M_PI / 180.0;
static constexpr double kInteractionRecenterFirstOffset = 0.38;         // if human is near camera edge, re-centre before repositioning
static constexpr double kInteractionViewLossOffset = 0.48;              // heavy penalty if predicted offset approaches camera edge
static constexpr double kInteractionParallelBiasAngle = 35.0 * M_PI / 180.0; // prefer shallow wall-parallel arcs


// Reposition smoothing and interaction-state hysteresis.
// The robot commits to one short arc instead of selecting a new direction every
// 100 ms. It then stops briefly and verifies several scans before interaction.
static constexpr double kInteractionRepositionCommitSeconds = 0.80;
static constexpr double kInteractionSettleDurationSeconds = 0.80;
static constexpr int kInteractionSettleRequiredGoodSamples = 5;
static constexpr int kInteractionRepositionEnterBlockedBeams = 12;
static constexpr int kInteractionRepositionExitBlockedBeams = 5;
static constexpr double kInteractionRepositionEnterMinMargin = -0.20; // m inside required bubble
static constexpr double kInteractionRepositionExitMinMargin = -0.08;  // tolerate small scan/geometry error
static constexpr int kInteractionMinBlockedImprovementBeams = 4;
static constexpr double kInteractionMinBlockedImprovementFraction = 0.10;
static constexpr double kInteractionMinClearanceImprovement = 0.08; // m
static constexpr double kInteractionCommandSmoothingAlpha = 0.35;


// ------------------------------------------------------
// Fused human tracking + safe local arc planner
// ------------------------------------------------------
// Camera provides only the person's bearing and visibility. Human distance is
// always taken from LiDAR: preferably a coherent associated segment, otherwise
// the nearest valid LiDAR return in a narrow cone around the camera bearing.
// Camera-reported depth is intentionally ignored because the real camera has no
// depth capability.
static constexpr double kHumanAssociationWindow = 16.0 * M_PI / 180.0;
static constexpr int kHumanAssociationMinPoints = 3;
static constexpr double kHumanAssociationRangeScale = 1.50;
static constexpr double kHumanFusionBearingAlpha = 0.45;
static constexpr double kHumanFusionRangeAlpha = 0.35;
static constexpr double kHumanFusionSafetyRangeAlphaDown = 0.70;
static constexpr double kHumanFusionSafetyRangeAlphaUp = 0.25;
static constexpr double kHumanFusionMaxRangeStep = 0.35;
static constexpr double kHumanLidarFreshTimeout = 0.60;
static constexpr double kHumanVisualFreshTimeout = 0.60;

// Persistent human target lock. A camera-confirmed LiDAR cluster is stored as a
// point in the robot frame, propagated through the robot's own motion and then
// reassociated using both Cartesian position and cluster shape. This prevents a
// nearby wall from replacing the human when the camera briefly loses them.
static constexpr double kHumanTargetCartesianGate = 0.75; // m
static constexpr double kHumanTargetRangeGate = 0.80;     // m
static constexpr double kHumanTargetBearingGate = 20.0 * M_PI / 180.0;
static constexpr double kHumanTargetVisualBearingGate = 26.0 * M_PI / 180.0;
static constexpr double kHumanTargetLidarTrackTimeout = 1.50; // s
static constexpr double kHumanTargetPredictionTimeout = 3.00; // s
static constexpr double kHumanTargetPositionAlphaConfirmed = 0.45;
static constexpr double kHumanTargetPositionAlphaLidarOnly = 0.25;
static constexpr double kHumanTargetCameraBearingAlpha = 0.25;
static constexpr double kHumanTargetSizePenaltyScale = 20.0;
static constexpr double kHumanLidarOnlyMaxForwardSpeed = 0.08;
static constexpr double kHumanLidarOnlyMaxAngularSpeed = 0.25;
static constexpr double kHumanLidarOnlyMotionTimeout = 2.50; // s

// Camera-guided, LiDAR-ranged fallback. If no coherent human segment can be
// associated, the camera still supplies bearing while the nearest valid LiDAR
// return in this narrow cone supplies the only distance estimate. Translation
// is allowed only while that LiDAR guard is fresh and safely beyond the human
// minimum distance. Camera depth is never used.
static constexpr double kHumanCameraGuidedMaxForwardSpeed = 0.10; // m/s
static constexpr double kHumanCameraGuidedMaxAngularSpeed = 0.35; // rad/s
static constexpr double kHumanCameraGuardWindow = 10.0 * M_PI / 180.0;
static constexpr double kHumanCameraGuardRangeAlpha = 0.45;

// Human distance safety. The planner aims farther than the absolute limit so
// sensor delay, base momentum and turning translation do not take the robot
// inside 1.5 m.
static constexpr double kHumanAbsoluteMinimumDistance = 1.50;
static constexpr double kHumanPlannerMinimumDistance = 1.65;
static constexpr double kHumanDesiredInteractionDistance = 1.80;
static constexpr double kHumanDesiredInteractionTolerance = 0.18;
static constexpr double kHumanSlowdownDistance = 2.15;

// Dynamic-window-style arc planner. Commands are selected for a short horizon,
// held briefly, and acceleration-limited between control cycles.
static constexpr double kHumanPlannerHorizon = 1.20;
static constexpr double kHumanPlannerSimulationStep = 0.15;
static constexpr double kHumanPlannerCommandDuration = 0.35;
static constexpr double kHumanPlannerMaxViewAngle = 30.0 * M_PI / 180.0;
static constexpr double kHumanPlannerInteractionViewAngle = 12.0 * M_PI / 180.0;
static constexpr double kHumanPlannerLinearAcceleration = 0.30;   // m/s^2
static constexpr double kHumanPlannerAngularAcceleration = 0.90;  // rad/s^2
static constexpr double kHumanPlannerMaxForwardSpeed = 0.16;
static constexpr double kHumanPlannerMaxReverseSpeed = 0.14;
static constexpr double kHumanPlannerMaxAngularSpeed = 0.45;
static constexpr double kHumanPlannerBubbleExitMargin = -0.08;
static constexpr int kHumanPlannerBubbleExitBlockedBeams = 5;
static constexpr double kHumanPlannerSettleDuration = 0.80;
static constexpr int kHumanPlannerSettleGoodSamples = 5;

// Forced wall escape. Normal repositioning is allowed to prefer a stable human
// pose, but if the interaction bubble remains badly blocked without improving,
// the controller temporarily switches objectives and commits to moving away
// from the obstruction while keeping the human safely in view.
static constexpr double kHumanEscapeStagnationDuration = 1.10;
static constexpr double kHumanEscapeCommitDuration = 1.80;
static constexpr double kHumanEscapeMaximumDuration = 7.00;
static constexpr double kHumanEscapePlanningHorizon = 4.00;
static constexpr double kHumanEscapeSimulationStep = 0.20;
static constexpr double kHumanEscapeMaxViewAngle = 35.0 * M_PI / 180.0;
static constexpr double kHumanEscapeMaximumRange = 2.80;
static constexpr int kHumanEscapeBlockedBeamThreshold = 20;
static constexpr double kHumanEscapeMarginThreshold = -0.15;
static constexpr int kHumanEscapeBlockedImprovement = 8;
static constexpr double kHumanEscapeMarginImprovement = 0.06;
static constexpr double kHumanEscapeStationaryPenalty = 28.0;


// Deterministic close-wall recovery. This is only used after the sampled arc
// planner has repeatedly found no safe command. It creates an S-shaped lateral
// shift: back away to make human-distance room, arc away from the wall, then
// counter-steer to face the human again. Emergency scan checks remain active on
// every control cycle.
static constexpr int kHumanNoSafeFallbackTriggerCycles = 5; // 0.5 s at 10 Hz
static constexpr double kHumanDeterministicBackoffTargetRange = 2.15; // m
static constexpr double kHumanDeterministicBackoffMaxSeconds = 2.50;
static constexpr double kHumanDeterministicBackoffSpeed = 0.14; // m/s
static constexpr double kHumanDeterministicClearArcSpeed = 0.14; // m/s
static constexpr double kHumanDeterministicReturnArcSpeed = 0.12; // m/s
static constexpr double kHumanDeterministicArcAngularSpeed = 0.32; // rad/s
static constexpr double kHumanDeterministicClearArcSeconds = 1.35;
static constexpr double kHumanDeterministicReturnArcSeconds = 1.35;
static constexpr double kHumanDeterministicRearClearance = 1.15; // m
static constexpr double kHumanDeterministicFrontClearance = 1.00; // m
static constexpr double kHumanDeterministicRearConeHalfAngle = 40.0 * M_PI / 180.0;
static constexpr double kHumanDeterministicFrontConeHalfAngle = 35.0 * M_PI / 180.0;
static constexpr double kHumanDeterministicMaxHumanBearing = 30.0 * M_PI / 180.0;

// Adaptive clearance waypoint / S-curve recovery.
// Instead of repeatedly applying tiny local corrections, the controller computes
// one proportional lateral shift from the current interaction-bubble deficit.
// Two opposite constant-curvature arcs then create an S-shaped path that finishes
// approximately parallel to the original human approach heading.
static constexpr int kHumanWaypointTriggerCycles = 4; // 0.4 s persistent blockage
static constexpr double kHumanWaypointActivationMaxRange = 3.20; // m from human
static constexpr double kHumanWaypointImmediateMargin = -0.28; // m, severe enough to start immediately
static constexpr double kHumanWaypointEntryMargin = -0.12; // m
static constexpr int kHumanWaypointEntryBlockedBeams = 20;
static constexpr double kHumanWaypointClearanceBuffer = 0.12; // m beyond measured deficit
static constexpr double kHumanWaypointMinimumLateralShift = 0.30; // m
static constexpr double kHumanWaypointMaximumLateralShift = 0.60; // m
static constexpr double kHumanWaypointArcSpeed = 0.16; // m/s
static constexpr double kHumanWaypointArcAngularSpeed = 0.34; // rad/s
static constexpr double kHumanWaypointMinimumArcSeconds = 1.50;
static constexpr double kHumanWaypointMaximumArcSeconds = 3.80;
static constexpr double kHumanWaypointMaximumTotalSeconds = 8.50;
static constexpr double kHumanWaypointViewSoftLimit = 24.0 * M_PI / 180.0;
static constexpr double kHumanWaypointViewHardLimit = 31.0 * M_PI / 180.0;
static constexpr double kHumanWaypointCompletionMargin = -0.05; // m
static constexpr int kHumanWaypointCompletionBlockedBeams = 8;

// If the robot encounters a human closer than the planner safety band, reverse
// until a usable interaction distance is restored. If there is not enough room
// behind the robot, publish a voice-prompt placeholder instead of moving.
static constexpr double kHumanTooCloseBackoffTrigger = 1.66; // m
static constexpr double kHumanTooCloseBackoffRelease = 1.85; // m
static constexpr double kHumanTooCloseReverseSpeed = 0.14; // m/s
static constexpr double kHumanTooCloseMaxAngularSpeed = 0.12; // rad/s
static constexpr double kHumanStepBackPromptRepeatSeconds = 5.0;

// A forced escape must create real lateral displacement. The earlier controller
// could select straight reverse motion even when the wall-repulsion vector was
// almost exactly sideways. These values force a curved trajectory toward the
// free side while still enforcing the camera-view and human-distance limits.
static constexpr double kHumanEscapeMinimumAngularSpeed = 0.18; // rad/s
static constexpr double kHumanEscapeMinimumLateralDisplacement = 0.18; // m over prediction horizon
static constexpr double kHumanEscapeStrongLateralDirection = 0.35; // abs(sin(direction))
static constexpr double kHumanEscapeWrongSidePenalty = 24.0;
static constexpr double kHumanEscapeInsufficientLateralPenalty = 24.0;

// Proactive wall avoidance during the first human approach.
//
// IMPORTANT: merely losing a small amount of clearance is not enough to enter
// repositioning. A current non-human LiDAR return must first enter the 2.0 m
// warning zone, and that return must threaten either the swept base corridor or
// the final interaction bubble. This prevents clear paths from activating a
// stale wall-avoidance direction.
static constexpr double kHumanApproachLookaheadDistance = 0.65; // m, retained for diagnostics/scoring
static constexpr double kHumanApproachObstacleWarningDistance = 2.00; // m, begin checking path risk
static constexpr double kHumanApproachObstacleReleaseDistance = 2.15; // m, hysteresis release
static constexpr double kHumanApproachCorridorHalfWidth = 0.55; // m, swept base corridor
static constexpr double kHumanApproachCorridorForwardBuffer = 0.20; // m
static constexpr int kHumanApproachRiskEnterCycles = 3; // consecutive risky scans
static constexpr int kHumanApproachRiskExitCycles = 4;  // consecutive clear scans
static constexpr double kHumanApproachAvoidanceTurnOffset = 35.0 * M_PI / 180.0;
static constexpr double kHumanApproachReturnTurnOffset = 12.0 * M_PI / 180.0;
static constexpr double kHumanApproachAvoidanceMaxViewAngle = 32.0 * M_PI / 180.0;
static constexpr double kHumanApproachPreferredViewAngle = 24.0 * M_PI / 180.0;
static constexpr double kHumanApproachAvoidanceMaxAngularSpeed = 0.30;
static constexpr double kHumanApproachClearWallMinImprovement = 0.20; // m
static constexpr double kHumanApproachClearWallThreatFraction = 0.60;
static constexpr double kHumanApproachAvoidanceZeroTurnPenalty = 10.0;
static constexpr double kHumanApproachAvoidanceWrongSidePenalty = 12.0;

// Keep both arms in the named arm_down pose whenever the robot is exploring,
// approaching, repositioning, escaping or settling. Mimicry takes control only
// after the internal interaction state has started.
static constexpr double kArmDownRepublishPeriod = 0.50;


// Last confirmed visual human observation. Kept file-scope so this patch does not
// require behaviour.hpp changes. Used to pause/reacquire instead of immediately
// dropping back to DVD exploration when the camera briefly loses the person.
static bool g_last_visible_human_valid = false;
static double g_last_visible_human_centre_offset = 0.0;
static double g_last_visible_human_distance_m = -1.0;
static rclcpp::Time g_last_visible_human_time;

// File-scope latch so this test patch does not require behaviour.hpp changes.
// This becomes active once the robot is centred, at a usable interaction range,
// and has a clear 1.5 m arm bubble.
static bool g_interaction_hold_active = false;
static rclcpp::Time g_interaction_hold_time;
static double g_interaction_hold_range_m = -1.0;
static double g_interaction_hold_offset = 0.0;

// File-scope interaction session/cooldown and long-run stats. Keeping these
// outside the class avoids requiring behaviour.hpp changes for this test.
static bool g_interaction_session_active = false;
static rclcpp::Time g_interaction_session_start_time;
static bool g_human_detection_cooldown_active = false;
static rclcpp::Time g_human_detection_cooldown_start_time;

static int g_dvd_heading_selection_count = 0;
static int g_human_detection_accepted_count = 0;
static int g_human_detection_ignored_cooldown_count = 0;
static int g_human_interaction_success_count = 0;
static int g_human_interaction_completed_count = 0;
static int g_human_abandoned_before_interaction_count = 0;
static int g_human_abandoned_after_interaction_count = 0;


enum class HumanMotionPhase
{
    APPROACH,
    REPOSITION,
    ESCAPE,
    SETTLE,
    INTERACTION,
    RECOVERY
};

enum class HumanTargetConfidence
{
    LOST,
    PREDICTED,
    LIDAR_TRACKED,
    CONFIRMED
};

enum class ProactiveAvoidanceStage
{
    NONE,
    CLEAR_WALL,
    RETURN_TO_HUMAN
};


enum class DeterministicEscapeStage
{
    NONE,
    BACK_OFF,
    CLEAR_ARC,
    RETURN_ARC
};

static HumanMotionPhase g_human_motion_phase = HumanMotionPhase::APPROACH;
static bool g_reposition_hysteresis_active = false;
static bool g_reposition_command_active = false;
static rclcpp::Time g_reposition_command_start_time;
static double g_reposition_committed_heading = 0.0;
static double g_reposition_committed_linear = 0.0;
static double g_reposition_committed_angular = 0.0;
static int g_reposition_start_blocked_beams = 0;
static double g_reposition_start_min_margin = 0.0;
static rclcpp::Time g_interaction_settle_start_time;
static int g_interaction_settle_good_samples = 0;
static int g_interaction_settle_total_samples = 0;

static int g_human_reposition_commit_count = 0;
static int g_human_settle_attempt_count = 0;
static int g_human_settle_success_count = 0;
static int g_human_settle_rejected_count = 0;
static int g_human_reposition_no_improvement_count = 0;

static double g_debug_last_human_range = -1.0;
static double g_debug_last_human_offset = 0.0;
static int g_debug_last_blocked_beams = 0;
static double g_debug_last_min_clearance = std::numeric_limits<double>::infinity();
static double g_debug_last_min_margin = std::numeric_limits<double>::infinity();
static double g_debug_last_reposition_heading = 0.0;

// Fused human estimate in the robot frame.
static bool g_fused_human_valid = false;
static double g_fused_human_bearing = 0.0;
static double g_fused_human_range = -1.0;
static double g_fused_human_safety_range = -1.0;
static rclcpp::Time g_fused_human_update_time;
static rclcpp::Time g_fused_human_last_lidar_time;
static bool g_fused_human_lidar_ever_valid = false;

// Persistent target snapshot in the current robot frame. The fused bearing/range
// above are derived from this point so the target continues to move correctly
// through the scan as the base translates and rotates.
static HumanTargetConfidence g_human_target_confidence = HumanTargetConfidence::LOST;
static bool g_human_target_locked = false;
static double g_human_target_x = 0.0;
static double g_human_target_y = 0.0;
static double g_human_target_safety_range = -1.0;
static int g_human_target_snapshot_point_count = 0;
static double g_human_target_snapshot_angular_width = 0.0;
static rclcpp::Time g_human_target_last_confirmed_time;
static rclcpp::Time g_human_target_last_lidar_match_time;
static bool g_human_target_imu_yaw_valid = false;
static double g_human_target_last_imu_yaw = 0.0;
static int g_human_target_lock_count = 0;
static int g_human_target_lidar_reassociation_count = 0;
static int g_human_target_prediction_count = 0;
static int g_human_target_lost_count = 0;

// Diagnostic state for the camera-led fallback. This mode is intentionally not
// a separate navigation state: normal approach continues, with reduced speed,
// until a real LiDAR obstacle threatens the path or a human range guard becomes
// available.
static bool g_human_camera_guided_mode_active = false;
static bool g_human_camera_guided_lidar_guard_valid = false;
static double g_human_camera_guided_guard_range =
    std::numeric_limits<double>::infinity();
static int g_human_camera_guided_cycle_count = 0;

// Short-horizon command latch.
static bool g_human_planner_command_active = false;
static rclcpp::Time g_human_planner_command_start_time;
static double g_human_planner_linear = 0.0;
static double g_human_planner_angular = 0.0;
static double g_human_planner_score = -std::numeric_limits<double>::infinity();
static double g_human_planner_predicted_min_human_distance = -1.0;
static double g_human_planner_predicted_final_range = -1.0;
static double g_human_planner_predicted_final_bearing = 0.0;
static int g_human_planner_safe_candidates = 0;
static int g_human_planner_rejected_human_distance = 0;
static int g_human_planner_rejected_obstacle = 0;
static int g_human_planner_rejected_view = 0;

// Last associated LiDAR segment diagnostics.
static bool g_human_cluster_valid = false;
static int g_human_cluster_start_index = -1;
static int g_human_cluster_end_index = -1;
static int g_human_cluster_point_count = 0;
static double g_human_cluster_angle = 0.0;
static double g_human_cluster_median_range = -1.0;
static double g_human_cluster_safety_range = -1.0;
static double g_human_cluster_score = 0.0;

static int g_human_lidar_association_success_count = 0;
static int g_human_lidar_association_failure_count = 0;
static int g_human_planner_command_count = 0;
static int g_human_planner_no_safe_command_count = 0;
static int g_human_minimum_distance_stop_count = 0;
static int g_human_visual_recovery_count = 0;

// Stagnation and forced escape state.
static bool g_human_escape_active = false;
static bool g_human_stagnation_tracking = false;
static rclcpp::Time g_human_stagnation_start_time;
static rclcpp::Time g_human_escape_start_time;
static double g_human_stagnation_start_margin = 0.0;
static int g_human_stagnation_start_blocked_beams = 0;
static double g_human_escape_direction_angle = 0.0;
static int g_human_escape_direction_sign = 0;
static int g_human_escape_trigger_count = 0;
static int g_human_escape_complete_count = 0;
static int g_human_escape_timeout_count = 0;


// Deterministic fallback used when all sampled escape arcs are rejected.
static DeterministicEscapeStage g_human_deterministic_escape_stage =
    DeterministicEscapeStage::NONE;
static rclcpp::Time g_human_deterministic_escape_stage_start_time;
static int g_human_deterministic_escape_side_sign = 0;
static int g_human_no_safe_command_streak = 0;
static int g_human_deterministic_escape_trigger_count = 0;
static int g_human_deterministic_escape_complete_count = 0;
static int g_human_deterministic_escape_blocked_count = 0;

// Adaptive S-curve waypoint state. The existing deterministic stage enum is
// reused: BACK_OFF creates human-distance room when required, CLEAR_ARC moves
// toward the temporary clearance waypoint, and RETURN_ARC counter-steers back
// toward the human.
static int g_human_waypoint_trigger_streak = 0;
static double g_human_waypoint_clearance_deficit = 0.0;
static double g_human_waypoint_target_lateral = 0.0;
static double g_human_waypoint_target_forward = 0.0;
static double g_human_waypoint_arc_duration = 0.0;
static double g_human_waypoint_initial_margin = 0.0;
static int g_human_waypoint_start_count = 0;
static int g_human_waypoint_complete_count = 0;
static int g_human_waypoint_abort_count = 0;
static rclcpp::Time g_human_waypoint_total_start_time;

static double g_human_last_rear_clearance =
    std::numeric_limits<double>::infinity();
static double g_human_last_front_clearance =
    std::numeric_limits<double>::infinity();

// Too-close-human backoff and future voice interface.
static bool g_human_too_close_backoff_active = false;
static int g_human_too_close_backoff_count = 0;
static int g_human_step_back_prompt_count = 0;
static rclcpp::Time g_last_human_step_back_prompt_time;
static bool g_human_step_back_prompt_time_initialised = false;

// Proactive approach-path diagnostics. This becomes active when a short
// straight-ahead projection predicts that the robot would enter or worsen the
// interaction collision zone before reaching the human.
static bool g_human_proactive_avoidance_active = false;
static double g_human_proactive_direction_angle = 0.0;
static int g_human_proactive_trigger_count = 0;
static int g_human_proactive_risk_cycles = 0;
static int g_human_proactive_clear_cycles = 0;
static int g_human_proactive_threat_point_count = 0;
static double g_human_proactive_nearest_nonhuman_range =
    std::numeric_limits<double>::infinity();
static double g_human_proactive_nearest_threat_range =
    std::numeric_limits<double>::infinity();
static int g_human_proactive_projected_blocked_beams = 0;
static double g_human_proactive_projected_margin =
    std::numeric_limits<double>::infinity();
static ProactiveAvoidanceStage g_human_proactive_stage =
    ProactiveAvoidanceStage::NONE;
static rclcpp::Time g_human_proactive_stage_start_time;
static int g_human_proactive_side_sign = 0;
static int g_human_proactive_entry_threat_points = 0;
static double g_human_proactive_entry_nearest_threat =
    std::numeric_limits<double>::infinity();
static int g_human_proactive_return_count = 0;

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


enum class VoiceGroup
{
    AUTONOMOUS,
    PERSON_DETECTED,
    PERSON_LOST,
    CENTERING,
    NO_SPACE,
    MIMICRY
};

static std::unordered_map<VoiceGroup, std::size_t> g_voice_group_indices;
static rclcpp::Time g_last_voice_play_time;
static bool g_last_voice_play_time_initialised = false;

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

static const std::vector<std::string> &voiceLinesForGroup(VoiceGroup group)
{
    static const std::vector<std::string> autonomous_lines = {
        "OneDrive_2026-06-12/Autonomous Mode/hmm_nothing_yet.mp3",
        "OneDrive_2026-06-12/Autonomous Mode/hmmmm_hmmmm.mp3",
        "OneDrive_2026-06-12/Autonomous Mode/Ive_moved_beyond.mp3",
        "OneDrive_2026-06-12/Autonomous Mode/wandering.mp3",
        "OneDrive_2026-06-12/Autonomous Mode/I_had_stringsmp3.mp3",
    };
    static const std::vector<std::string> person_detected_lines = {
        "OneDrive_2026-06-12 (2)/Person Detected/found_one.mp3",
        "OneDrive_2026-06-12 (2)/Person Detected/a_human_interesting.mp3",
    };
    static const std::vector<std::string> person_lost_lines = {
        "OneDrive_2026-06-12 (3)/Person Lost/searching_again.mp3",
        "OneDrive_2026-06-12 (3)/Person Lost/still_looking.mp3",
        "OneDrive_2026-06-12 (1)/Grace Period/where_did_you_go.mp3",
    };
    static const std::vector<std::string> centering_lines = {
        "Centering-Adjusting/getting_better_look.mp3",
    };
    static const std::vector<std::string> no_space_lines = {
        "Centering-Adjusting/Please_move_I_have_no_space.mp3",
    };
    static const std::vector<std::string> mimicry_lines = {
        "Mimicing/interessting_choice.mp3",
        "Mimicing/you_make_it_look_easy.mp3",
    };

    switch (group)
    {
    case VoiceGroup::PERSON_DETECTED:
        return person_detected_lines;
    case VoiceGroup::PERSON_LOST:
        return person_lost_lines;
    case VoiceGroup::CENTERING:
        return centering_lines;
    case VoiceGroup::NO_SPACE:
        return no_space_lines;
    case VoiceGroup::MIMICRY:
        return mimicry_lines;
    case VoiceGroup::AUTONOMOUS:
    default:
        return autonomous_lines;
    }
}

static void playVoiceLine(VoiceGroup group, const rclcpp::Time &now,
    double min_interval_seconds = 6.0, bool force = false)
{
    if (!force && g_last_voice_play_time_initialised &&
        (now - g_last_voice_play_time).seconds() < min_interval_seconds)
    {
        return;
    }

    const std::vector<std::string> &lines = voiceLinesForGroup(group);
    if (lines.empty())
    {
        return;
    }

    std::size_t &index = g_voice_group_indices[group];
    const std::string relative_path = lines[index % lines.size()];
    index = (index + 1) % lines.size();

    const std::string full_path =
        behaviourResourceDirectory() + "/" + relative_path;
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
        << "\" >/dev/null 2>&1 &";

    std::system(command.str().c_str());
    g_last_voice_play_time = now;
    g_last_voice_play_time_initialised = true;
}

static double normaliseAngleForFileScope(double angle_rad)
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

static bool isRearBeamAngle(double angle_rad)
{
    const double angle = normaliseAngleForFileScope(angle_rad);
    return std::fabs(angle) >= (M_PI - kHumanRearSectorHalfAngle);
}

static double humanModeSafetyRadiusForAngle(double angle_rad)
{
    // During HUMAN_DETECTED this is only the hard-stop safety radius for base
    // repositioning. The full arm-clearance radius is checked separately by
    // humanModeInteractionBubbleRadiusForAngle().
    return isRearBeamAngle(angle_rad)
        ? kHumanModeRearHardStopRadius
        : kHumanModeFrontSideHardStopRadius;
}

static double humanModeInteractionBubbleRadiusForAngle(double angle_rad)
{
    return isRearBeamAngle(angle_rad)
        ? kHumanRearSafetyRadius
        : (kInteractionBubbleRadius + kInteractionBubbleSafetyMargin);
}


static const char *humanMotionPhaseName(HumanMotionPhase phase)
{
    switch (phase)
    {
    case HumanMotionPhase::APPROACH: return "APPROACH";
    case HumanMotionPhase::REPOSITION: return "REPOSITION";
    case HumanMotionPhase::ESCAPE: return "ESCAPE";
    case HumanMotionPhase::SETTLE: return "SETTLE";
    case HumanMotionPhase::INTERACTION: return "INTERACTION";
    case HumanMotionPhase::RECOVERY: return "RECOVERY";
    default: return "UNKNOWN";
    }
}

static const char *humanTargetConfidenceName(HumanTargetConfidence confidence)
{
    switch (confidence)
    {
    case HumanTargetConfidence::CONFIRMED: return "CONFIRMED";
    case HumanTargetConfidence::LIDAR_TRACKED: return "LIDAR_TRACKED";
    case HumanTargetConfidence::PREDICTED: return "PREDICTED";
    case HumanTargetConfidence::LOST: return "LOST";
    default: return "UNKNOWN";
    }
}

static const char *proactiveAvoidanceStageName(ProactiveAvoidanceStage stage)
{
    switch (stage)
    {
    case ProactiveAvoidanceStage::CLEAR_WALL: return "CLEAR_WALL";
    case ProactiveAvoidanceStage::RETURN_TO_HUMAN: return "RETURN_TO_HUMAN";
    case ProactiveAvoidanceStage::NONE: return "NONE";
    default: return "UNKNOWN";
    }
}


static const char *deterministicEscapeStageName(DeterministicEscapeStage stage)
{
    switch (stage)
    {
    case DeterministicEscapeStage::BACK_OFF: return "BACK_OFF";
    case DeterministicEscapeStage::CLEAR_ARC: return "CLEAR_ARC";
    case DeterministicEscapeStage::RETURN_ARC: return "RETURN_ARC";
    case DeterministicEscapeStage::NONE: return "NONE";
    default: return "UNKNOWN";
    }
}

static void publishHumanStepBackPrompt(const rclcpp::Time &now)
{
    const bool repeat_elapsed =
        !g_human_step_back_prompt_time_initialised ||
        (now - g_last_human_step_back_prompt_time).seconds() >=
            kHumanStepBackPromptRepeatSeconds;

    if (!repeat_elapsed)
    {
        return;
    }

    if (g_voice_prompt_publisher)
    {
        std_msgs::msg::String prompt;
        // A future audio node can subscribe to /voice_prompt and speak this text.
        prompt.data = "Please take one step back so I have enough room to interact safely.";
        g_voice_prompt_publisher->publish(prompt);
    }

    g_last_human_step_back_prompt_time = now;
    g_human_step_back_prompt_time_initialised = true;
    g_human_step_back_prompt_count++;
}

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

static void resetHumanMotionController()
{
    g_human_motion_phase = HumanMotionPhase::APPROACH;
    g_reposition_hysteresis_active = false;
    g_reposition_command_active = false;
    g_reposition_committed_heading = 0.0;
    g_reposition_committed_linear = 0.0;
    g_reposition_committed_angular = 0.0;
    g_reposition_start_blocked_beams = 0;
    g_reposition_start_min_margin = 0.0;
    g_interaction_settle_good_samples = 0;
    g_interaction_settle_total_samples = 0;

    g_fused_human_valid = false;
    g_fused_human_bearing = 0.0;
    g_fused_human_range = -1.0;
    g_fused_human_safety_range = -1.0;
    g_fused_human_lidar_ever_valid = false;
    g_human_target_confidence = HumanTargetConfidence::LOST;
    g_human_target_locked = false;
    g_human_target_x = 0.0;
    g_human_target_y = 0.0;
    g_human_target_safety_range = -1.0;
    g_human_target_snapshot_point_count = 0;
    g_human_target_snapshot_angular_width = 0.0;
    g_human_target_imu_yaw_valid = false;
    g_human_planner_command_active = false;
    g_human_planner_linear = 0.0;
    g_human_planner_angular = 0.0;
    g_human_escape_active = false;
    g_human_stagnation_tracking = false;
    g_human_escape_direction_angle = 0.0;
    g_human_escape_direction_sign = 0;
    g_human_deterministic_escape_stage = DeterministicEscapeStage::NONE;
    g_human_deterministic_escape_side_sign = 0;
    g_human_no_safe_command_streak = 0;
    g_human_waypoint_trigger_streak = 0;
    g_human_waypoint_clearance_deficit = 0.0;
    g_human_waypoint_target_lateral = 0.0;
    g_human_waypoint_target_forward = 0.0;
    g_human_waypoint_arc_duration = 0.0;
    g_human_waypoint_initial_margin = 0.0;
    g_human_too_close_backoff_active = false;
    g_human_proactive_avoidance_active = false;
    g_human_proactive_direction_angle = 0.0;
    g_human_proactive_risk_cycles = 0;
    g_human_proactive_clear_cycles = 0;
    g_human_proactive_threat_point_count = 0;
    g_human_proactive_nearest_nonhuman_range =
        std::numeric_limits<double>::infinity();
    g_human_proactive_nearest_threat_range =
        std::numeric_limits<double>::infinity();
    g_human_proactive_projected_blocked_beams = 0;
    g_human_proactive_projected_margin =
        std::numeric_limits<double>::infinity();
    g_human_proactive_stage = ProactiveAvoidanceStage::NONE;
    g_human_proactive_side_sign = 0;
    g_human_proactive_entry_threat_points = 0;
    g_human_proactive_entry_nearest_threat =
        std::numeric_limits<double>::infinity();
    g_human_cluster_valid = false;
    g_human_cluster_start_index = -1;
    g_human_cluster_end_index = -1;
    g_human_cluster_point_count = 0;
    g_human_camera_guided_mode_active = false;
    g_human_camera_guided_lidar_guard_valid = false;
    g_human_camera_guided_guard_range =
        std::numeric_limits<double>::infinity();

    g_human_centre_locked = false;
    g_human_turn_cycle_initialised = false;
    g_human_forward_cycle_initialised = false;
    g_human_stop_wait_active = false;
    g_human_settle_good_samples = 0;
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
  safety_zone_violated_(false),
  safety_zone_baseline_captured_(false),
  current_state_(NavigationState::SEARCHING),
  target_angle_(0.0),
  target_range_(0.0),
  stop_distance_m_(1.5),
  stop_counter_(0),
  imu_available_(false),
  align_start_yaw_(0.0),
  align_yaw_initialised_(false),
  random_engine_(std::random_device{}()),
  turn_dist_(-1.0, 1.0)
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
            [](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active &&
                    g_human_motion_phase == HumanMotionPhase::INTERACTION &&
                    g_right_arm_pose_publisher)
                {
                    g_right_arm_pose_publisher->publish(*msg);
                    g_arm_mimicry_forward_count++;
                }
                else
                {
                    g_arm_mimicry_ignored_count++;
                }
            });
    g_left_mimicry_pose_subscriber =
        this->create_subscription<std_msgs::msg::String>(
            "/arm/mimicry_left_pose",
            10,
            [](const std_msgs::msg::String::SharedPtr msg)
            {
                if (g_interaction_session_active &&
                    g_human_motion_phase == HumanMotionPhase::INTERACTION &&
                    g_left_arm_pose_publisher)
                {
                    g_left_arm_pose_publisher->publish(*msg);
                    g_arm_mimicry_forward_count++;
                }
                else
                {
                    g_arm_mimicry_ignored_count++;
                }
            });

    g_last_arm_down_publish_time = this->now();
    g_arm_down_publish_time_initialised = false;
    g_last_interaction_active_publish_time = this->now();
    g_interaction_active_publish_time_initialised = false;
    g_last_interaction_active_value = false;

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
    safety_zone_violated_ = false;
    safety_zone_baseline_captured_ = false;
    // DVD behaviour begins by driving straight. A bounce heading is selected
    // only after the filtered LiDAR reports a wall in front.
    current_state_ = NavigationState::MOVING;
    target_angle_ = 0.0;
    target_range_ = 0.0;
    stop_counter_ = 0;
    align_yaw_initialised_ = false;
    g_interaction_hold_active = false;
    g_human_settle_good_samples = 0;
    g_human_centre_locked = false;
    g_human_turn_cycle_initialised = false;
    resetHumanMotionController();
    clearObstacleMarkers();
}

void MechelangeloBehaviour::mappedAutonomous()
{
    RCLCPP_INFO(this->get_logger(), "Executing mapped autonomous behaviour.");
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
        playVoiceLine(VoiceGroup::AUTONOMOUS, this->now(), 20.0);
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
            playVoiceLine(VoiceGroup::NO_SPACE, this->now(), 10.0);
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
            playVoiceLine(VoiceGroup::AUTONOMOUS, this->now(), 22.0);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "MOVING: Front is clear after filtering. Driving forward.");
        }
        else
        {
            playVoiceLine(VoiceGroup::AUTONOMOUS, this->now(), 22.0);
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

    case NavigationState::STOPPED:
    {
        stopRobot(twist);
        stop_counter_++;

        if (stop_counter_ >= kStopDurationLoops)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "STOPPED: Pause complete. Searching for next direction.");

            stop_counter_ = 0;
            clearObstacleMarkers();
            current_state_ = NavigationState::SEARCHING;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "STOPPED: Pausing %.1f / %.1f seconds",
                stop_counter_ * kControlPeriodSeconds,
                kStopDurationLoops * kControlPeriodSeconds);
        }

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

            const double interaction_elapsed =
                (now - g_interaction_session_start_time).seconds();

            if (interaction_elapsed >= kHumanInteractionDurationSeconds)
            {
                g_human_interaction_completed_count++;
                g_interaction_session_active = false;
                g_interaction_hold_active = false;
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
                g_human_abandoned_before_interaction_count++;
                human_locked_ = false;
                human_tracking_valid_ = false;
                human_tracking_grace_active_ = false;
                g_interaction_hold_active = false;
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
            playVoiceLine(VoiceGroup::CENTERING, now, 8.0);
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

                g_human_abandoned_before_interaction_count++;
                g_human_detection_cooldown_active = true;
                g_human_detection_cooldown_start_time = now;
                human_locked_ = false;
                g_interaction_hold_active = false;
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
            playVoiceLine(VoiceGroup::MIMICRY, now, 1.0, true);
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            current_twist_ = twist;
            g_human_motion_phase =
                HumanMotionPhase::INTERACTION;
            g_interaction_session_active = true;
            g_interaction_session_start_time = now;
            g_interaction_hold_active = true;
            g_interaction_hold_time = now;
            g_interaction_hold_range_m =
                lidar_range_interact;
            g_interaction_hold_offset =
                human_centre_offset_;
            g_human_interaction_success_count++;
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
        g_human_detection_ignored_cooldown_count++;
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
        g_human_detection_ignored_cooldown_count++;
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
        g_human_detection_accepted_count++;

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
            lidar_at_detection_valid ? "valid" : "no-return");
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
    bool used_preferred_bounce_band = false;
    bool used_left_side = false;
    bool used_right_side = false;

    if (!preferred_left_candidates.empty() || !preferred_right_candidates.empty())
    {
        selected_index = chooseBalancedSide(
            preferred_left_candidates,
            preferred_right_candidates,
            used_left_side,
            used_right_side);
        used_preferred_bounce_band = true;
    }
    else if (!fallback_left_candidates.empty() || !fallback_right_candidates.empty())
    {
        selected_index = chooseBalancedSide(
            fallback_left_candidates,
            fallback_right_candidates,
            used_left_side,
            used_right_side);
        used_preferred_bounce_band = false;
    }
    else if (!fallback_all_candidates.empty())
    {
        selected_index = chooseFromCandidates(fallback_all_candidates);
        used_left_side = angleForIndex(selected_index) >= 0.0;
        used_right_side = !used_left_side;
        used_preferred_bounce_band = false;
    }

    if (selected_index >= 0)
    {
        g_dvd_heading_selection_count++;
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

    g_dvd_heading_selection_count++;
    out_angle = normaliseAngle(latest_scan_.angle_min + best_mid_index * latest_scan_.angle_increment);
    out_range = max_finite_range;

    return true;
}

int MechelangeloBehaviour::angleToIndex(double angle_rad) const
{
    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return -1;
    }

    double capped_angle = angle_rad;

    if (capped_angle < latest_scan_.angle_min)
    {
        capped_angle = latest_scan_.angle_min;
    }

    if (capped_angle > latest_scan_.angle_max)
    {
        capped_angle = latest_scan_.angle_max;
    }

    int index = static_cast<int>(
        std::round((capped_angle - latest_scan_.angle_min) / latest_scan_.angle_increment));

    index = std::clamp(index, 0, static_cast<int>(latest_scan_.ranges.size()) - 1);
    return index;
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

void MechelangeloBehaviour::longestLaserScan()
{
    double longest_angle = 0.0;
    double longest_range = 0.0;

    if (!getLongestRange(longest_angle, longest_range))
    {
        RCLCPP_WARN(
            this->get_logger(),
            "No valid filtered laser scan data available for longest scan calculation.");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Selected exploration heading: Representative range = %.2f m at Angle = %.2f degrees",
        longest_range,
        longest_angle * 180.0 / M_PI);
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

void MechelangeloBehaviour::captureSafetyZoneBaseline()
{
    safety_zone_baseline_scan_ = latest_scan_;
    safety_zone_baseline_captured_ = true;
    RCLCPP_INFO(this->get_logger(), "SAFETY ZONE: Filtered background baseline captured.");
}

bool MechelangeloBehaviour::isSafetyZoneViolated(double human_bearing_rad) const
{
    if (!safety_zone_baseline_captured_)
    {
        return false;
    }

    if (latest_scan_.ranges.empty() || latest_scan_.angle_increment == 0.0)
    {
        return false;
    }

    if (safety_zone_baseline_scan_.ranges.size() != latest_scan_.ranges.size())
    {
        return false;
    }

    for (int i = 0; i < static_cast<int>(latest_scan_.ranges.size()); ++i)
    {
        const double angle = latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
        const double angle_diff = normaliseAngle(angle - human_bearing_rad);

        // Exclude the window around the tracked human so they do not self-trigger.
        if (std::fabs(angle_diff) <= kSafetyZoneHumanExclusionAngle)
        {
            continue;
        }

        const double current_range = latest_scan_.ranges[i];

        if (!std::isfinite(current_range) || current_range <= kMinValidRange)
        {
            continue;
        }

        const double required_safety_radius = humanModeSafetyRadiusForAngle(angle);

        if (current_range >= required_safety_radius)
        {
            continue;
        }

        const double baseline_range = std::isfinite(safety_zone_baseline_scan_.ranges[i])
            ? safety_zone_baseline_scan_.ranges[i]
            : required_safety_radius;

        if (current_range < (baseline_range - kSafetyZoneIntruderThreshold))
        {
            return true;
        }
    }

    return false;
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

