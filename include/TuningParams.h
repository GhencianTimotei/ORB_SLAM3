/**
* Runtime-tunable parameters that upstream ORB-SLAM3 hardcodes.
*
* Every knob here defaults to the exact upstream constant, so a settings file
* that mentions none of them reproduces stock behaviour bit for bit. That
* matters: the EuRoC control run in docu/orbslam_diagnosis_2026-08-28.md
* (finding #23) is only a valid reference if the library still behaves as
* upstream when nothing is overridden.
*
* Values are read once, from the same settings yaml System already opens, and
* held in statics. A settings object is not threaded through Optimizer (whose
* functions are static and take no context), so a process-global is the least
* invasive place for them.
*/

#ifndef TUNINGPARAMS_H
#define TUNINGPARAMS_H

#include <mutex>
#include <string>

#include <Eigen/Core>

namespace ORB_SLAM3
{

class Tuning
{
public:
    // Reads every key below from strSettingsFile. Absent keys leave the
    // corresponding member at its "use upstream" sentinel. Safe to call on a
    // file that has none of them. Must run before Tracking/LocalMapping are
    // constructed.
    static void LoadFromSettings(const std::string &strSettingsFile);

    // Prints the resolved values, marking each as (default) or (override).
    static void Print();

    // ---- Keyframe insertion rate -------------------------------------------
    // Tracking.minFramesBetweenKFs: minimum frames between two keyframes
    // (Tracking::mMinFrames). Upstream hardcodes 0, which makes NeedNewKeyFrame's
    // c1b condition -- "at least mMinFrames since the last KF AND LocalMapping is
    // idle" -- true on almost every frame whenever the host is fast enough to
    // keep LocalMapping idle. Measured on this rig: keyframes at ~10 Hz against
    // a 0.25 s floor the rest of the code is tuned for (finding #20).
    //
    // Every keyframe costs LocalMapping a full CreateNewMapPoints +
    // SearchInNeighbors + LocalInertialBA + KeyFrameCulling cycle, and costs
    // LoopClosing a place-recognition query, so this value is close to a direct
    // divisor on the LocalMapping and LoopClosing thread load.
    //
    // It also widens the keyframe-triplet displacement that LocalMapping's
    // "Not enough motion for initializing" guard tests against absolute 2 cm /
    // 5 cm thresholds (findings #21 and #23), so raising it attacks the reset
    // storm and the CPU load with the same change.
    //
    // -1 = upstream (0). A sensible value is fps/4.
    static int minFramesBetweenKFs;

    // ---- Local inertial bundle adjustment window ---------------------------
    // Optimizer::LocalInertialBA optimises the last Nd keyframes for opt_it
    // iterations. Upstream picks between two hardcoded pairs on `bLarge`:
    //
    //   bLarge == false  ->  Nd = 10, iterations = 10
    //   bLarge == true   ->  Nd = 25, iterations = 4
    //
    // and bLarge is `(matchesInliers > 75) && monocular`. This rig runs 89-263
    // inliers (median 204, finding #20), so it is permanently in the 25-keyframe
    // branch -- the wider window, with correspondingly more map points and
    // inertial edges per iteration. These knobs make that window measurable and
    // bounded instead of implicit.
    //
    // -1 = upstream.
    static int localBAWindowKFs;        // upstream 10
    static int localBAIterations;       // upstream 10
    static int localBAWindowKFsLarge;   // upstream 25
    static int localBAIterationsLarge;  // upstream 4

    // ---- Inertial initialisation gates -------------------------------------
    // LocalMapping::InitializeIMU refuses to run until the map holds at least
    // imuInitMinKF keyframes spanning at least imuInitMinTime seconds. Upstream
    // hardcodes 10 keyframes / 2.0 s for monocular, 10 / 1.0 s otherwise. This
    // wait is exactly the "drone has no idea where it is" window at startup.
    //
    // Lower these ONLY once initialisation is better conditioned than upstream
    // assumes -- e.g. with IMU.useGravityPrior below, which removes the gravity
    // direction from the set of unknowns the first InertialOptimization must
    // solve. Lowering them on their own buys a faster wait at the cost of a
    // worse gravity/scale estimate that is then baked into the map permanently.
    //
    // -1 / negative = upstream.
    static int   imuInitMinKF;
    static float imuInitMinTime;

    // ---- Instrumentation ---------------------------------------------------
    // LocalMapping.timingLogKFs: print a mean/max millisecond breakdown of the
    // LocalMapping stages every N keyframes. 0 disables. This is the cheap
    // equivalent of building the whole library with -DREGISTER_TIMES, and it is
    // what attributes the LocalMapping thread's share of process CPU to a
    // specific stage.
    static int localMappingTimingKFs;

    // ---- Startup gravity prior ---------------------------------------------
    // IMU.useGravityPrior: consume a gravity direction measured by the caller
    // while the platform is at rest, instead of estimating it inside
    // InitializeIMU by integrating keyframe velocities. 0 disables.
    static int useGravityPrior;
};

/**
 * A gravity direction measured outside the SLAM system, valid only while the
 * platform is demonstrably at rest.
 *
 * Upstream InitializeIMU recovers the gravity direction from the accumulated
 * preintegrated velocity across the whole keyframe window, which is why it needs
 * seconds of motion before it can run at all. An accelerometer sitting still
 * measures that same direction directly and instantly. The caller is responsible
 * for proving "still" -- see the rest test in orbslam_imu_node.cpp -- because a
 * prior taken during motion is worse than no prior at all.
 *
 * Deliberately one-shot: Take() clears it. A prior is only ever truthful for the
 * pre-takeoff map, and must not be reapplied to a map that is re-initialised
 * mid-flight.
 */
class GravityPrior
{
public:
    // dirInBody: unit vector along gravity (the direction it pulls), expressed
    // in the IMU body frame. For PX4 FRD at rest this is approximately (0,0,1).
    static void Set(const Eigen::Vector3f &dirInBody);

    // Returns false and leaves dirInBody untouched if no prior is pending.
    // Clears the prior on success.
    static bool Take(Eigen::Vector3f &dirInBody);

    static bool Pending();

    static void Clear();

private:
    static std::mutex mMutex;
    static Eigen::Vector3f mDirInBody;
    static bool mbPending;
};

} // namespace ORB_SLAM3

#endif // TUNINGPARAMS_H
