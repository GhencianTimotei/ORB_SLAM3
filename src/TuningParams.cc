#include "TuningParams.h"

#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

// Sentinels: negative means "upstream constant", so a settings file that
// mentions none of these reproduces stock ORB-SLAM3 exactly.
int   Tuning::minFramesBetweenKFs    = -1;
int   Tuning::localBAWindowKFs       = -1;
int   Tuning::localBAIterations      = -1;
int   Tuning::localBAWindowKFsLarge  = -1;
int   Tuning::localBAIterationsLarge = -1;
int   Tuning::imuInitMinKF           = -1;
float Tuning::imuInitMinTime         = -1.0f;
int   Tuning::localMappingTimingKFs  = 0;
int   Tuning::useGravityPrior        = 0;

namespace
{

// cv::FileNode::empty() is true for a missing key; real() on a missing node
// throws in some OpenCV builds, so every read is guarded.
bool readInt(const cv::FileStorage &fs, const std::string &key, int &out)
{
    const cv::FileNode node = fs[key];
    if (node.empty())
        return false;
    if (!node.isInt() && !node.isReal())
    {
        std::cerr << "Tuning: " << key << " is present but not a number; ignoring" << std::endl;
        return false;
    }
    out = static_cast<int>(static_cast<double>(node));
    return true;
}

bool readFloat(const cv::FileStorage &fs, const std::string &key, float &out)
{
    const cv::FileNode node = fs[key];
    if (node.empty())
        return false;
    if (!node.isInt() && !node.isReal())
    {
        std::cerr << "Tuning: " << key << " is present but not a number; ignoring" << std::endl;
        return false;
    }
    out = static_cast<float>(static_cast<double>(node));
    return true;
}

} // namespace

void Tuning::LoadFromSettings(const std::string &strSettingsFile)
{
    cv::FileStorage fs(strSettingsFile, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        // System already fails loudly on an unopenable settings file; do not
        // duplicate that error, just leave every knob at its upstream default.
        return;
    }

    readInt(fs,   "Tracking.minFramesBetweenKFs",  minFramesBetweenKFs);
    readInt(fs,   "LocalMapping.baWindowKFs",      localBAWindowKFs);
    readInt(fs,   "LocalMapping.baIterations",     localBAIterations);
    readInt(fs,   "LocalMapping.baWindowKFsLarge", localBAWindowKFsLarge);
    readInt(fs,   "LocalMapping.baIterationsLarge",localBAIterationsLarge);
    readInt(fs,   "IMU.InitMinKF",                 imuInitMinKF);
    readFloat(fs, "IMU.InitMinTime",               imuInitMinTime);
    readInt(fs,   "LocalMapping.timingLogKFs",     localMappingTimingKFs);
    readInt(fs,   "IMU.useGravityPrior",           useGravityPrior);

    fs.release();
    Print();
}

void Tuning::Print()
{
    auto showInt = [](const char *name, int value, int upstream) {
        std::cout << "  " << name << ": ";
        if (value < 0)
            std::cout << upstream << "  (default)" << std::endl;
        else
            std::cout << value << "  (override, upstream " << upstream << ")" << std::endl;
    };

    std::cout << std::endl << "Tuning overrides:" << std::endl;
    showInt("Tracking.minFramesBetweenKFs",   minFramesBetweenKFs,    0);
    showInt("LocalMapping.baWindowKFs",       localBAWindowKFs,      10);
    showInt("LocalMapping.baIterations",      localBAIterations,     10);
    showInt("LocalMapping.baWindowKFsLarge",  localBAWindowKFsLarge, 25);
    showInt("LocalMapping.baIterationsLarge", localBAIterationsLarge, 4);
    showInt("IMU.InitMinKF",                  imuInitMinKF,          10);
    std::cout << "  IMU.InitMinTime: ";
    if (imuInitMinTime < 0.0f)
        std::cout << "2.0 mono / 1.0 other  (default)" << std::endl;
    else
        std::cout << imuInitMinTime << " s  (override)" << std::endl;
    std::cout << "  LocalMapping.timingLogKFs: " << localMappingTimingKFs
              << (localMappingTimingKFs > 0 ? "  (stage timing on)" : "  (off)") << std::endl;
    std::cout << "  IMU.useGravityPrior: " << useGravityPrior
              << (useGravityPrior ? "  (accept a caller-supplied gravity direction)" : "  (off)")
              << std::endl << std::endl;
}

std::mutex      GravityPrior::mMutex;
Eigen::Vector3f GravityPrior::mDirInBody = Eigen::Vector3f::Zero();
bool            GravityPrior::mbPending  = false;

void GravityPrior::Set(const Eigen::Vector3f &dirInBody)
{
    const float norm = dirInBody.norm();
    if (!std::isfinite(norm) || norm < 1e-3f)
    {
        std::cerr << "GravityPrior: refusing a degenerate direction" << std::endl;
        return;
    }

    std::unique_lock<std::mutex> lock(mMutex);
    mDirInBody = dirInBody / norm;
    mbPending = true;
    std::cout << "GravityPrior: armed, gravity in body frame = ["
              << mDirInBody.transpose() << "]" << std::endl;
}

bool GravityPrior::Take(Eigen::Vector3f &dirInBody)
{
    std::unique_lock<std::mutex> lock(mMutex);
    if (!mbPending)
        return false;
    dirInBody = mDirInBody;
    mbPending = false;
    return true;
}

bool GravityPrior::Pending()
{
    std::unique_lock<std::mutex> lock(mMutex);
    return mbPending;
}

void GravityPrior::Clear()
{
    std::unique_lock<std::mutex> lock(mMutex);
    mbPending = false;
}

} // namespace ORB_SLAM3
