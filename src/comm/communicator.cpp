/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

// COVINS
#include "comm/communicator.hpp"
#include <covins/covins_base/utils_base.hpp>

// ORB-SLAM3
#include "Atlas.h"
#include "KeyFrame.h"
#include "MapPoint.h"

// Sockets
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <cstdint>
#include <cstdlib>
#include <sys/time.h>
#include <sys/types.h>

namespace ORB_SLAM3 {

static constexpr int kConnectTimeoutMs = 5000;
static constexpr int kSendTimeoutS = 10;

Communicator::Communicator(std::string server_ip, std::string port, Atlas* map)
    : CommunicatorBase()
{
    covins_params::ShowParamsComm();

    map_ = map;

    if(const char* join_env = std::getenv("COVINS_JOIN_MAP"); join_env && *join_env && std::atoi(join_env) >= 0) {
        join_map_id_ = std::atoi(join_env);
        join_mode_ = true;
    }

    std::cout << "--> Connect to server" << std::endl;
    newfd_ = ConnectToServer(server_ip.c_str(),port);
    if(newfd_ <= 2){
        std::cerr << COUTWARN << "Could not connect to COVINS server " << server_ip << ":" << port << std::endl;
        connected_ = false;
        return;
    }

    if(join_mode_) {
        // Join request the server peeks for right after accept(): "JOIN" + int32 map id, little endian.
        const int32_t id = join_map_id_;
        uint8_t req[8] = {'J', 'O', 'I', 'N', uint8_t(id), uint8_t(id >> 8), uint8_t(id >> 16), uint8_t(id >> 24)};
        if(send(newfd_, req, sizeof(req), MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof(req))) {
            std::cerr << "COVINS: JOIN request failed, COVINS off" << std::endl;
            close(newfd_);
            connected_ = false;
            return;
        }
        std::cout << "COVINS: joining server map " << join_map_id_ << std::endl;
    }

    connected_ = true;
    // A hung server would otherwise block send() forever; the timeout makes it fail and drop the link.
    const timeval send_timeout{kSendTimeoutS, 0};
    setsockopt(newfd_, SOL_SOCKET, SO_SNDTIMEO, &send_timeout, sizeof(send_timeout));
    std::cout << "newfd_: " << newfd_ << std::endl;
    std::cout << "--> Connected" << std::endl;
    if(join_mode_) kf_id_offset_ = 0;  // loaded KFs hold the low ids, so new ids are never 0
}

// Same as the base class, but bounded by kConnectTimeoutMs instead of the kernel's multi-minute default.
auto Communicator::ConnectToServer(const char *node, std::string port)->int {
    addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    addrinfo *servinfo = nullptr;
    if(getaddrinfo(node, port.c_str(), &hints, &servinfo) != 0)
        return 1;

    int sockfd = -1;
    for(addrinfo *p = servinfo; p != nullptr; p = p->ai_next) {
        const int fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if(fd == -1)
            continue;

        const int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        int rc = connect(fd, p->ai_addr, p->ai_addrlen);
        if(rc == -1 && errno == EINPROGRESS) {
            pollfd pfd;
            pfd.fd = fd;
            pfd.events = POLLOUT;
            pfd.revents = 0;
            rc = -1;
            if(poll(&pfd, 1, kConnectTimeoutMs) == 1) {
                int err = 0;
                socklen_t len = sizeof(err);
                if(getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0 && err == 0)
                    rc = 0;
            }
        }

        if(rc == 0) {
            fcntl(fd, F_SETFL, flags);
            sockfd = fd;
            break;
        }
        close(fd);
    }
    freeaddrinfo(servinfo);

    return sockfd >= 0 ? sockfd : 2;
}

auto Communicator::ProcessAdditional()->void {

}

auto Communicator::Stop()->void {
    std::unique_lock<std::mutex> lock_convert(mtx_convert_);
    stopped_ = true;
    {
        std::unique_lock<std::mutex> lock_queue(mtx_kf_queue_);
        kf_out_buffer_.clear();
    }
    SetFinish();
}

auto Communicator::Disconnect()->void {
    if(connected_) shutdown(newfd_, SHUT_RDWR);
}

auto Communicator::QueuedKeyframes()->size_t {
    std::unique_lock<std::mutex> lock(mtx_kf_queue_);
    return kf_out_buffer_.size();
}

auto Communicator::ProcessKfBuffer()->void {
    int cnt = 0;

    while(cnt < covins_params::comm::max_sent_kfs_per_iteration) {
        std::unique_lock<std::mutex> lock_convert(mtx_convert_);
        if(stopped_) break;
        KeyFrame* kfi = nullptr;
        {
            std::unique_lock<std::mutex> lock_queue(mtx_kf_queue_);
            if(kf_out_buffer_.empty()) break;
            kfi = kf_out_buffer_.front();
            kf_out_buffer_.pop_front();
        }

        if(kfi->sent_once_ && !covins_params::comm::send_updates) continue;
        if(kfi->sent_once_ && static_cast<long>(kfi->mnId) == kf_id_offset_) continue;
        covins::data_bundle map_chunk;
        covins::MsgKeyframe msg_kf;
        const bool is_update = kfi->sent_once_;
        KeyFrame* kf_ref = kfi->mPrevKF;
        while(kf_ref && kf_ref->isBad() && kf_ref->mPrevKF) kf_ref = kf_ref->mPrevKF;
        // A new chain after relocalization has no predecessor; the server needs a reference for all but the first KF.
        if(!kf_ref && static_cast<long>(kfi->mnId) != kf_id_offset_ && last_sent_kf_ != kfi) kf_ref = last_sent_kf_;
        const long offset = kf_id_offset_ < 0 ? static_cast<long>(kfi->mnId) : kf_id_offset_;
        if(!kfi->ConvertToMsg(msg_kf,kf_ref,is_update,client_id_,offset)) continue;
        kf_id_offset_ = offset;
        static const bool debug = getenv("COVINS_DEBUG") != nullptr;
        if(debug)
            std::cout << "COVINS_DEBUG send KF " << kfi->mnId << (is_update ? " upd" : " new") << " ref "
                      << (kf_ref ? (long)kf_ref->mnId : -1L) << (kfi->isBad() ? " BAD" : "") << std::endl;
        kfi->sent_once_ = true;
        if(!is_update) last_sent_kf_ = kfi;
        map_chunk.keyframes.push_back(msg_kf);
        if(!is_update) cnt++;
        auto kfi_lms = kfi->GetMapPointMatches();
        for(auto lmi : kfi_lms){
            if(!lmi || lmi->isBad()) continue;
            if(lmi->sent_once_ && !covins_params::comm::send_updates) continue;
            covins::MsgLandmark msg_lm;
            lmi->ConvertToMsg(msg_lm,kfi,lmi->sent_once_,client_id_,kf_id_offset_);
            lmi->sent_once_ = true;
            map_chunk.landmarks.push_back(msg_lm);
        }
        this->PassDataBundle(map_chunk);
    }
}

auto Communicator::ProcessKeyframeMessages()->void {
    std::unique_lock<std::mutex> lock(mtx_in_);
    while(!buffer_keyframes_in_.empty()) {
        covins::MsgKeyframe msg = buffer_keyframes_in_.front();
        buffer_keyframes_in_.pop_front();
        if(kf_id_offset_ >= 0 && static_cast<int>(msg.id.second) == client_id_)
            UpdateCorrection(msg);
    }
}

auto Communicator::UpdateCorrection(const covins::MsgKeyframe &msg)->void {
    // Patched server: T_sref_s is the KF pose in the merged server map. Stock: relative to our first KF.
    const bool shared_frame = msg.id_reference.first == static_cast<size_t>(KFRANGE);
    // MAPRANGE: earlier server patch without anchor fallback, always anchored on client 0.
    const int anchor_client = msg.id_reference.second == static_cast<size_t>(MAPRANGE)
                              ? 0 : static_cast<int>(msg.id_reference.second);
    const unsigned long kf_id = msg.id.first + kf_id_offset_;
    // Tracking clears the map right after Stop(), which waits on this lock.
    std::unique_lock<std::mutex> lock_convert(mtx_convert_);
    if(stopped_) return;
    KeyFrame* kf = nullptr;
    KeyFrame* kf0 = nullptr;
    for(KeyFrame* pKF : map_->GetAllKeyFrames()) {
        if(pKF->isBad()) continue;
        if(pKF->mnId == kf_id) kf = pKF;
        if(pKF->mnId == static_cast<unsigned long>(kf_id_offset_)) kf0 = pKF;
    }
    if(!kf || (!shared_frame && !kf0)) return;

    Eigen::Matrix4d T_W_s = msg.T_sref_s;
    if(!shared_frame) T_W_s = kf0->GetImuPose().matrix().cast<double>() * msg.T_sref_s;
    const Eigen::Matrix4d T_W_A = T_W_s * kf->GetImuPose().inverse().matrix().cast<double>();

    std::unique_lock<std::mutex> lock(mtx_correction_);
    const Eigen::Matrix4d dT = correction_.T_W_A.inverse() * T_W_A;
    const double dAngle = Eigen::AngleAxisd(Eigen::Matrix3d(dT.block<3,3>(0,0))).angle() * 180.0 / M_PI;
    correction_.T_W_A = T_W_A;
    corr_kf_ = kf;
    T_W_corr_kf_ = T_W_s;
    correction_.kf_id = kf_id;
    correction_.shared_frame = shared_frame;
    correction_.anchor_client = anchor_client;
    correction_.seq++;
    std::cout << "COVINS: correction #" << correction_.seq << " from KF " << kf_id
              << (shared_frame ? " (server map, anchor " + std::to_string(anchor_client) + ")" : " (own first KF)")
              << " |t| " << T_W_A.block<3,1>(0,3).norm() << " m, step " << dT.block<3,1>(0,3).norm()
              << " m / " << dAngle << " deg" << std::endl;
}

auto Communicator::GetCorrection(Correction &out) const->bool {
    std::unique_lock<std::mutex> lock(mtx_correction_);
    if(correction_.seq == 0) return false;
    out = correction_;
    // Live against the KF's current local pose: VIBA / scale refinement re-align the local map between messages.
    out.T_W_A = T_W_corr_kf_ * corr_kf_->GetImuPose().inverse().matrix().cast<double>();
    return true;
}

auto Communicator::ProcessLandmarkMessages()->void {

}

auto Communicator::ProcessNewKeyframes()->void {

}

auto Communicator::ProcessNewLandmarks()->void {

}

auto Communicator::Run()->void {
    std::thread thread_recv(&Communicator::RecvMsg, this);
    thread_recv.detach();

    while(true)
    {
        this->ProcessKfBuffer();
        this->ProcessBufferOut();
        this->ProcessBufferIn();
        if(this->TryLock()){
            this->ProcessKeyframeMessages();
            this->ProcessLandmarkMessages();
            this->ProcessNewKeyframes();
            this->ProcessNewLandmarks();
            this->ProcessAdditional();
            this->UnLock();
        }

        if(this->ShallFinish()){
            std::unique_lock<std::mutex> lock(mtx_kf_queue_);
            if(!kf_out_buffer_.empty()) {
                std::cout << "Comm:: waiting for kf_out_buffer_" << std::endl;
            } else {
                std::cout << "Comm " << client_id_ << ": close" << std::endl;
                break;
            }
        }
        usleep(1000);
    }

    std::cout << "Comm " << client_id_ << ": leave " << __PRETTY_FUNCTION__ << std::endl;

    std::unique_lock<std::mutex> lock(mtx_finish_);
    is_finished_ = true;
}

} //end ns
