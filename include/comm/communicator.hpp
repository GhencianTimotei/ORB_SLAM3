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

#pragma once

// C++
#include <atomic>
#include <mutex>

// COVINS
#include <covins/covins_base/communicator_base.hpp>

namespace ORB_SLAM3 {

class Atlas;
class KeyFrame;

class Communicator : public covins::CommunicatorBase, public std::enable_shared_from_this<Communicator> {
public:
    // Server correction of this agent's map frame A, from the newest KF the server sent back.
    struct Correction {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Matrix4d T_W_A        = Eigen::Matrix4d::Identity();
        unsigned long   kf_id        = 0;      // ORB id of the KF it was computed from
        bool            shared_frame = false;  // W = merged server map (patched server), else this agent's first KF
        int             anchor_client = 0;     // agent whose first KF pins W
        unsigned long   seq          = 0;
    };

public:
    Communicator(std::string server_ip, std::string port, Atlas*);

    // False if the connection could not be established; Run() must not be started then.
    auto IsConnected() const                                                            ->bool { return connected_; }

    virtual auto ConnectToServer(const char *node, std::string port)                    ->int override;

    // main function
    virtual auto Run()                                                                  ->void;

    virtual auto PassKfToComm(KeyFrame* kf)                                             ->void {
        std::unique_lock<std::mutex> lock(mtx_kf_queue_);
        kf_out_buffer_.push_back(kf);
    }

    // Ends all map access (waits for an in-flight conversion or correction) and Run(); call before clearing the map.
    auto Stop()                                                                         ->void;
    // Unblocks the receive thread so the object can be destroyed.
    auto Disconnect()                                                                   ->void;
    auto QueuedKeyframes()                                                              ->size_t;

    // Returns false until a usable keyframe pose has been received from the server.
    auto GetCorrection(Correction &out) const                                           ->bool;

protected:

    // data handling
    virtual auto ProcessAdditional()                                                    ->void;
    virtual auto ProcessKeyframeMessages()                                              ->void;
    virtual auto ProcessLandmarkMessages()                                              ->void;
    virtual auto ProcessNewKeyframes()                                                  ->void;
    virtual auto ProcessNewLandmarks()                                                  ->void;

    virtual auto ProcessKfBuffer()                                                      ->void;

    // Infrastructure
    Atlas*                  map_                                                                = nullptr;

    bool connected_ = false;
    std::list<KeyFrame*> kf_out_buffer_;
    std::mutex              mtx_kf_queue_;
    std::mutex              mtx_convert_;       // held while the Comm thread reads KFs/MPs
    bool                    stopped_ = false;   // guarded by mtx_convert_
    long                    kf_id_offset_ = -1;   // ORB id of the first sent KF; the server sees it as 0
    KeyFrame*               last_sent_kf_ = nullptr;

    auto UpdateCorrection(const covins::MsgKeyframe &msg)                               ->void;

    mutable std::mutex      mtx_correction_;
    Correction              correction_;
    KeyFrame*               corr_kf_ = nullptr;                                     // KF the last correction came from
    Eigen::Matrix4d         T_W_corr_kf_ = Eigen::Matrix4d::Identity();             // its pose in the server frame
};

} //end ns
