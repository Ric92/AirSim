#pragma once

#include "CoreMinimal.h"

#include "PawnSimApi.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/ornithopter/Ornithopter.hpp"
#include "vehicles/ornithopter/OrnithopterParams.hpp"
#include <future>

class OrnithopterPawnSimApi : public PawnSimApi
{
public:
    typedef msr::airlib::real_T real_T;
    typedef msr::airlib::Utils Utils;
    typedef msr::airlib::Ornithopter Ornithopter;
    typedef msr::airlib::StateReporter StateReporter;
    typedef msr::airlib::UpdatableObject UpdatableObject;
    typedef msr::airlib::Pose Pose;


public:
    virtual void initialize() override;

    virtual ~OrnithopterPawnSimApi() = default;

    //VehicleSimApiBase interface
    //implements game interface to update pawn
    OrnithopterPawnSimApi(const Params& params);
    virtual void updateRenderedState(float dt) override;
    virtual void updateRendering(float dt) override;

    //PhysicsBody interface
    //this just wrapped around Ornithopter physics body
    virtual void resetImplementation() override;
    virtual void update() override;
    virtual void reportState(StateReporter& reporter) override;
    virtual UpdatableObject* getPhysicsBody() override;

    virtual void setPose(const Pose& pose, bool ignore_collision) override;
    virtual void pawnTick(float dt) override;

    void collectSensors(ImuBase::Output& _imu, BarometerBase::Output& _baro, MagnetometerBase::Output& _magne, GpsBase::Output& _gps);

private:
    std::unique_ptr<msr::airlib::OrnithopterParams> vehicle_params_;

    std::unique_ptr<Ornithopter> phys_vehicle_;
    bool sensorInit_ = false;
    //show info on collision response from physics engine
    CollisionResponse collision_response;

    //when pose needs to set from non-physics thread, we set it as pending
    bool pending_pose_collisions_;
    enum class PendingPoseStatus
    {
        NonePending,
        RenderStatePending,
        RenderPending
    } pending_pose_status_;
    Pose pending_phys_pose_; //force new pose through API

    //reset must happen while World is locked so its async task initiated from API thread
    bool reset_pending_;
    bool did_reset_;
    std::packaged_task<void()> reset_task_;

    Pose last_phys_pose_; //for trace lines showing vehicle path
    std::vector<std::string> vehicle_api_messages_;
};
