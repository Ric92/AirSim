#include "OrnithopterPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "UnrealSensors/UnrealSensorFactory.h"
#include "sensors/SensorBase.hpp"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include <exception>

#include "OrnithopterPawn.h"

using namespace msr::airlib;

OrnithopterPawnSimApi::OrnithopterPawnSimApi(const Params& params)
    : PawnSimApi(params)
{
    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    setPose(pose, false);
}

void OrnithopterPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //setup physics vehicle
    APawn* pawn = getPawn();
    AOrnithopterPawn* ornithopter_pawn = static_cast<AOrnithopterPawn*>(pawn);
    OrnithopterParams* aerodynamic_params = ornithopter_pawn->getParams();
    phys_vehicle_ = std::unique_ptr<Ornithopter>(new Ornithopter(aerodynamic_params, getKinematics(), getEnvironment()));

    //initialize private vars
    last_phys_pose_ = pending_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;

    // initialize sensors
    //SensorCollection& sensors = vehicle_params_.get()->getSensors();
    const msr::airlib::Kinematics::State* kinematics = getGroundTruthKinematics();
    const msr::airlib::Environment* environment = getGroundTruthEnvironment();
    /*sensors.initialize(kinematics, environment);
    sensors.reset();*/

    //UAirBlueprintLib::LogMessageString("Ornithopter pawn mass: ", Utils::stringf("%f", ornithopter_pawn->M), LogDebugLevel::Failure, 10000);
}

void OrnithopterPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void OrnithopterPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();
    phys_vehicle_->setCollisionInfo(collision_info);

    if (pending_pose_status_ == PendingPoseStatus::RenderStatePending) {
        phys_vehicle_->setPose(pending_phys_pose_);
        pending_pose_status_ = PendingPoseStatus::RenderPending;
    }

    last_phys_pose_ = phys_vehicle_->getPose();

    collision_response = phys_vehicle_->getCollisionResponseInfo();
}

void OrnithopterPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {

        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ == PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"),
                                 FString::FromInt(collision_response.collision_count_non_resting),
                                 LogDebugLevel::Informational);
  
}

// sensor updates and collect
void OrnithopterPawnSimApi::collectSensors(ImuBase::Output& _imu, BarometerBase::Output& _baro, MagnetometerBase::Output& _magne, GpsBase::Output& _gps)
{

    //SensorCollection& sensors = vehicle_params_.get()->getSensors();
    //sensors.update();

    //// imu
    //const SensorBase* imusensor = sensors.getByType(SensorBase::SensorType::Imu, 0);
    //if (imusensor) {
    //    _imu = static_cast<const ImuSimple*>(imusensor)->getOutput();
    //    UAirBlueprintLib::LogMessageString("IMU - > Linear accel x y z: ", Utils::stringf("%f,%f,%f", _imu.linear_acceleration[0], _imu.linear_acceleration[1], _imu.linear_acceleration[2]), LogDebugLevel::Failure, 10000);
    //}

    //// barometer
    //const SensorBase* barometersensor = sensors.getByType(SensorBase::SensorType::Barometer, 0);
    //if (barometersensor) {
    //    _baro = static_cast<const BarometerSimple*>(barometersensor)->getOutput();
    //    UAirBlueprintLib::LogMessageString("Barometer - > presure, altitude: ", Utils::stringf("%f,%f", _baro.pressure, _baro.altitude), LogDebugLevel::Failure, 10000);
    //}

    //// magnetometer
    //const SensorBase* magnetometersensor = sensors.getByType(SensorBase::SensorType::Magnetometer, 0);
    //if (magnetometersensor) {
    //    _magne = static_cast<const MagnetometerSimple*>(magnetometersensor)->getOutput();
    //    UAirBlueprintLib::LogMessageString("Magnetometer - > magnetic field x y z: ", Utils::stringf("%f,%f,%f", _magne.magnetic_field_body[0], _magne.magnetic_field_body[1], _magne.magnetic_field_body[2]), LogDebugLevel::Failure, 10000);
    //}

    //// GPS
    //const SensorBase* gpssensor = sensors.getByType(SensorBase::SensorType::Gps, 0);
    //if (gpssensor) {
    //    _gps = static_cast<const GpsSimple*>(gpssensor)->getOutput();
    //    UAirBlueprintLib::LogMessageString("GPS - > latitude, longitude, altitude: ", Utils::stringf("%f,%f,%f", _gps.gnss.geo_point.latitude, _gps.gnss.geo_point.longitude, _gps.gnss.geo_point.altitude), LogDebugLevel::Failure, 10000);
    //}
}

void OrnithopterPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    pending_phys_pose_ = pose;
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderStatePending;
}

//*** Start: UpdatableState implementation ***//
void OrnithopterPawnSimApi::resetImplementation()
{
    PawnSimApi::reset();
    phys_vehicle_->reset();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void OrnithopterPawnSimApi::update()
{
    //environment update for current position
    PawnSimApi::update();

    //update forces on vertices
    phys_vehicle_->update();

    //update to controller must be done after kinematics have been updated by physics engine
}

void OrnithopterPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    phys_vehicle_->reportState(reporter);
}

OrnithopterPawnSimApi::UpdatableObject* OrnithopterPawnSimApi::getPhysicsBody()
{
    return phys_vehicle_->getPhysicsBody();
}
//*** End: UpdatableState implementation ***//
