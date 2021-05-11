// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "SimModeWorldOrnithopter.h"
//#include "ConstructorHelpers.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Logging/MessageLog.h"

#include "AirBlueprintLib.h"
#include "OrnithopterPawnSimApi.h"
#include "Runtime/Engine/Public/ImageUtils.h"
#include "common/ClockFactory.hpp"
#include "physics/OrnithopterPhysicsEngine.hpp"
#include "physics/PhysicsBody.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"
#include "vehicles/ornithopter/OrnithopterParams.hpp"
#include <memory>

void ASimModeWorldOrnithopter::BeginPlay()
{
    Super::BeginPlay();

    //let base class setup physics world
    initializeForPlay();

    //// initialize physics engine with ornithopter pawn parameters
    //TArray<AActor*> pawns;
    //getExistingVehiclePawns(pawns);
    //if (pawns[0] != nullptr) {
    //    pawn_ = static_cast<AOrnithopterPawn*>(pawns[0]);
    //    static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->setParameters(static_cast<AOrnithopterPawn*>(pawns[0])->getErnestoParams());
    //    VehicleSimApiBase* vehicle_sim_api = getApiProvider()->getVehicleSimApi("");
    //    static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->setVehicleApi(static_cast<OrnithopterPawnSimApi*>(vehicle_sim_api));
    //}
    //// static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->updateTailAngle(90);
    //// static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->updateWingsAngle(90);
    //static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->updateFlapping(false);

}

void ASimModeWorldOrnithopter::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (pawn_ != nullptr) {
        //float tailAngle;
        //float wingAngle;
        //bool flap;
        //float wingMovement;
        // pawn_->getWingsAndTailAngles(wingAngle, tailAngle);
        //pawn_->getFlapping(flap);
        //static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->updateFlapping(flap);
        //static_cast<OrnithopterPhysicsEngine*>(physics_engine_)->getWingMovement(wingMovement);
        //pawn_->setWingMovement(wingMovement);
        /*auto cams = pawn_->getCameras();
		FRotator orientation(roll_, pitch_, yaw_);
		cams.at("back_center")->setCameraOrientation(orientation);*/
    }

    UAirBlueprintLib::LogMessageString("Calback counter: ", Utils::stringf("%i", counter_), LogDebugLevel::Informational, 10);

    // for (auto& api : getApiProvider()->getVehicleSimApis())
    //api->getImage("back_center", ImageCaptureBase::ImageType::Scene);
}

void ASimModeWorldOrnithopter::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    //stop physics thread before we dismantle
    stopAsyncUpdator();

    Super::EndPlay(EndPlayReason);
}

void ASimModeWorldOrnithopter::setupClockSpeed()
{
    typedef msr::airlib::ClockFactory ClockFactory;

    float clock_speed = getSettings().clock_speed;

    //setup clock in ClockFactory
    std::string clock_type = getSettings().clock_type;

    if (clock_type == "ScalableClock") {
        //scalable clock returns interval same as wall clock but multiplied by a scale factor
        ClockFactory::get(std::make_shared<msr::airlib::ScalableClock>(clock_speed == 1 ? 1 : 1 / clock_speed));
    }
    else if (clock_type == "SteppableClock") {
        //steppable clock returns interval that is a constant number irrespective of wall clock
        //we can either multiply this fixed interval by scale factor to speed up/down the clock
        //but that would cause vehicles like quadrotors to become unstable
        //so alternative we use here is instead to scale control loop frequency. The downside is that
        //depending on compute power available, we will max out control loop frequency and therefore can no longer
        //get increase in clock speed

        //Approach 1: scale clock period, no longer used now due to quadrotor instability
        //ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
        //static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));

        //Approach 2: scale control loop frequency if clock is speeded up
        if (clock_speed >= 1) {
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9))); //no clock_speed multiplier

            setPhysicsLoopPeriod(getPhysicsLoopPeriod() / static_cast<long long>(clock_speed));
        }
        else {
            //for slowing down, this don't generate instability
            ClockFactory::get(std::make_shared<msr::airlib::SteppableClock>(
                static_cast<msr::airlib::TTimeDelta>(getPhysicsLoopPeriod() * 1E-9 * clock_speed)));
        }
    }
    else
        throw std::invalid_argument(common_utils::Utils::stringf(
            "clock_type %s is not recognized", clock_type.c_str()));
}

//-------------------------------- overrides -----------------------------------------------//

std::unique_ptr<msr::airlib::ApiServerBase> ASimModeWorldOrnithopter::createApiServer() const
{
#ifdef AIRLIB_NO_RPC
    return ASimModeBase::createApiServer();
#else
    return std::unique_ptr<msr::airlib::ApiServerBase>(new msr::airlib::MultirotorRpcLibServer(
        getApiProvider(), getSettings().api_server_address));
#endif
}

void ASimModeWorldOrnithopter::getExistingVehiclePawns(TArray<AActor*>& pawns) const
{
    UAirBlueprintLib::FindAllActor<TVehiclePawn>(this, pawns);
}

bool ASimModeWorldOrnithopter::isVehicleTypeSupported(const std::string& vehicle_type) const
{
    return ((vehicle_type == AirSimSettings::kVehicleTypeSimpleFlight) ||
            (vehicle_type == AirSimSettings::kVehicleTypePX4) ||
            (vehicle_type == AirSimSettings::kVehicleTypeArduCopterSolo));
}

std::string ASimModeWorldOrnithopter::getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const
{
    //decide which derived BP to use
    std::string pawn_path = vehicle_setting.pawn_path;
    if (pawn_path == "")
        pawn_path = "DefaultQuadrotor";

    return pawn_path;
}

const common_utils::UniqueValueMap<std::string, APIPCamera*> ASimModeWorldOrnithopter::getVehiclePawnCameras(
    APawn* pawn) const
{
    return (static_cast<const TVehiclePawn*>(pawn))->getCameras();
}
void ASimModeWorldOrnithopter::initializeVehiclePawn(APawn* pawn)
{
    static_cast<TVehiclePawn*>(pawn)->initializeForBeginPlay();
}
std::unique_ptr<PawnSimApi> ASimModeWorldOrnithopter::createVehicleSimApi(
    const PawnSimApi::Params& pawn_sim_api_params) const
{
    auto vehicle_sim_api = std::unique_ptr<PawnSimApi>(new OrnithopterPawnSimApi(pawn_sim_api_params));
    vehicle_sim_api->initialize();
    //For multirotors the vehicle_sim_api are in PhysicsWOrld container and then get reseted when world gets reseted
    //vehicle_sim_api->reset();
    return vehicle_sim_api;
}