#pragma once

#include "CoreMinimal.h"

#include "OrnithopterPawn.h"
#include "SimMode/SimModeWorldBase.h"
#include "api/VehicleSimApiBase.hpp"
#include "common/Common.hpp"
#include "Runtime/Engine/Classes/Engine/TextureRenderTarget2D.h"
#include <mutex>

#include "SimModeWorldOrnithopter.generated.h"

UCLASS()
class AIRSIM_API ASimModeWorldOrnithopter : public ASimModeWorldBase
{
    GENERATED_BODY()

public:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

protected: //overrides
    virtual void setupClockSpeed() override;
    virtual std::unique_ptr<msr::airlib::ApiServerBase> createApiServer() const override;
    virtual void getExistingVehiclePawns(TArray<AActor*>& pawns) const override;
    virtual bool isVehicleTypeSupported(const std::string& vehicle_type) const override;
    virtual std::string getVehiclePawnPathName(const AirSimSettings::VehicleSetting& vehicle_setting) const override;
    virtual const common_utils::UniqueValueMap<std::string, APIPCamera*> getVehiclePawnCameras(APawn* pawn) const override;
    virtual void initializeVehiclePawn(APawn* pawn) override;
    virtual std::unique_ptr<PawnSimApi> createVehicleSimApi(
        const PawnSimApi::Params& pawn_sim_api_params) const override;

private:
    typedef AOrnithopterPawn TVehiclePawn;
    TVehiclePawn* pawn_;
    int counter_ = 0;
    float roll_ = 0;
    float pitch_ = 0;
    float yaw_ = 0;
};
