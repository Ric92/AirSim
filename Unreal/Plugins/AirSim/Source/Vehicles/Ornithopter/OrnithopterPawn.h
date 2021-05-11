#pragma once

#include "CoreMinimal.h"
#include "Engine.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/RotatingMovementComponent.h"

#include "PIPCamera.h"
#include "common/common_utils/Signal.hpp"
#include "common/common_utils/UniqueValueMap.hpp"
#include "vehicles/ornithopter/OrnithopterParams.hpp"
#include <memory>
#include "OrnithopterPawn.generated.h"
namespace msr
{
namespace airlib
{
    class OrnithopterParams;
}
};
UCLASS()
class AIRSIM_API AOrnithopterPawn : public APawn
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float m;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters ")
    float Sw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float St;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Bw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Bt;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Lw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Lt;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Hw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Ht;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Iy;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float h0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float K;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float frequency;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Cd0w;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Cd0t;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Epsilon;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float Li;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float scaleU;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter parameters")
    float scaleW;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float x0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float z0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float u0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float du0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float w0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float dw0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float theta0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float q0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Initial condition")
    float h0flap;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control variable")
    float alfaw0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Control variable")
    float delta0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time parameters")
    float dt;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Time parameters")
    float adt;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter state")
    bool flapping;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter state")
    float wingAngle;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter state")
    float tailAngle;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug variable")
    float fakeDt;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Debug variable")
    bool fakeTime;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ornithopter non-dimensional parameters")
    bool non_dimensional = false;

    UPROPERTY(VisibleDefaultsOnly, Category = Mesh)
    USkeletalMeshComponent* OrnithopterMesh;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comunication")
    int fakeStateSubscriber;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comunication")
    int controlActionSubscriber;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comunication")
    int headMovementSubscriber;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comunication")
    int statePublisherPort;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Comunication")
    int imagePublisherPort;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orientation")
    float roll;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orientation")
    float yaw;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Orientation")
    float pitch;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Head movement")
    FString imageIp;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Head movement")
    bool OrientationCommunication;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    bool stereo;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    bool imagePublish;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    int compression;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    bool detector = true;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    float fx;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    float fy;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    float cx;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Image")
    float cy;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual servoing")
    bool activate_visual_servoing;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual servoing")
    float kp;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual servoing")
    float ki;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual servoing")
    float kd;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual servoing")
    float maxSaturation;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Visual servoing")
    float minSaturation;

    AOrnithopterPawn();
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaSeconds) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                           FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit) override;

    void getFlapping(bool& _flapping);
    void getWingsAndTailAngles(float& _wing, float& _tail);
    void setFlapping(bool& _flapping);
    void setWingsAndTailAngles(float& _wing, float& _tail);
    void setWingMovement(double wingMovement);

    msr::airlib::OrnithopterParams* getParams();

    //interface
    void initializeForBeginPlay();
    const common_utils::UniqueValueMap<std::string, APIPCamera*> getCameras() const;

private: //variables
    //Unreal components
    UPROPERTY()
    APIPCamera* camera_front_left_;
    UPROPERTY()
    APIPCamera* camera_front_right_;
    UPROPERTY()
    APIPCamera* camera_front_center_;
    UPROPERTY()
    APIPCamera* camera_back_center_;
    UPROPERTY()
    APIPCamera* camera_bottom_center_;


    // visual servoing
    std::chrono::high_resolution_clock::time_point start_;
    std::chrono::high_resolution_clock::time_point lastDetection_;
    bool imageInitialized_ = false;
    std::ofstream vsLogFile_;
    // utility
    std::chrono::high_resolution_clock::time_point initialTime_;
    std::chrono::high_resolution_clock::time_point lastTick_;
    std::chrono::high_resolution_clock::time_point lastImage_;
    float inputTail = 90;
};
