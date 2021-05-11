#include "OrnithopterPawn.h"
#include "AirBlueprintLib.h"
#include "Components/InputComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
//#include "ConstructorHelpers.h"
#include "Engine/TextureRenderTarget2D.h"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include <chrono>

AOrnithopterPawn::AOrnithopterPawn()
{
    //static ConstructorHelpers::FObjectFinder<USkeletalMesh> OrnithopterMesh(TEXT("SkeletalMesh'/Game/Ornithopter/ComplexOrnithopter/Ornithopter_eagle.Ornithopter_eagle'"));
    //OrnithopterMesh->SetSkeletalMesh(MeshContainer.Object);
}

void AOrnithopterPawn::BeginPlay()
{
    Super::BeginPlay();
 /*   ornithopterParams_ = new OrnithopterParametersErnesto();
    ornithopterParams_->m = m;
    ornithopterParams_->Sw = Sw;
    ornithopterParams_->St = St;
    ornithopterParams_->Bw = Bw;
    ornithopterParams_->Bt = Bt;
    ornithopterParams_->Lw = Lw;
    ornithopterParams_->Lt = Lt;
    ornithopterParams_->Hw = Hw;
    ornithopterParams_->Ht = Ht;
    ornithopterParams_->Iy = Iy;
    ornithopterParams_->h0 = h0;
    ornithopterParams_->K = K;
    ornithopterParams_->frequency = frequency;
    ornithopterParams_->Cd0w = Cd0w;
    ornithopterParams_->Cd0t = Cd0t;
    ornithopterParams_->scaleU = scaleU;
    ornithopterParams_->scaleW = scaleW;

    ornithopterParams_->x0 = x0;
    ornithopterParams_->z0 = z0;
    ornithopterParams_->u0 = u0;
    ornithopterParams_->du0 = du0;
    ornithopterParams_->w0 = w0;
    ornithopterParams_->dw0 = dw0;
    ornithopterParams_->theta0 = theta0;
    ornithopterParams_->q0;

    ornithopterParams_->alfaw0 = alfaw0;
    ornithopterParams_->delta0 = delta0;

    ornithopterParams_->dt = dt;
    ornithopterParams_->adt = adt;
    ornithopterParams_->Li = Li;
    ornithopterParams_->Epsilon = Epsilon;

    ornithopterParams_->frequency = frequency;*/
}

void AOrnithopterPawn::initializeForBeginPlay()
{
    //get references of existing camera
    camera_front_right_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontRightCamera")))->GetChildActor());
    camera_front_left_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontLeftCamera")))->GetChildActor());
    camera_front_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("FrontCenterCamera")))->GetChildActor());
    camera_back_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BackCenterCamera")))->GetChildActor());
    camera_bottom_center_ = Cast<APIPCamera>(
        (UAirBlueprintLib::GetActorComponent<UChildActorComponent>(this, TEXT("BottomCenterCamera")))->GetChildActor());

    //APIPCamera::ImageType image_type = ImageCaptureBase::ImageType::Scene;
    //auto projMatrix = camera_front_right_->getProjectionMatrix(image_type);
    //UAirBlueprintLib::LogMessageString("Projection matrix: ", Utils::stringf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, ", projMatrix.matrix[0][0], projMatrix.matrix[0][1], projMatrix.matrix[0][2], projMatrix.matrix[0][3],
    //	projMatrix.matrix[1][0], projMatrix.matrix[1][1], projMatrix.matrix[1][2], projMatrix.matrix[1][3],
    //	projMatrix.matrix[2][0], projMatrix.matrix[2][1], projMatrix.matrix[2][2], projMatrix.matrix[2][3],
    //	projMatrix.matrix[3][0], projMatrix.matrix[3][1], projMatrix.matrix[3][2], projMatrix.matrix[3][3]), LogDebugLevel::Informational, 100000000);

    // get copy of physics engine
    //std::unique_ptr<PhysicsEngineBase> physics_engine;
    //msr::airlib::Settings fast_phys_settings;
    //if (msr::airlib::Settings::singleton().getChild("OrnithopterPhysicsEngine", fast_phys_settings)) {
    //	auto ornit = physics_engine.get();
    //	auto ornithopterPhysi = static_cast<OrnithopterPhysicsEngine*>(ornit);
    //	// set physics engine parameters from blueprint
    //	ornithopterPhysi->setParameters(ornithopterParams_);
    //}

}

void AOrnithopterPawn::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now - initialTime_).count();
    auto tickTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTick_).count();
    //UAirBlueprintLib::LogMessageString("Pawn tick (Milliseconds Hz): ", Utils::stringf("%f, %f", (float)tickTime, 1000 / ((float)tickTime)), LogDebugLevel::Informational, 10);
    lastTick_ = std::chrono::high_resolution_clock::now();

    auto imageTickTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastImage_).count();
}

void AOrnithopterPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{

    camera_front_right_ = nullptr;
    camera_front_left_ = nullptr;
    camera_front_center_ = nullptr;
    camera_back_center_ = nullptr;
    camera_bottom_center_ = nullptr;

    Super::EndPlay(EndPlayReason);

}

const common_utils::UniqueValueMap<std::string, APIPCamera*> AOrnithopterPawn::getCameras() const
{
    common_utils::UniqueValueMap<std::string, APIPCamera*> cameras;
    cameras.insert_or_assign("front_center", camera_front_center_);
    cameras.insert_or_assign("front_right", camera_front_right_);
    cameras.insert_or_assign("front_left", camera_front_left_);
    cameras.insert_or_assign("bottom_center", camera_bottom_center_);
    cameras.insert_or_assign("back_center", camera_back_center_);

    cameras.insert_or_assign("0", camera_front_center_);
    cameras.insert_or_assign("1", camera_front_right_);
    cameras.insert_or_assign("2", camera_front_left_);
    cameras.insert_or_assign("3", camera_bottom_center_);
    cameras.insert_or_assign("4", camera_back_center_);

    cameras.insert_or_assign("", camera_front_center_);
    cameras.insert_or_assign("fpv", camera_front_center_);

    return cameras;
}

void AOrnithopterPawn::NotifyHit(class UPrimitiveComponent* MyComp, class AActor* Other, class UPrimitiveComponent* OtherComp, bool bSelfMoved, FVector HitLocation,
                                 FVector HitNormal, FVector NormalImpulse, const FHitResult& Hit)
{
    //pawn_events_.getCollisionSignal().emit(MyComp, Other, OtherComp, bSelfMoved, HitLocation, HitNormal, NormalImpulse, Hit);
}

msr::airlib::OrnithopterParams* AOrnithopterPawn::getParams()
{
    msr::airlib::OrnithopterParams* orni_params = new msr::airlib::OrnithopterParams();
    return orni_params;
}

void AOrnithopterPawn::getWingsAndTailAngles(float& _wing, float& _tail)
{
    _wing = wingAngle;
    _tail = tailAngle;
}

void AOrnithopterPawn::getFlapping(bool& _flapping)
{
    _flapping = flapping;
}

void AOrnithopterPawn::setWingsAndTailAngles(float& _wing, float& _tail)
{
    wingAngle = _wing;
    tailAngle = _tail;
}

void AOrnithopterPawn::setFlapping(bool& _flapping)
{
    flapping = _flapping;
}

void AOrnithopterPawn::setWingMovement(double wingMovement)
{
    wingAngle = wingMovement;
};