// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_Ornithopter_hpp
#define msr_airlib_Ornithopter_hpp

#include "AirBlueprintLib.h"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/PhysicsBody.hpp"
#include "vehicles/ornithopter/OrnithopterParams.hpp"
#include <chrono>
#include <thread>
#include <vector>

namespace msr
{
namespace airlib
{

    class Ornithopter : public PhysicsBody
    {
    public:
        Ornithopter(OrnithopterParams* params, Kinematics* kinematics, Environment* environment)
            : params_(params)
        {
            initialize(kinematics, environment);
        }

        OrnithopterParams* getParams()
        {
            return params_;
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            //reset rotors, kinematics and environment
            PhysicsBody::reset();

            //reset sensors last after their ground truth has been reset
            resetSensors();
        }

        virtual void update() override
        {
            //update forces on vertices that we will use next
            PhysicsBody::update();

            //Note that controller gets updated after kinematics gets updated in updateKinematics
            //otherwise sensors will have values from previous cycle causing lags which will appear
            //as crazy jerks whenever commands like velocity is issued
        }
        virtual void reportState(StateReporter& reporter) override
        {
            //call base
            PhysicsBody::reportState(reporter);

            reportSensors(*params_, reporter);

        }
        //*** End: UpdatableState implementation ***//

        //Physics engine calls this method to set next kinematics
        virtual void updateKinematics(const Kinematics::State& kinematics) override
        {
            msr::airlib::Vector3r linearTwist(0.5, 0, 0);
            //msr::airlib::Vector3r angularTwist(2000,2000,1000);
            // msr::airlib::Twist initialVelocity(linearTwist,angularTwist);
            //msr::airlib::Kinematics::State fakeState;
            auto fakeState = kinematics;
            //fakeState.twist.linear(0) = 10.0f;
            fakeState.twist.angular(1) = 0.0f;
            msr::airlib::Vector3r linearAccel(10.f, 0.0f, 0.0f);
            fakeState.accelerations.linear = VectorMath::transformToWorldFrame(linearAccel, fakeState.pose.orientation);

            // fakeState.twist = linearTwist;
            auto now = std::chrono::high_resolution_clock::now();
            auto dur = now - initialTime;
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(dur).count();

            //UAirBlueprintLib::LogMessageString("Ornithopter xyz velocity: ", Utils::stringf("%f, %f, %f",
            //	kinematics.twist.linear(0), kinematics.twist.linear(1), kinematics.twist.linear(2)), LogDebugLevel::Informational, 1);

            if (secs < 1.0f) {
                PhysicsBody::updateKinematics(fakeState);
                //UAirBlueprintLib::LogMessageString("Ornithopter faked velocity: ", Utils::stringf("%f",
                //	kinematics.twist.linear(0)), LogDebugLevel::Informational, 0.1);
            }
            else {
                PhysicsBody::updateKinematics(kinematics);
                //UAirBlueprintLib::LogMessageString("Ornithopter faked velocity: ", "OFF", LogDebugLevel::Informational, 0.1);
            }

            //updateSensors(*params_, getKinematics(), getEnvironment());

        }

        virtual real_T getRestitution() const override
        {
            return params_->getParams().restitution;
        }
        virtual real_T getFriction() const override
        {
            return params_->getParams().friction;
        }

        //sensor getter
        const SensorCollection& getSensors() const
        {
            //return params_->getSensors();
        }

        virtual ~Ornithopter() = default;

    private: //methods
        void initialize(Kinematics* kinematics, Environment* environment)
        {
            PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);

            initSensors(*params_, getKinematics(), getEnvironment());

            //PhysicsBody::updateKinematics(kinematics);
            initialTime = std::chrono::high_resolution_clock::now();
        }

        void reportSensors(OrnithopterParams& params, StateReporter& reporter)
        {
            //params.getSensors().reportState(reporter);
        }

       void updateSensors(OrnithopterParams& params, const Kinematics::State& state, const Environment& environment)
        {
           // unused(state);
           //unused(environment);
           //params.getSensors().update();
        }

        void initSensors(OrnithopterParams& params, const Kinematics::State& state, const Environment& environment)
        {
            //params.getSensors().initialize(&state, &environment);
        }

        void resetSensors()
        {
            //params_->getSensors().reset();
        }

    private: //fields
        OrnithopterParams* params_;

        //let us be the owner of rotors object
        std::unique_ptr<Environment> environment_;
        std::chrono::high_resolution_clock::time_point initialTime;
    };

}
} //namespace
#endif
