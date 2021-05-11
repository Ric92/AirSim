// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_ornithopterphysicsbody_hpp
#define msr_airlib_ornithopterphysicsbody_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/VehicleApiBase.hpp"
#include "api/VehicleSimApiBase.hpp"
#include <vector>
#include "physics/PhysicsBody.hpp"
#include "OrnithopterParams.hpp"

namespace msr { namespace airlib {

class OrnithopterPhysicsBody : public PhysicsBody
{
public:
    OrnithopterPhysicsBody(OrnithopterParams* params,
        Kinematics* kinematics, Environment* environment) : params_(params)
    {
            setName("OrnithopterPhysicsBody");
            initialize(kinematics, environment);
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
            //reset rotors, kinematics and environment
            PhysicsBody::resetImplementation();
    }

    virtual void update() override
    {
            //update forces on vertices that we will use next
            PhysicsBody::update();
    }

    virtual void reportState(StateReporter& reporter) override
    {
            //call base
            PhysicsBody::reportState(reporter);
    }
    //*** End: UpdatableState implementation ***//


    //Fast Physics engine calls this method to set next kinematics
    virtual void updateKinematics(const Kinematics::State& kinematics) override
    {
            PhysicsBody::updateKinematics(kinematics);
    }

    //External Physics engine calls this method to keep physics bodies updated and move rotors
    virtual void updateKinematics() override
    {
            PhysicsBody::updateKinematics();
    }

    virtual ~OrnithopterPhysicsBody() = default;

private: //methods
    void initialize(Kinematics* kinematics, Environment* environment)
    {
            PhysicsBody::initialize(params_->getParams().mass, params_->getParams().inertia, kinematics, environment);
    }
private:
    OrnithopterParams* params_;
    std::unique_ptr<Environment> environment_;
};

}} //namespace
#endif