/* --------------------------------------------------------------------------*
*                OpenSim:  testExponentialContact.cpp                        *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2025 Stanford University and the Authors                     *
* Author(s): F. C. Anderson                                                  *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */
#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/ForceReporter.h>

#include <OpenSim/Simulation/Model/ContactGeometrySet.h>
#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactMesh.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/Model/ElasticFoundationForce.h>
#include <OpenSim/Simulation/Model/HuntCrossleyForce.h>
#include <OpenSim/Simulation/Model/ExponentialContactForce.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/FreeJoint.h>
#include <OpenSim/Simulation/StatesTrajectory.h>
#include <OpenSim/Simulation/StatesTrajectoryReporter.h>
#include <OpenSim/Simulation/StatesDocument.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <OpenSim/Actuators/osimActuators.h>

#include "SimTKsimbody.h"
#include <catch2/catch_all.hpp>

using namespace SimTK;
using namespace OpenSim;
using std::cout;
using std::endl;
using std::string;
using std::vector;

//=============================================================================
// Class ExponentialContactTester provides a scope and framework for
// evaluating and testing the ExponentialContactForce class. Using this testing
// class gets a lot of variables out of the global scope and allows for more
// structured memory management.
class ExponentialContactTester
{
public:
    // Contact choices
    enum ContactChoice {
        Exp = 0
    };

    // Initial condition choices
    enum InitialConditionsChoice{
        Static = 0,
        Bounce,
        Slide,
        Spin,
        SpinSlide,
        SpinTop,
        Tumble
    };

    // Constructor
    ExponentialContactTester() {
        corner[0] = Vec3( hs, -hs,  hs);
        corner[1] = Vec3( hs, -hs, -hs);
        corner[2] = Vec3(-hs, -hs, -hs);
        corner[3] = Vec3(-hs, -hs,  hs);
        corner[4] = Vec3( hs,  hs,  hs);
        corner[5] = Vec3( hs,  hs, -hs);
        corner[6] = Vec3(-hs,  hs, -hs);
        corner[7] = Vec3(-hs,  hs,  hs);
    };

    // Destructor
    ~ExponentialContactTester() {
        if (model) {
            //model->disownAllComponents();  // See note just below.
            delete model;
        }
        // If the model still owns its components, the following deletes should
        // not be called. On the other hand, if all components are disowned,
        // they must be individually deleted.
        /*
        if (blockEC) delete blockEC;
        if (blockHC) delete blockHC;
        for (int i = 0; i < n; i++) {
        if (sprEC[i]) delete sprEC[i];
        if (sprHC[i]) delete sprHC[i];
        if (geomHC[i]) delete geomHC[i];
        }
        */
    }

    // Utility
    void buildModel();
    OpenSim::Body* addBlock(const std::string& suffix);
    void addExponentialContact(OpenSim::Body* body);
    void setInitialConditions(SimTK::State& state,
        const SimTK::MobilizedBody& body, double dz);
    void printDiscreteVariableAbstractValue(const string& pathName,
        const AbstractValue& value) const;

    //-------------------------------------------------------------------------
    // Member variables
    //-------------------------------------------------------------------------
    // Simulation related
    double integ_accuracy{1.0e-5};
    double dt_max{0.03};
    SimTK::Vec3 gravity{SimTK::Vec3(0, -9.8065, 0)};
    double mass{10.0};
    double tf{5.0};
    const static int n{8};
    const double hs{0.10}; // half of a side of a cube (like a radius)
    Vec3 corner[n];
    // Command line options and their defaults
    ContactChoice whichContact{Exp};
    InitialConditionsChoice whichInit{Slide};
    bool noDamp{false};
    // Model and parts
    Model* model{nullptr};
    OpenSim::Body* blockEC{nullptr};
    OpenSim::ExponentialContactForce* sprEC[n]{nullptr};
    // Expected simulation results for running the Simulation test case.
    // Depending on the platform (e.g,. Windows, Linux, MacOS), the actual
    // values may be less than or equal to the expected values.
    static const int expectedTrys{1817};
    static const int expectedSteps{1247};

    // Reporters
    StatesTrajectoryReporter* statesReporter{nullptr};

}; // End class ExponentialContactTester declarations


//-----------------------------------------------------------------------------
// Method implementations for ExponentialContactTester
//-----------------------------------------------------------------------------
void
ExponentialContactTester::
buildModel()
{
    // Create the bodies
    model = new Model();
    model->setGravity(gravity);
    model->setName("BouncingBlock_ExponentialContact");
    blockEC = addBlock("EC");
    addExponentialContact(blockEC);

    // Reporters
    // StatesTrajectory
    statesReporter = new StatesTrajectoryReporter();
    statesReporter->setName("states_reporter");
    statesReporter->set_report_time_interval(0.1);
    model->addComponent(statesReporter);
}

OpenSim::Body*
ExponentialContactTester::
addBlock(const std::string& suffix)
{
    Ground& ground = model->updGround();

    // Body
    std::string name = "block" + suffix;
    OpenSim::Body* block = new OpenSim::Body();
    block->setName(name);
    block->set_mass(mass);
    block->set_mass_center(Vec3(0));
    block->setInertia(Inertia(1.0));

    // Joint
    name = "free" + suffix;
    FreeJoint *free = new
        FreeJoint(name, ground, Vec3(0), Vec3(0), *block, Vec3(0), Vec3(0));
    model->addBody(block);
    model->addJoint(free);

    return block;
}

void
ExponentialContactTester::
addExponentialContact(OpenSim::Body* block)
{
    Ground& ground = model->updGround();

    // Contact Plane Transform
    Real angle = convertDegreesToRadians(90.0);
    Rotation floorRot(-angle, XAxis);
    Vec3 floorOrigin(0., -0.004, 0.);
    Transform floorXForm(floorRot, floorOrigin);

    // Contact Parameters
    SimTK::ExponentialSpringParameters params;  // yields default params
    if (noDamp) {
        params.setNormalViscosity(0.0);
        params.setFrictionViscosity(0.0);
        params.setInitialMuStatic(0.0);
    }


    // Place a spring at each of the 8 corners
    std::string name = "";
    for (int i = 0; i < n; ++i) {
        name = "Exp" + std::to_string(i);
        Station* station = new Station(*block, corner[i]);
        station->setName(fmt::format("corner_{}", i));
        block->addComponent(station);
        sprEC[i] = new OpenSim::ExponentialContactForce(floorXForm,
            *station, params);
        sprEC[i]->setName(name);
        model->addForce(sprEC[i]);
    }
}

// dz allows for the body to be shifted along the z axis. This is useful for
// displacing the body spring points upward above the floor.
void
ExponentialContactTester::
setInitialConditions(SimTK::State& state, const SimTK::MobilizedBody& body,
    double dz)
{
    SimTK::Rotation R;
    SimTK::Vec3 pos(0.0, 0.0, dz);
    SimTK::Vec3 vel(0.0);
    SimTK::Vec3 angvel(0.0);

    switch (whichInit) {
    case Static:
        pos[0] = 0.0;
        pos[1] = hs;
        body.setQToFitTranslation(state, pos);
        break;
    case Bounce:
        pos[0] = 0.0;
        pos[1] = 1.0;
        body.setQToFitTranslation(state, pos);
        break;
    case Slide:
        pos[0] = 2.0;
        pos[1] = 2.0 * hs;
        vel[0] = -4.0;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        break;
    case Spin:
        pos[0] = 0.0;
        pos[1] = hs;
        vel[0] = 0.0;
        angvel[1] = 8.0 * SimTK::Pi;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        body.setUToFitAngularVelocity(state, angvel);
        break;
    case SpinSlide:
        pos[0] = 1.0;
        pos[1] = hs;
        vel[0] = -3.0;
        angvel[1] = 4.0 * SimTK::Pi;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        body.setUToFitAngularVelocity(state, angvel);
        break;
    case SpinTop:
        R.setRotationFromAngleAboutNonUnitVector(
            convertDegreesToRadians(54.74), Vec3(1, 0, 1));
        pos[0] = 0.0;
        pos[1] = 2.0*hs;
        vel[0] = 0.0;
        angvel[1] = 1.5 * SimTK::Pi;
        body.setQToFitRotation(state, R);
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        body.setUToFitAngularVelocity(state, angvel);
        break;
    case Tumble:
        pos[0] = -1.5;
        pos[1] = 2.0 * hs;
        vel[0] = -1.0;
        angvel[2] = 2.0 * SimTK::Pi;
        body.setQToFitTranslation(state, pos);
        body.setUToFitLinearVelocity(state, vel);
        body.setUToFitAngularVelocity(state, angvel);
        break;
    default:
        cout << "Unrecognized set of initial conditions!" << endl;
    }
}

void
ExponentialContactTester::
printDiscreteVariableAbstractValue(const string& pathName,
    const AbstractValue& value) const
{
    cout << pathName << " = type{" << value.getTypeName() << "} ";
    cout << value << " = ";

    // Switch depending on the type
    if (SimTK::Value<double>::isA(value)) {
        double x = SimTK::Value<double>::downcast(value);
        cout << x << endl;
    } else if (SimTK::Value<Vec3>::isA(value)) {
        Vec3 x = SimTK::Value<Vec3>::downcast(value);
        cout << x << endl;
    }
}


//=============================================================================
// Test Cases
//=============================================================================

// Execute a simulation of a bouncing block with exponential contact forces,
// recording states along the way and serializing the states upon completion.
TEST_CASE("Simulation")
{
    // Create the tester, build the tester model, and initialize the state.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());
    CHECK_NOTHROW(tester.model->buildSystem());
    SimTK::State& state = tester.model->initializeState();


    // Set initial conditions
    double dz = 1.0;
    tester.whichInit = ExponentialContactTester::SpinSlide;
    tester.setInitialConditions(state, tester.blockEC->getMobilizedBody(), dz);

    // Reset the elastic anchor point for each ExponentialContactForce instance
    // Resetting the anchor points moves the anchor point directly below the
    // body station of the block. So, initially, there will be no elastic
    // friction force acting on the block.
    ExponentialContactForce::resetAnchorPoints(*tester.model, state);

    // Integrate
    Manager manager(*tester.model);
    manager.getIntegrator().setMaximumStepSize(tester.dt_max);
    manager.setIntegratorAccuracy(tester.integ_accuracy);
    state.setTime(0.0);
    manager.initialize(state);
    manager.setWriteToStorage(true);
    std::clock_t startTime = std::clock();
    state = manager.integrate(tester.tf);
    auto runTime = 1.e3 * (std::clock() - startTime) / CLOCKS_PER_SEC;

    // Output
    int trys = manager.getIntegrator().getNumStepsAttempted();
    int steps = manager.getIntegrator().getNumStepsTaken();
    //printConditions();
    cout << "           trys:  " << trys << endl;
    cout << "          steps:  " << steps << endl;
    cout << "       cpu time:  " << runTime << " msec" << endl;

    // Check that the number of trys and steps match the expected values.
    CHECK(trys <= ExponentialContactTester::expectedTrys);
    CHECK(steps <= ExponentialContactTester::expectedSteps);

    // Serialize the states
    int precision = 10;
    const StatesTrajectory& statesTraj = tester.statesReporter->getStates();
    StatesDocument statesDocSe = statesTraj.exportToStatesDocument(
        *tester.model, "sliding simulation", precision);
    SimTK::String filename01 = "BouncingBlock_ExponentialContact_1.ostates";
    CHECK_NOTHROW( statesDocSe.serialize(filename01) );

    // Deserialize the states
    StatesDocument statesDocDe(filename01);
    Array_<State> statesTrajDeserialized;
    CHECK_NOTHROW(
        statesDocDe.deserialize(*tester.model, statesTrajDeserialized));

    // Check that the number of State objects in the trajectories matches
    CHECK(statesTraj.getSize() == statesTrajDeserialized.size());

    // Copy the model.
    Model modelCopy(*tester.model);
    SimTK::String modelFileName = "BouncingBlock_ExponentialContact_Copy.osim";
    CHECK_NOTHROW(modelCopy.print(modelFileName));
}


// Test that the model can be serialized and deserialized.
TEST_CASE("Model Serialization")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());
    CHECK_NOTHROW(tester.model->buildSystem());

    // Serialize the model with default properties and spring parameters.
    std::string fileName = "BouncingBlock_ExponentialContact_Default.osim";
    CHECK_NOTHROW(tester.model->print(fileName));

    // Deserialize the model
    Model modelCopy(fileName);

    // Check that the properties and spring parameters match the original.
    const ForceSet& fSet0 = tester.model->getForceSet();
    const ForceSet& fSet1 = modelCopy.getForceSet();
    int n = fSet1.getSize();
    for (int i = 0; i < n; ++i) {
        try {
            ExponentialContactForce& ec0 =
                dynamic_cast<ExponentialContactForce&>(fSet0.get(i));
            ExponentialContactForce& ec1 =
                dynamic_cast<ExponentialContactForce&>(fSet1.get(i));

            CHECK(ec1.getContactPlaneTransform() ==
                    ec0.getContactPlaneTransform());

            const Station& s0 = ec0.getConnectee<Station>("station");
            const Station& s1 = ec1.getConnectee<Station>("station");
            CHECK(s0.getParentFrame().getAbsolutePathString() ==
                  s1.getParentFrame().getAbsolutePathString());
            CHECK(s0.get_location() == s1.get_location());

            CHECK(ec1.getParameters() == ec0.getParameters());

        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContactForce.
        }
    }

    // Alter the default spring parameters to test re-serialization.
    double delta = 0.123;
    Vec3 shape;
    ExponentialSpringParameters p = tester.sprEC[0]->getParameters();
    p.getShapeParameters(shape[0], shape[1], shape[2]);
    p.setShapeParameters(
        shape[0] + delta, shape[1] + delta, shape[2] + delta);
    p.setNormalViscosity(p.getNormalViscosity() + delta);
    p.setMaxNormalForce(p.getMaxNormalForce() + delta);
    p.setFrictionElasticity(p.getFrictionElasticity() + delta);
    p.setFrictionViscosity(p.getFrictionViscosity() + delta);
    p.setSettleVelocity(p.getSettleVelocity() + delta);
    p.setInitialMuStatic(p.getInitialMuStatic() + delta);
    p.setInitialMuKinetic(p.getInitialMuKinetic() + delta);
    n = fSet0.getSize();
    for (int i = 0; i < n; ++i) {
        try {
            ExponentialContactForce& ec =
                dynamic_cast<ExponentialContactForce&>(fSet0.get(i));
            ec.setParameters(p);

        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContactForce.
        }
    }

    // Serialize the model with altered properties and spring parameters.
    fileName = "BouncingBlock_ExponentialContact_Altered.osim";
    CHECK_NOTHROW(tester.model->print(fileName));

    // Deserialize the model
    Model modelCopy2(fileName);

    // Check that the re-deserialized model has the correct spring parameters.
    const ForceSet& fSet2 = modelCopy2.getForceSet();
    n = fSet2.getSize();
    for (int i = 0; i < n; ++i) {
        try {
            ExponentialContactForce& ec0 =
                dynamic_cast<ExponentialContactForce&>(fSet0.get(i));
            ExponentialContactForce& ec2 =
                dynamic_cast<ExponentialContactForce&>(fSet2.get(i));

            CHECK(ec2.getContactPlaneTransform() ==
                ec0.getContactPlaneTransform());

            const Station& s0 = ec0.getConnectee<Station>("station");
            const Station& s2 = ec2.getConnectee<Station>("station");
            CHECK(s0.getParentFrame().getAbsolutePathString() ==
                  s2.getParentFrame().getAbsolutePathString());
            CHECK(s0.get_location() == s2.get_location());

            CHECK(ec2.getParameters() == ec0.getParameters());

        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContactForce.
        }
    }

}

// Test copy construction, move construction, and the copy assignment
// operator before and after the SimTK System has been built.
//
// Before the SimTK::System is built, none of the ExponentialContactForce
// instances wraps an instatiated SimTK::ExponentialSpringForce. That is,
// ExponentialContactForce::_spr = nullptr. In this case, it makes sense to
// alter the contact_plane_transform, body station, and contact parameters.
//
// After the SimTK::System is built, each ExponentialContactForce instance
// wraps an instantiated ExponentialContactForce. That is,
// ExponentialContactForce::_spr != nullptr. In this case, neither the
// contact_plane_transform nor body station can be altered, but the
// contact_parameters CAN be still be altered.
//
// Copy construction, copy assignment, and move construction should enforce
// this logic.
TEST_CASE("Construction")
{

}


// Test that the discrete states of an ExponentialContactForce instance can be
// set and retrieved properly.
TEST_CASE("Discrete State Accessors")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());
    CHECK_NOTHROW(tester.model->buildSystem());

    // Realize the model and get the state.
    SimTK::State& state = tester.model->initSystem();

    // Check current properties/parameters of all springs are equal.
    for (int i = 0; i < tester.n; i++) {
       CHECK_NOTHROW( tester.sprEC[i]->assertPropertiesAndParametersEqual() );
    }

    // Pick a contact instance to manipulate.
    ExponentialContactForce& spr = *tester.sprEC[0];

    // Declarations
    double deltaDbl = 0.1;
    Vec3 deltaVec3(deltaDbl);
    double vali{NaN}, valf{NaN};
    Vec3 veci{NaN}, vecf{NaN};

    // Static Friction Coefficient
    vali = spr.getMuStatic(state);
    spr.setMuStatic(state, vali + deltaDbl);
    valf = spr.getMuStatic(state);
    CHECK(valf == vali + deltaDbl);

    // Kinetic Friction Coefficient
    vali = spr.getMuKinetic(state);
    spr.setMuKinetic(state, vali + deltaDbl);
    valf = spr.getMuKinetic(state);
    CHECK(valf == vali + deltaDbl);

    // Sliding
    // Note that the "sliding" state is an auto-update discrete state and so
    // it is not settable. It is only retrievable. The "sliding" state is
    // updated by the ExponentialContactForce instance during simulation after
    // each successful integration step.
    // There are bounds (0 <= sliding <= 1.0) that can be checked, however.
    // In addition, retrieving the sldiing state also requites the state to be
    // realized to Stage::Dynamics or higher, so we can check that an
    // exception is thrown if the state is not realized to that stage and
    // a "get" is attempted.
    state.setTime(0.0); // Resets the system to Stage::Time
    CHECK_THROWS(vali = spr.getSliding(state));
    tester.model->getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
    vali = spr.getSliding(state);
    CHECK(vali >= 0.0);
    CHECK(vali <= 1.0);

    // Elastic Anchor Point
    // Like sliding, the "anchor" state is an auto-update discrete state and
    // so it is not settable in a simple way. See comments for "sliding" above.
    // The position of an anchar point can, however, be set to correspond
    // exactly to the position of the body station of its spring.
    // Note - this is also a good check for 1) resetAnchorPoint(),
    // 2) getAnchorPointPosition(), and 3) getStationPosition().
    state.setTime(0.0); // Resets the system to Stage::Time
    CHECK_THROWS(veci = spr.getAnchorPointPosition(state));
    spr.resetAnchorPoint(state);
    tester.model->getMultibodySystem().realize(state, SimTK::Stage::Dynamics);
    veci = spr.getAnchorPointPosition(state);
    vecf = spr.getStationPosition(state);
    CHECK(vecf[0] == veci[0]);
    // vecf[1] won't be equal because the anchor point is always on the contact
    // plane while the body station is usually located above the contact plane.
    CHECK(vecf[2] == veci[2]);
}


// Test that the contact plane property of an ExponentialContactForce instance
// can be set and retrieved properly. This property, along with the properties
// encapsulated in the ExponentialContactForce::Parameters class (see below),
// are is needed to construct an ExponentialContactForce instance. The
// ExponentialContactForce::Parameters are tested below in the test case
// "Spring Parameters".
TEST_CASE("Contact Plane Transform")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());
    CHECK_NOTHROW(tester.model->buildSystem());

    // Default Contact Plane Transform
    Real angle = convertDegreesToRadians(90.0);
    Rotation floorRot(-angle, XAxis);
    Vec3 floorOrigin(0., -0.004, 0.);
    Transform floorXForm(floorRot, floorOrigin);

    // Check the accessor.
    SimTK::Transform xformf = tester.sprEC[0]->getContactPlaneTransform();
    CHECK(xformf.p() == floorXForm.p());
    CHECK(xformf.R() == floorXForm.R());
}


// Test that the underlying spring parameters of an ExponentialContactForce instance
// can be set and retrieved properly. In addition, verify that the
// corresponding OpenSim properties and the underlying parameters that belong
// to the SimTK::ExponentialSpringForce instance are kept consistent with
// one another.
TEST_CASE("Spring Parameters")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());
    CHECK_NOTHROW(tester.model->buildSystem());

    // Check current properties/parameters of all springs are equal.
    for (int i = 0; i < tester.n; i++) {
        CHECK_NOTHROW( tester.sprEC[i]->assertPropertiesAndParametersEqual() );
    }

    // Pick a contact instance to manipulate.
    ExponentialContactForce& spr = *tester.sprEC[0];

    // Save the starting parameters.
    // Note that pi is not a reference. The underlying parameters of spr can
    // be changed without affecting pi.
    const SimTK::ExponentialSpringParameters pi = spr.getParameters();

    // Create a copy of the parameters that will be systematically modified.
    SimTK::ExponentialSpringParameters pf = pi;

    // Test equality of the Paremeter instances.
    CHECK(pf == pi);

    // Exponential Shape
    double delta = 0.1;
    Vec3 di, df;
    pf.getShapeParameters(di[0], di[1], di[2]);
    // d[0]
    pf.setShapeParameters(di[0] + delta, di[1], di[2]);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0] + delta);
    CHECK(df[1] == di[1]);
    CHECK(df[2] == di[2]);
    spr.setParameters(pi);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    // d[1]
    pf.setShapeParameters(di[0], di[1] + delta, di[2]);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0]);
    CHECK(df[1] == di[1] + delta);
    CHECK(df[2] == di[2]);
    spr.setParameters(pi);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    // d[2]
    pf.setShapeParameters(di[0], di[1], di[2] + delta);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0]);
    CHECK(df[1] == di[1]);
    CHECK(df[2] == di[2] + delta);
    spr.setParameters(pi);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    // all at once
    pf.setShapeParameters(di[0] + delta, di[1] + delta, di[2] + delta);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0] + delta);
    CHECK(df[1] == di[1] + delta);
    CHECK(df[2] == di[2] + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Normal Viscosity
    double vali, valf;
    vali = pi.getNormalViscosity();
    pf.setNormalViscosity(vali + delta);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    valf = pf.getNormalViscosity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Max Normal Force
    vali = pi.getMaxNormalForce();
    pf.setMaxNormalForce(vali + delta);
    valf = pf.getMaxNormalForce();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Settle Velocity
    vali = pi.getSettleVelocity();
    pf.setSettleVelocity(vali + delta);
    valf = pf.getSettleVelocity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Friction Elasticity
    vali = pi.getFrictionElasticity();
    pf.setFrictionElasticity(vali + delta);
    valf = pf.getFrictionElasticity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Friction Viscosity
    vali = pi.getFrictionViscosity();
    pf.setFrictionViscosity(vali + delta);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Settle Velocity
    vali = pi.getSettleVelocity();
    pf.setSettleVelocity(vali + delta);
    valf = pf.getSettleVelocity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Initial Static Coefficient of Friction
    vali = pi.getInitialMuStatic();
    pf.setInitialMuStatic(vali + delta);
    valf = pf.getInitialMuStatic();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Initial Kinetic Coefficient of Friction
    vali = pi.getInitialMuKinetic();
    pf.setInitialMuKinetic(vali - delta);
    valf = pf.getInitialMuKinetic();
    CHECK(valf == vali - delta);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Make a change to mus that should also change muk
    double musi = pi.getInitialMuStatic();
    double muki = pi.getInitialMuKinetic();
    pf.setInitialMuStatic(muki - delta);  // should enforce muk <= mus
    double musf = pf.getInitialMuStatic();
    double mukf = pf.getInitialMuKinetic();
    CHECK(musf == muki - delta);
    CHECK(mukf == musf);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );

    // Make a change to musk that should also change mus
    musi = pi.getInitialMuStatic();
    muki = pi.getInitialMuKinetic();
    pf.setInitialMuKinetic(musi + delta);  // should enforce mus >= musk
    musf = pf.getInitialMuStatic();
    mukf = pf.getInitialMuKinetic();
    CHECK(mukf == musi + delta);
    CHECK(musf == mukf);
    spr.setParameters(pf);
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
    spr.setParameters(pi); // now back to original
    CHECK_NOTHROW( spr.assertPropertiesAndParametersEqual() );
}



/* This is not currently used in the test suite, but it is a good example of
// how to set/get discrete variables at a low level.

// The only types that are handled are double and Vec3 at this point.
// The significant changes in how Discrete Variables are handled are:
//      1. Values are now not assumed to be doubles but are AbstractValues.
//      2. Discrete variables allocated external to OpenSim are permitted.
//      3. Discrete variables may be accessed via the Component API by
//      specifying the path (e.g., path = "/forceset/Exp0/anchor").
void
ExponentialContactTester::
testDiscreteVariables(State& state, const ForceSet& fSet) {

    // Get the names
    OpenSim::Array<std::string> names = fSet.getDiscreteVariableNames();

    // Loop
    int n = names.size();
    for (int i = 0; i < n; ++i) {

        // Print values for debugging purposes.
        AbstractValue& valAbstract =
            fSet.updDiscreteVariableAbstractValue(state, names[i]);
        //printDiscreteVariableAbstractValue(names[i], valAbstract);

        // Declarations
        double tol = 1.0e-6;
        double deltaDbl = 0.1;
        Vec3 deltaVec3(deltaDbl);
        double valStartDbl{NaN};
        Vec3 valStartVec3{NaN};

        // Perturb
        if (SimTK::Value<double>::isA(valAbstract)) {
            SimTK::Value<double>& valDbl =
                SimTK::Value<double>::updDowncast(valAbstract);
            valStartDbl = valDbl;
            valDbl = valStartDbl + deltaDbl;
        } else if (SimTK::Value<Vec3>::isA(valAbstract)) {
            SimTK::Value<Vec3>& valVec3 =
                SimTK::Value<Vec3>::updDowncast(valAbstract);
            valStartVec3 = valVec3.get();
            valVec3 = valStartVec3 + deltaVec3;
        }
        //printDiscreteVariableAbstractValue(names[i], valAbstract);

        // Check that the value changed correctly
        if (SimTK::Value<double>::isA(valAbstract)) {
            SimTK::Value<double>& valDbl =
                SimTK::Value<double>::updDowncast(valAbstract);
            ASSERT_EQUAL(valDbl.get(), valStartDbl + deltaDbl, tol);
        } else if (SimTK::Value<Vec3>::isA(valAbstract)) {
            SimTK::Value<Vec3>& valVec3 =
                SimTK::Value<Vec3>::updDowncast(valAbstract);
            ASSERT_EQUAL(valVec3.get(), valStartVec3 + deltaVec3, tol);
        }

        // Restore the starting value
        if (SimTK::Value<double>::isA(valAbstract)) {
            SimTK::Value<double>& valDbl =
                SimTK::Value<double>::updDowncast(valAbstract);
            valDbl = valStartDbl;
        } else if (SimTK::Value<Vec3>::isA(valAbstract)) {
            SimTK::Value<Vec3>& valVec3 =
                SimTK::Value<Vec3>::updDowncast(valAbstract);
            valVec3 = valStartVec3;
        }
        //printDiscreteVariableAbstractValue(names[i], valAbstract);

        // Check that the value was correctly restored
        if (SimTK::Value<double>::isA(valAbstract)) {
            SimTK::Value<double>& valDbl =
                SimTK::Value<double>::updDowncast(valAbstract);
            ASSERT_EQUAL(valDbl.get(), valStartDbl, tol);
        } else if (SimTK::Value<Vec3>::isA(valAbstract)) {
            SimTK::Value<Vec3>& valVec3 =
                SimTK::Value<Vec3>::updDowncast(valAbstract);
            ASSERT_EQUAL(valVec3.get(), valStartVec3, tol);
        }

    }

}
*/