/* -------------------------------------------------------------------------- *
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
#include <OpenSim/Simulation/Model/ExponentialContact.h>
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
/** Class ExponentialContactTester provides a scope and framework for
evaluating and testing the ExponentialContact class. Using a testing class, as
opposed to just a main() and C-style procedures, gets a lot of variables out
of the global scope and allows for more structured memory management. */
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

    // Model Creation
    void buildModel();
    OpenSim::Body* addBlock(const std::string& suffix);
    void addExponentialContact(OpenSim::Body* body);

    // Test stuff not covered elsewhere.
    void test();
    void checkParametersAndPropertiesEqual(const ExponentialContact& spr) const;
    void testModelSerialization();
    void printDiscreteVariableAbstractValue(const string& pathName,
        const AbstractValue& value) const;
    void testDiscreteVariables(State& state, const ForceSet& fSet);

    // Simulation
    void setInitialConditions(SimTK::State& state,
        const SimTK::MobilizedBody& body, double dz);
    void simulate();

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
    Model* model{NULL};
    OpenSim::Body* blockEC{NULL};
    OpenSim::ExponentialContact* sprEC[n]{nullptr};

    // Reporters
    StatesTrajectoryReporter* statesReporter{nullptr};

}; // End class ExponentialContactTester declarations



//_____________________________________________________________________________
// Build the model
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

    // Build the System
    model->buildSystem();
}
//______________________________________________________________________________
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
//______________________________________________________________________________
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
        sprEC[i] = new OpenSim::ExponentialContact(floorXForm,
            block->getName(), corner[i], params);
        sprEC[i]->setName(name);
        model->addForce(sprEC[i]);
    }
}
//_____________________________________________________________________________
// dz allows for the body to be shifted along the z axis. This is useful for
// displacing the Exp and Hunt models.
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


//_____________________________________________________________________________
void
ExponentialContactTester::
simulate()
{
    // Initialize the state
    // Note that model components have already been connected and the
    // SimTK::System has already been built.
    // See TestExponentialContact::buildModel().
    SimTK::State& state = model->initializeState();

    // Set initial conditions
    double dz = 1.0;
    if (blockEC != NULL)
        setInitialConditions(state, blockEC->getMobilizedBody(), dz);

    // Reset the elastic anchor point for each ExponentialContact instance
    ForceSet& fSet = model->updForceSet();
    ExponentialContact::resetAnchorPoints(fSet, state);

    // Integrate
    Manager manager(*model);
    manager.getIntegrator().setMaximumStepSize(dt_max);
    manager.setIntegratorAccuracy(integ_accuracy);
    state.setTime(0.0);
    manager.initialize(state);
    manager.setWriteToStorage(true);
    std::clock_t startTime = std::clock();
    state = manager.integrate(tf);
    auto runTime = 1.e3 * (std::clock() - startTime) / CLOCKS_PER_SEC;

    // Output
    int trys = manager.getIntegrator().getNumStepsAttempted();
    int steps = manager.getIntegrator().getNumStepsTaken();
    //printConditions();
    cout << "           trys:  " << trys << endl;
    cout << "          steps:  " << steps << endl;
    cout << "       cpu time:  " << runTime << " msec" << endl;

    // Save the model to file
    //model->print("C:\\Users\\fcand\\Documents\\block.osim");

    // Serialize the states
    int precision = 10;
    const StatesTrajectory& statesTraj = statesReporter->getStates();
    StatesDocument statesDocSe =
        statesTraj.exportToStatesDocument(*model);
    SimTK::String filename01 =
        "C:/Users/fcand/Documents/GitHub/Work/Testing/OpenSim/test01.ostates";
    statesDocSe.serialize(filename01);

    // Deserialize the states
    StatesDocument statesDocDe(filename01);
    Array_<State> traj;
    statesDocDe.deserialize(*model, traj);

    // Reserialize the states
    String note = "Should be identical to test01.ostates.";
    SimTK::String filename02 =
        "C:/Users/fcand/Documents/GitHub/Work/Testing/OpenSim/test02.ostates";
    StatesDocument statesDocRe(*model, traj, note, precision);
    statesDocRe.serialize(filename02);
}

//_____________________________________________________________________________
void
ExponentialContactTester::
checkParametersAndPropertiesEqual(const ExponentialContact& spr) const {
    // Get the OpenSim properties
    const ExponentialContact::Parameters& a = spr.get_contact_parameters();
    const SimTK::ExponentialSpringParameters& b = spr.getParameters();

    const SimTK::Vec3& vecA = a.get_exponential_shape_parameters();
    SimTK::Vec3 vecB;
    b.getShapeParameters(vecB[0], vecB[1], vecB[2]);
    CHECK(vecA[0] == vecB[0]);
    CHECK(vecA[1] == vecB[1]);
    CHECK(vecA[2] == vecB[2]);

    double valA, valB;
    valA = a.get_normal_viscosity();
    valB = b.getNormalViscosity();
    CHECK(valA == valB);

    valA = a.get_friction_elasticity();
    valB = b.getFrictionElasticity();
    CHECK(valA == valB);

    valA = a.get_friction_viscosity();
    valB = b.getFrictionViscosity();
    CHECK(valA == valB);

    valA = a.get_settle_velocity();
    valB = b.getSettleVelocity();
    CHECK(valA == valB);

    valA = a.get_initial_mu_static();
    valB = b.getInitialMuStatic();
    CHECK(valA == valB);

    valA = a.get_initial_mu_kinetic();
    valB = b.getInitialMuKinetic();
    CHECK(valA == valB);
}

//_____________________________________________________________________________
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

//_____________________________________________________________________________
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

//_____________________________________________________________________________
// Test that the model can be serialized and deserialized.
TEST_CASE("Model Serialization")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());

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
            ExponentialContact& ec0 =
                dynamic_cast<ExponentialContact&>(fSet0.get(i));
            ExponentialContact& ec1 =
                dynamic_cast<ExponentialContact&>(fSet1.get(i));

            CHECK(ec1.getContactPlaneTransform() ==
                    ec0.getContactPlaneTransform());

            CHECK(ec1.getBodyName() == ec0.getBodyName());
            CHECK(ec1.getBodyStation() == ec0.getBodyStation());

            ExponentialSpringParameters p = ec1.getParameters();
            CHECK(ec1.getParameters() == ec0.getParameters());

        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContact.
        }
    }

    // Alter the default spring parameters to test reserialization.
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
            ExponentialContact& ec =
                dynamic_cast<ExponentialContact&>(fSet0.get(i));
            ec.setParameters(p);

        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContact.
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
            ExponentialContact& ec0 =
                dynamic_cast<ExponentialContact&>(fSet0.get(i));
            ExponentialContact& ec2 =
                dynamic_cast<ExponentialContact&>(fSet2.get(i));

            CHECK(ec2.getContactPlaneTransform() ==
                ec0.getContactPlaneTransform());

            CHECK(ec2.getBodyName() == ec0.getBodyName());
            CHECK(ec2.getBodyStation() == ec0.getBodyStation());

            ExponentialSpringParameters p2 = ec2.getParameters();
            CHECK(ec2.getParameters() == ec0.getParameters());

        } catch (const std::bad_cast&) {
            // Nothing should happen here. Execution is just skipping any
            // OpenSim::Force that is not an ExponentialContact.
        }
    }

}

//_____________________________________________________________________________
// Test that the properties of an ExponentialContact instance can be set
// and retrieved properly. These properties are members ExponentialContact.
// The spring parameters are encapsulated in class
// ExponentialContact::Parameters. The API for those parameters are tested
// in the test case "Spring Parameters" below.
TEST_CASE("Property Accessors")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());

    // Contact Plane
    const SimTK::Transform xformi =
        tester.sprEC[0]->getContactPlaneTransform();
    SimTK::Rotation R;
    R.setRotationFromAngleAboutX(1.234);
    SimTK::Transform xformp;
    xformp.set(R, Vec3(0.2,0.2,0.2));
    tester.sprEC[0]->setContactPlaneTransform(xformp);
    SimTK::Transform xformf = tester.sprEC[0]->getContactPlaneTransform();
    CHECK(xformf.p() == xformp.p());
    CHECK(xformf.R() == xformp.R());

    // Body Name
    const std::string bodyNamei = tester.sprEC[0]->getBodyName();
    tester.sprEC[0]->setBodyName(bodyNamei + "new");
    const std::string bodyNamef = tester.sprEC[0]->getBodyName();
    CHECK(bodyNamef == bodyNamei + "new");

    // Body Station
    Vec3 delta(0.1, 0.2, 0.3);
    const SimTK::Vec3 stationi = tester.sprEC[0]->getBodyStation();
    tester.sprEC[0]->setBodyStation(stationi + delta);
    const SimTK::Vec3 stationf = tester.sprEC[0]->getBodyStation();
    CHECK(stationf[0] == stationi[0] + delta[0]);
    CHECK(stationf[1] == stationi[1] + delta[1]);
    CHECK(stationf[2] == stationi[2] + delta[2]);
}

//_____________________________________________________________________________
// Test that the discrete states of an ExponentialContact instance can be set
// and retrieved properly.
TEST_CASE("Discrete State Accessors")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());

    // Realize the model and get the state.
    SimTK::State& state = tester.model->initSystem();

    // Check current properties/parameters of all springs are equal.
    for (int i = 0; i < tester.n; i++) {
        tester.checkParametersAndPropertiesEqual(*tester.sprEC[i]);
    }

    // Pick a contact instance to manipulate.
    ExponentialContact& spr = *tester.sprEC[0];

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
    // updated by the ExponentialContact instance during simulation after each
    // successful integration step.
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
    // y won't be equal because the anchor point is on the contact plane
    CHECK(vecf[2] == veci[2]);
}

//_____________________________________________________________________________
// Test that the underlying spring parameters of an ExponentialContact instance
// can be set and retrieved properly. In addition, verify that the
// corresponding OpenSim properties and the underlying parameters that belong
// to the SimTK::ExponentialSpringForce instance are kept consistent with
// one another.
TEST_CASE("Spring Parameters")
{
    // Create the tester and build the tester model.
    ExponentialContactTester tester;
    CHECK_NOTHROW(tester.buildModel());

    // Check current properties/parameters of all springs are equal.
    for (int i = 0; i < tester.n; i++) {
        tester.checkParametersAndPropertiesEqual(*tester.sprEC[i]);
    }

    // Pick a contact instance to manipulate.
    ExponentialContact& spr = *tester.sprEC[0];

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
    tester.checkParametersAndPropertiesEqual(spr);
    // d[1]
    pf.setShapeParameters(di[0], di[1] + delta, di[2]);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0]);
    CHECK(df[1] == di[1] + delta);
    CHECK(df[2] == di[2]);
    spr.setParameters(pi);
    tester.checkParametersAndPropertiesEqual(spr);
    // d[2]
    pf.setShapeParameters(di[0], di[1], di[2] + delta);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0]);
    CHECK(df[1] == di[1]);
    CHECK(df[2] == di[2] + delta);
    spr.setParameters(pi);
    tester.checkParametersAndPropertiesEqual(spr);
    // all at once
    pf.setShapeParameters(di[0] + delta, di[1] + delta, di[2] + delta);
    pf.getShapeParameters(df[0], df[1], df[2]);
    CHECK(df[0] == di[0] + delta);
    CHECK(df[1] == di[1] + delta);
    CHECK(df[2] == di[2] + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Normal Viscosity
    double vali, valf;
    vali = pi.getNormalViscosity();
    pf.setNormalViscosity(vali + delta);
    tester.checkParametersAndPropertiesEqual(spr);
    valf = pf.getNormalViscosity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Max Normal Force
    vali = pi.getMaxNormalForce();
    pf.setMaxNormalForce(vali + delta);
    valf = pf.getMaxNormalForce();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Settle Velocity
    vali = pi.getSettleVelocity();
    pf.setSettleVelocity(vali + delta);
    valf = pf.getSettleVelocity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Friction Elasticity
    vali = pi.getFrictionElasticity();
    pf.setFrictionElasticity(vali + delta);
    valf = pf.getFrictionElasticity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Friction Viscosity
    vali = pi.getFrictionViscosity();
    pf.setFrictionViscosity(vali + delta);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Settle Velocity
    vali = pi.getSettleVelocity();
    pf.setSettleVelocity(vali + delta);
    valf = pf.getSettleVelocity();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Initial Static Coefficient of Friction
    vali = pi.getInitialMuStatic();
    pf.setInitialMuStatic(vali + delta);
    valf = pf.getInitialMuStatic();
    CHECK(valf == vali + delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Initial Kinetic Coefficient of Friction
    vali = pi.getInitialMuKinetic();
    pf.setInitialMuKinetic(vali - delta);
    valf = pf.getInitialMuKinetic();
    CHECK(valf == vali - delta);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Make a change to mus that should also change muk
    double musi = pi.getInitialMuStatic();
    double muki = pi.getInitialMuKinetic();
    pf.setInitialMuStatic(muki - delta);  // should enforce muk <= mus
    double musf = pf.getInitialMuStatic();
    double mukf = pf.getInitialMuKinetic();
    CHECK(musf == muki - delta);
    CHECK(mukf == musf);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);

    // Make a change to musk that should also change mus
    musi = pi.getInitialMuStatic();
    muki = pi.getInitialMuKinetic();
    pf.setInitialMuKinetic(musi + delta);  // should enforce mus >= musk
    musf = pf.getInitialMuStatic();
    mukf = pf.getInitialMuKinetic();
    CHECK(mukf == musi + delta);
    CHECK(musf == mukf);
    spr.setParameters(pf);
    tester.checkParametersAndPropertiesEqual(spr);
    spr.setParameters(pi); // now back to original
    tester.checkParametersAndPropertiesEqual(spr);
}