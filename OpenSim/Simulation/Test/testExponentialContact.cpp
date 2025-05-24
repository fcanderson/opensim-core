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
    void testParameters();
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
private:

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

    // Check the Component API for discrete states.
    int n = fSet.getSize();
    try {
        testDiscreteVariables(state, fSet);
    } catch (const std::exception& e) {
        cout << e.what() << endl;
    }

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


//-----------------------------------------------------------------------------
// TESTING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
ExponentialContactTester::
test()
{
    testParameters();
    testModelSerialization();
    //testStatesDocument();
}
//_____________________________________________________________________________
void
ExponentialContactTester::
testParameters()
{
    // Check current properties/parameters for all springs
    for (int i = 0; i < n; i++) {
        sprEC[i]->assertPropertiesAndParametersEqual();
    }

    // Pick just one contact instance to manipulate.
    ExponentialContact& spr = *sprEC[0];

    // Save the starting parameters
    SimTK::ExponentialSpringParameters p0 = spr.getParameters();

    // Change the properties systematically
    SimTK::ExponentialSpringParameters p1 = p0;

    // Shape
    double delta = 0.1;
    Vec3 d;
    p1.getShapeParameters(d[0], d[1], d[2]);
    p1.setShapeParameters(d[0] + delta, d[1], d[2]);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();
    p1.setShapeParameters(d[0], d[1] + delta, d[2]);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();
    p1.setShapeParameters(d[0], d[1], d[2] + delta);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();

    // Normal Viscosity
    double value;
    value = p1.getNormalViscosity();
    p1.setNormalViscosity(value + delta);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();

    // Friction Elasticity
    value = p1.getFrictionElasticity();
    p1.setFrictionElasticity(value + delta);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();

    // Friction Viscosity
    value = p1.getFrictionViscosity();
    p1.setFrictionViscosity(value + delta);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();

    // Settle Velocity
    value = p1.getSettleVelocity();
    p1.setSettleVelocity(value + delta);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();

    // Initial Coefficients of Friction
    double mus = p1.getInitialMuStatic();
    double muk = p1.getInitialMuKinetic();
    p1.setInitialMuStatic(muk - delta);  // Changes muk also
    mus = p1.getInitialMuStatic();
    muk = p1.getInitialMuKinetic();
    SimTK_TEST_EQ(mus, muk);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();
    p1.setInitialMuKinetic(mus + delta); // Changes mus also
    SimTK_TEST_EQ(mus, muk);
    spr.setParameters(p1);
    spr.assertPropertiesAndParametersEqual();

    // Return to the starting parameters
    spr.setParameters(p0);
    spr.assertPropertiesAndParametersEqual();
}
//_____________________________________________________________________________
void
ExponentialContactTester::
testModelSerialization() {
    // Serialize the current model
    std::string fileName = "BouncingBlock_ExponentialContact_Serialized.osim";
    model->print(fileName);
    ExponentialSpringParameters p = sprEC[0]->getParameters();

    // Deserialize the model
    Model modelCopy(fileName);

    // Get the 0th contact instance
    ExponentialContact* spr = sprEC[0];
    ExponentialContact* sprCopy = dynamic_cast<ExponentialContact*>(
        &modelCopy.updForceSet().get(spr->getName()));
    ExponentialSpringParameters pCopy = sprCopy->getParameters();

    // Parameters should be equal
    SimTK_ASSERT_ALWAYS(pCopy == p,
        "Deserialized parameters are not equal to original parameters.");
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


TEST_CASE("A")
{

}

TEST_CASE("B")
{

}