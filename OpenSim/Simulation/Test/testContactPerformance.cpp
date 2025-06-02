﻿/* -------------------------------------------------------------------------- *
 *               OpenSim:  testContactPerformance.cpp                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2022-2023 Stanford University and the Authors                *
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

using namespace SimTK;
using namespace std;
using namespace OpenSim;

//=============================================================================
/** Class ContactPerformanceTester provides a scope and framework for
evaluating the performance of the ExponentialContact class relative to the
HuntCrossleyContact. Using this class, as  opposed to just a main() and
C-style procedures, gets a lot of variables out of the global scope and allows
for more structured memory management. */
class ContactPerformanceTester
{
public:
    // Contact choices
    enum ContactChoice {
        Exp = 0,
        Hunt,
        Both
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
    ContactPerformanceTester() {
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
    ~ContactPerformanceTester() {
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

    // Command line parsing and usage
    int parseCommandLine(int argc, char** argv);
    void printUsage();
    void printConditions();

    // Model Creation
    void buildModel();
    OpenSim::Body* addBlock(const std::string& suffix);
    void addExponentialContact(OpenSim::Body* body);
    void addHuntCrossleyContact(OpenSim::Body* body);
    void setForceData(
            double t, const SimTK::Vec3& point, const SimTK::Vec3& force);
    void setForceDataHeader();

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
    bool applyFx{false};
    bool showVisuals{false};

    // Model and parts
    Model* model{NULL};
    OpenSim::Body* blockEC{NULL};
    OpenSim::Body* blockHC{NULL};
    OpenSim::ExponentialContact* sprEC[n]{nullptr};
    OpenSim::HuntCrossleyForce* sprHC[n]{nullptr};
    OpenSim::ContactGeometry* geomHC[n]{nullptr};
    Storage fxData;
    ExternalForce* fxEC{nullptr};
    ExternalForce* fxHC{nullptr};

    // Reporters
    StatesTrajectoryReporter* statesReporter{nullptr};

}; // End class ContactPerformanceTester declarations

//_____________________________________________________________________________
int
ContactPerformanceTester::
parseCommandLine(int argc, char** argv)
{
    std::string option;
    for (int i = 1; i < argc; ++i) {

        option = argv[i];

        // Contact choice
        if (option == "Exp")
            whichContact = Exp;
        else if (option == "Hunt")
            whichContact = Hunt;
        else if (option == "Both")
            whichContact = Both;

        // Initial condition choice
        else if (option == "Static")
            whichInit = Static;
        else if (option == "Bounce")
            whichInit = Bounce;
        else if (option == "Slide")
            whichInit = Slide;
        else if (option == "Spin")
            whichInit = Spin;
        else if (option == "SpinSlide")
            whichInit = SpinSlide;
        else if (option == "SpinTop")
            whichInit = SpinTop;
        else if (option == "Tumble")
            whichInit = Tumble;

        // Turn off all dissipative terms
        else if (option == "NoDamp")
            noDamp = true;

        // Apply a horizontal ramping force
        else if (option == "Fx")
            applyFx = true;

        // Show the visuals
        else if (option == "Vis")
            showVisuals = true;

        // Unrecognized
        else {
            printUsage();
            return -1;
        }
    }
    return 0;
}
//_____________________________________________________________________________
void
ContactPerformanceTester::
printUsage()
{
    cout << endl << "Usage:" << endl;
    cout << "$ testExponetialContact "
         << "[InitCond] [Contact] [NoDamp] [Fx] [Vis]" << endl;
    cout << "\tInitCond (choose one): Static Bounce Slide Spin ";
    cout << "SpinSlide SpinTop Tumble" << endl;
    cout << "\t Contact (choose one): Exp Hunt Both" << endl << endl;

    cout << "All arguments are optional. If no arguments are specified, ";
    cout << "a 'Slide' will" << endl;
    cout << "be simulated with one block that uses ";
    cout << "ExponentialContact instances," << endl;
    cout << "with typical damping settings, ";
    cout << "with no extnerally applied force, " << endl;
    cout << "and with no visuals." << endl << endl;

    cout << "Example:" << endl;
    cout << "To simulated 2 blocks (one with ExponentialContact and one";
    cout << " with Hunt-Crossley)" << endl;
    cout << "that bounce without energy dissipation and with Visuals, ";
    cout << "enter the following: " << endl << endl;

    cout << "$ testExponentialContact Bounce Both NoDamp Vis" << endl << endl;
}
//_____________________________________________________________________________
void
ContactPerformanceTester::
printConditions() {
    std::string modelDes;
    std::string contactDes;
    switch (whichContact) {
    case (Exp):
        modelDes = "One Block";
        contactDes = "Exponential";
        break;
    case (Hunt):
        modelDes = "One Block";
        contactDes = "Hunt Crossley";
        break;
    case (Both):
        modelDes = "Two Blocks";
        contactDes = "1 Block Exponential, 1 Block Hunt Crossley";
    }

    std::string motion;
    switch (whichInit) {
    case (Static): motion = "Sitting Still"; break;
    case (Bounce): motion = "Bouncing"; break;
    case (Slide): motion = "Sliding"; break;
    case (Spin): motion = "Spinning"; break;
    case (SpinSlide): motion = "Spinning & Sliding"; break;
    case (SpinTop): motion = "Spinnning like a Top"; break;
    case (Tumble): motion = "Tumbling Horizontally";
    }

    std::string appliedForce;
    if (applyFx)
        appliedForce = "Ramping Fx after 25 sec.";
    else
        appliedForce = "None";

    cout << endl << endl;
    cout << "          model:  " << modelDes << " (10 kg, 6 dof, 20x20x20 cm^3)" << endl;
    cout << "        contact:  " << contactDes << endl;
    cout << "         motion:  " << motion << endl;
    cout << "  applied force:  " << appliedForce << endl;
    cout << "      integ acc:  " << integ_accuracy << endl;
    cout << "  max step size:  " << dt_max << " sec" << endl;
    cout << "             tf:  " << tf << " sec" << endl;
}
//_____________________________________________________________________________
void
ContactPerformanceTester::
buildModel()
{
    // Create the bodies
    model = new Model();
    model->setGravity(gravity);
    model->setName("BouncingBlock_ExponentialContact");
    switch (whichContact) {
    case Exp:
        blockEC = addBlock("EC");
        addExponentialContact(blockEC);
        break;
    case Hunt:
        blockHC = addBlock("HC");
        addHuntCrossleyContact(blockHC);
        break;
    case Both:
        blockEC = addBlock("EC");
        addExponentialContact(blockEC);
        blockHC = addBlock("HC");
        addHuntCrossleyContact(blockHC);
    }

    // Add the external force
    if (applyFx) {
        setForceDataHeader();
        SimTK::Vec3 point(0.0, -hs, 0.0);
        SimTK::Vec3 zero(0.0);
        SimTK::Vec3 force(-0.7*gravity[1]*mass, 0.0, 0.0);
        setForceData(0.0, point, zero);
        setForceData(tf, point, zero);
        tf = tf + 25.0;
        setForceData(tf, point, force);
        if (blockEC) {
            cout << "Adding fx for " << blockEC->getName() << endl;
            fxEC = new ExternalForce(fxData,"force", "point", "",
                blockEC->getName(), "ground", blockEC->getName());
            fxEC->setName("externalforceES");
            model->addForce(fxEC);
        }
        if (blockHC) {
            cout << "Adding fx for " << blockHC->getName() << endl;
            fxHC = new ExternalForce(fxData, "force", "point", "",
                blockHC->getName(), "ground", blockHC->getName());
            fxHC->setName("externalforceHC");
            model->addForce(fxHC);
        }
    }

    // Reporters
    // StatesTrajectory
    statesReporter = new StatesTrajectoryReporter();
    statesReporter->setName("states_reporter");
    statesReporter->set_report_time_interval(0.1);
    model->addComponent(statesReporter);


    // Visuals?
    if (showVisuals) {
        if (blockEC) {
            auto blockESGeometry = new Brick(Vec3(hs));
            blockESGeometry->setColor(Vec3(0.1, 0.1, 0.8));
            blockEC->attachGeometry(blockESGeometry);
        }
        if (blockHC) {
            auto blockHCGeometry = new Brick(Vec3(hs));
            blockHCGeometry->setColor(Vec3(0.8, 0.1, 0.1));
            blockHC->attachGeometry(blockHCGeometry);
        }
        model->setUseVisualizer(true);
    }

    // Build the System
    model->buildSystem();
}
//______________________________________________________________________________
void
ContactPerformanceTester::
setForceDataHeader()
{
    fxData.setName("fx");
    fxData.setDescription("An external force applied to a block.");
    Array<std::string> lab; // labels
    lab.append("time");
    lab.append("point.x");
    lab.append("point.y");
    lab.append("point.z");
    lab.append("force.x");
    lab.append("force.y");
    lab.append("force.z");
    fxData.setColumnLabels(lab);
}
//______________________________________________________________________________
void
ContactPerformanceTester::
setForceData(double t, const SimTK::Vec3& point, const SimTK::Vec3& force)
{
    SimTK::Vector_<double> data(6);
    for (int i = 0; i < 3; ++i) {
        data[i] = point[i];
        data[3 + i] = force[i];
    }
    StateVector sv(t, data);
    fxData.append(sv);
}
//______________________________________________________________________________
OpenSim::Body*
ContactPerformanceTester::
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
ContactPerformanceTester::
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
//______________________________________________________________________________
void
ContactPerformanceTester::
addHuntCrossleyContact(OpenSim::Body* block)
{
    Ground& ground = model->updGround();

    // Geometry for the floor
    ContactHalfSpace* floor = new ContactHalfSpace(
            Vec3(0), Vec3(0, 0, -0.5 * SimTK_PI), ground, "floor");
    model->addContactGeometry(floor);

    // Place a contact sphere at each of the 8 corners
    std::string name = "";
    for (int i = 0; i < n; ++i) {
        // Geometry
        name = "sphere_" + std::to_string(i);
        geomHC[i] = new ContactSphere(0.005, corner[i], *block, name);
        model->addContactGeometry(geomHC[i]);

        // HuntCrossleyForce
        double dissipation = 4e-1;
        double mus = 0.7;
        double muk = 0.5;
        if (noDamp) {
            dissipation = 0.0;
            mus = 0.0;
            muk = 0.0;
        }
        auto* contactParams = new OpenSim::HuntCrossleyForce::
            ContactParameters(1.0e7, dissipation, mus, muk, 0.0);

        contactParams->addGeometry(name);
        contactParams->addGeometry("floor");
        sprHC[i] = new OpenSim::HuntCrossleyForce(contactParams);
        name = "HuntCrossleyForce_" + std::to_string(i);
        sprHC[i]->setName(name);
        sprHC[i]->setTransitionVelocity(0.01);
        model->addForce(sprHC[i]);
    }
}
//_____________________________________________________________________________
// dz allows for the body to be shifted along the z axis. This is useful for
// displacing the Exp and Hunt models upward or downward so that the initial
// normal contact force is reasonable.
void
ContactPerformanceTester::
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
ContactPerformanceTester::
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
    if (blockHC != NULL)
        setInitialConditions(state, blockHC->getMobilizedBody(), -dz);

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
    printConditions();
    cout << "           trys:  " << trys << endl;
    cout << "          steps:  " << steps << endl;
    cout << "       cpu time:  " << runTime << " msec" << endl;
}





//_____________________________________________________________________________
/* Entry Point (i.e., main())

The motion of a 10 kg, 6 degree-of-freedom block and its force interaction
with a laboratory floor are simulated.

Contact with the floor is modeled using either
    1) 8 ExponentialContact instances, one at each corner of the block, or
    2) 8 HuntCrossleyForce instances, one at each corner of the block.

For a side-by-side comparison of simulated motions, two blocks (one using
the ExponentialContact class for contact and the other using the
HuntCrossleyForce class) can be created and visualized simultaneously.

For an assessment of computational performance, just one block should be
simulated at a time. Number of integration trys and steps, as well as cpu
time, are reported.

Choice of initial conditions can be made in to simulate the following motions:
    1) Static (y = 0.1 m, sitting at rest on the floor)
    2) Bouncing (y = 1.0 m, dropped)
    3) Sliding (y = 0.2 m, vx = -4.0 m/s)
    4) Spinning (y = 0.1 m, vx = 0.0 m/s, wy = 8.0 pi rad/sec)
    5) Spining and Sliding (y = 0.1 m, vx = -3.0 m/s, wy = 4.0 pi rad/sec)
    6) Spinning like a Top. (y = 0.2 m, vx = 0.0 m/s, wy = 1.5 pi rad/sec)
    7) Tumbling (py = 0.2 m, vx = -1.0 m/s, wz = 2.0 pi rad/sec)

Additional options allow the following to be specified:
    NoDamp   Parameters are chosen to eliminate all energy dissipation.
    Fx       A ramping horizontal force (Fx) is applied.
    Vis      Opens a visualization window.

If no external force is applied, tf = 5.0 s.

If a horizontal force is applied, tf = 30.0 s and the force ramps up
linearly from a value of fx = 0.0 at t = 25.0 s to a value of
fx = |mass*g| at t = 30.0 s. The force is not ramped up prior to t = 25.0 s
in order to allow the block an opportunity to come fully to rest.
This ramping profile was done with the "Static" initial condition choice
in mind so that friction models could be evaluated more critically.
In particular, a static block should not start sliding until fx > μₛ Fₙ. */
int main(int argc, char** argv) {

    // Register object types
    RegisterTypes_osimActuators();

    // Create and execute the tester
    ContactPerformanceTester *tester = new ContactPerformanceTester;
    try {
        int status = tester->parseCommandLine(argc, argv);
        if (status < 0) {
            cout << "Exiting..." << endl;
            return 1;
        }
        tester->buildModel();
        tester->simulate();
    } catch (const OpenSim::Exception& e) {
        e.print(cerr);
        return 1;
    }

    // Clean up
    delete tester;
    cout << endl << endl << endl;
    cout << "------------------------------------------------------" << endl;
    cout << endl << endl << endl;

    return 0;
}




/**----------------------------------------------------------------------------
* Useful memory leak detection stuff for Windows:
*
/*
#ifdef _DEBUG
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type
#else
#define DBG_NEW new
#endif

#define _CRTDBG_MAP_ALLOC
*/



/*
// Static variables sometimes look like leaks but are not actual leaks.
// Leak detection for statics has to be done after main() completes. To ask
// the Crt memory manager to check after main() completes, set the bits for
// flag _CRTDBG_LEAK_CHECK_DF to true. Note that a memory leak dump only
// occurs if a leak is detected.
    _CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );

// Dump all memory leaks since execution began:
    _CrtDumpMemoryLeaks();

// To isolate leaks that occur in specific block of code, create a check point
// and filter out all leaks that occured before the check point:
    _CrtMemState memoryState = {0}; _CrtMemCheckpoint(&memoryState);
    ... the block of code to be tested ...
    _CrtMemDumpAllObjectsSince(&memoryState);

*/


/*

void testForXMLMemoryLeaks() {

    SimTK::String inFile =
        "C:/Users/fcand/Documents/GitHub/opensim-models/Models/RajagopalModel/Rajagopal2015.osim";

    // Deserialize
    OpenSim::Model model(inFile);
    cout << endl << endl << "Rajagopal2015 Model loaded!" << endl << endl;

    // Serialize with a new file name
    SimTK::String outFile =
        "C:/Users/fcand/Documents/GitHub/opensim-models/Models/RajagopalModel/Rajagopal2015_out.osim";
    model.print(outFile);
}


/* Part of main ...

// Check for mem leaks on a more complex model.
//testForXMLMemoryLeaks();

// Intentional leak
//_CrtMemState memoryStateB4Intentional = {0};
//_CrtMemCheckpoint(&memoryStateB4Intentional);
//char* intentionalLeak = new char[11];
//_CrtMemDumpAllObjectsSince(&memoryStateB4Intentional);

//_CrtMemState memoryState = {0}; _CrtMemCheckpoint(&memoryState);
//_CrtMemState memoryStateB4Sim = {0};

//char* intentionalLeak = new char[11];

//_CrtMemDumpAllObjectsSince(&memoryState);
//_CrtMemDumpAllObjectsSince(&memoryStateB4Sim);
char* intentionalLeak = new char[11];
//delete[] intentionalLeak;
//_CrtDumpMemoryLeaks();


*/
