#ifndef OPENSIM_STATES_DOCUMENT_H_
#define OPENSIM_STATES_DOCUMENT_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  StatesDocument.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2023 Stanford University and the Authors                     *
 * Author(s): Frank C. Anderson                                                  *
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

// INCLUDE
#include <SimTKsimbody.h>
#include "osimSimulationDLL.h"
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/** Class StatesDocument provides a means of writing (serializing) and
reading (deserializing) a complete time history of model states to and from
a file. This capability is key when analyzing model behavior, visualizing
simulation results, and conducting a variety of computationally demanding
tasks (e.g., fitting a model to experimental data, solving optimal
control problems, etc.).

The states of an OpenSim::Model consist of all the independent variables that
change (or can change) during a simulation. At each time step during a
simulation, the underlying SimTK infrastructure captures the states in a
SimTK::State object. A state variable falls into one of the following
categories:

        1) Continuous Variables (aka OpenSim::StateVariable%s)
        2) Discrete Variables
        3) Modeling Options

Continuous Variables are governed by differential equations. They are
numerically integrated during a simulation based on the values of their
derivatives. Examples include joint coordinates, joint speeds, and muscle
activations. In OpenSim, because Continuous Variables are the most commonly
encountered kind of state, they are simply referred to as State Variables. All
concrete instances of Continuous Variables in OpenSim are derived from the
abstract class OpenSim::StateVariable.

Discrete Variable are not governed by differential equations and so can change
discontinuously during a simulation. Examples can include inputs to a
simulation, like muscle excitations, coefficients of friction, and torque
motor voltages. Examples can also include outputs to a simulation, like points
of contact between colliding bodies and whether those bodies are experiencing
static or kinetic frictional conditions. Such output discrete variables are
updated at each time step during numerical integration. Unlike continuous
states, however, they are updated based on closed-form algebraic expressions
rather than based on their derivatives. In the underlying SimTK infrastructure,
these output variables are implemented as a specialized kind of discrete
variable called an Auto-Update Discrete Variable.

Modeling Options are flags, usually of type int, that are used to choose
between viable ways to model the System or whether or not to apply a
constraint. Examples include a flag that specifies whether Euler angles or
quaternions are used to represent rotation or a flag that specifies whether a
particular joint coordinate is locked or unlocked. When a Modeling Options is
changed, low-level aspects of the System must be reconstituted or, in SimTK
terminology, re-realized through SimTK::Stage::Model.

Prior to the introduction of this class StatesDocument, only Continuous
Variables (i.e., OpenSim::StateVariable%s) were routinely and systematically
serialized, most commonly via the OpenSim::Manager as an OpenSim::Storage file
or via class OpenSim::StatesTrajectory as an OpenSim::TimeSeriesTable.
Discrete Variables and Modeling Options, if serialized, had to be stored in
separate files or handled as OpenSim::Property objects. In addition, prior to
this class, all Discrete Variables in OpenSim were assumed to be type double,
which is not a requirement of the underlying SimTK infrastructure.

With the introduction of this class, all state variables {i.e., Continuous
Variables (OpenSim::StateVariable%s), Discrete Variables, and Modeling Options}
can be serialized into a single file, which by convention has the```.ostates```
file name exention. In addition, a variety of types (e.g., bool, int, double,
Vec3, Vec4, etc.) are supported for Discrete Variables. Continuous States are
still assumed to be type double, and Modeling Options are still assumed to be
type int. Note, however, that the ```.ostates``` file format has the
flexibility to relax these assumptions and include other types if needed.

@note A point of clarification about Data Cache Variables...
By definition, state variables are independent. That is, the value of one
cannot be determined from the values of others. If a quantity of interest can
be computed from values of state variables, particularly if that quantity is
needed frequently, that quantity is often formalized as a Data Cache Variable.
The value of a Data Cach Variable is computed at each time step of a simulation
and stored in the SimTK::State. However, because a Data Cache Variable can
always be computed from the Continuous Variables, Discrete Variables, and
Modeling Options, they are not serialized.

        SimTK::State Contents    | Serialized in ```.ostates```?
        ------------------------ | -----------------------
        Continuous Variables     | yes
        Discrete Variables       | yes
        Modeling Options         | yes
        Data Cache Variables     | no


-----------------
Design Notes
-----------------

### Dependencies
Most operations in class StatesDocument rely on underlying SimTK classes,
most notably SimTK::String, SimTK::Vector_<T>, SimTK::Array<T>, SimTK::State,
and SimTK::Xml.

StatesDocument has just one key OpenSim dependency: OpenSim::Model.
OpenSim::Model brings with it all the methods it inherits from class
OpenSim::Component, which are essential for getting and setting state
information in OpenSim. StatesDocument does not know about classes like
OpenSim::Storage, OpenSim::TimeSeriesTable, OpenSim::StatesTrajectory, or
OpenSim::Manager.

Exchanges of state information between class StatesDocument and the rest of
OpenSim are accomplished via objects of type SimTK::Array_<SimTK::State>,
which are informally referred to as state trajectories (see directly below).

### Trajectories
In many methods of this class, as well as in related classes, you will
frequently encounter the term 'trajectory'. In these contexts, the term
connotes a time-ordered sequence, or a time-history, of values.

An array of knee angles (-10.0, -2.3, 4.5, 6.2, 7.1) would be termed a knee
angle trajectory if those knee angles were recorded sequentially during a
simulation. Similarly, an array of SimTK::State objects, if time ordered,
would be called a states trajectory.

Because of the flexibility and computational speed of the SimTK::Array_<T>
container class, you will often see trajectories passed in argument lists as
SimTK::Array_<T>%s. SimTK::Array_<double> might represent the trajectory of a
knee angle. SimTK::Array_<SimTK::Vec3> might represent the trajectory of the
center of presser between a foot and the floor during a walking motion.
SimTK::Array_<SimTK::State> is used to capture the full trajectory of states
(continuous variables, discrete variables, and modeling options) recorded
during a simulation.

This class relies heavily on a few trjectory-centric methods available in
the OpenSim::Component class. A few examples follow.

```
        template<class T>
        Component::getDiscreteVariableTrajectory(
                        const std::string& pathName,
                        const SimTK::Array_<SimTK::State>& input,
                        SimTK::Array_<T>& output) const
```
A call to the above method first finds a Discrete Variable in the model
hierarchy based on the specifed path name (```pathName```). Then, from the
input states trajectory (```input```), the method extracts the values of the
specified Discrete Variable and returns its trajectory as the output
(```output```). Notice that the type of the Discrete Variable can be specified
by the caller (i.e., T = int, double, Vec3, Vec4, etc.).

```
        template<class T>
        void setStatesTrajectoryForDiscreteVariable(
                        const std::string& pathName,
                        const SimTK::Array_<T>& input,
                        SimTK::Array_<SimTK::State>& output) const
```
On the other hand, based on the input trajectory of a specified Discrete
Variable (```input```), a call to the above method sets the appropriate
element in each of the SimTK::State objects held in the states trajectory
(```output```). Notice again that the type T of the Discrete Variable can be
specified by the caller.

### Complete and Constant XML Document upon Construction
Upon construction, a StatesDocument instance always contains a complete
internal XML document that represents a complete serialization of a specific
model's state trajectory. Moreover, that internal XML document cannot be
altered after construction!

If a model is changed (e.g., a muscle or contact model is added) or
a change has occurred in its state trajectory, the intended way to generate
an internal XML document that reflects those changes is to construct a new
StatesDocument instance. Constructing a new instance is the most reliable
approach for ensuring an accurate serialization. This approach also greatly
simplifies the implementation of the StatesDocument class, as methods for
selectively editing aspects of the internal XML document are not needed.
Finally, it should be noted that constructing a new StatesDocument instance
typically requires just a fraction of a second.

### Lossless Serialization
The precision with which numbers are serialized to a ```.ostates``` file can be
specified at the time of construction. The ```precision``` parameter specifies
the maximum number of significant digits used to represent numbers. If a
number can be represented without data loss with fewer digits, fewer digits
are used. In other words, trailing zeros are not written to file, thus
reducing file size. For example, if ```precision``` = 5, the number
1.50000000000000000000 would be represented in a ```.ostates``` file
as '1.5'; however, π would be represented as '3.1415'.

By default, the ```precision``` parameter of a StatesDocument is set to the
constant ```SimTK::LosslessNumDigitsReal```, which results in lossless
serialization. When ```precision``` = ```SimTK::LosslessNumDigitsReal```, the
```SimTK::State``` can be serialized and deserialized repeatedly without loss
of information. In applications where exact values of the states are needed,
lossless precision should be used.


-------------------
.ostate File Format
-------------------
XML is used as the organizing framework for ```.ostates``` files
(see SimTK::Xml), allowing them to be viewed and edited with a text editor.
Internet browsers can be also be used to view a ```.ostate``` file but may
require a ```.xml``` file extension to be added to the file name for the
XML format to be recognized.

### Sample .ostates File
```
<?xml version="1.0" encoding="UTF-8" ?>
<!--OpenSim States Document (Version 40000)-->
<ostates model="BouncingBlock" nTime="51" precision="3" date="Tue May 30 2023 03:42:40">
  <time>(0,0.1, ...)</time>
  <continuous>
    <variable path="/jointset/free/free_coord_0/value" type="double">(0,7.14, ...)</variable>
    <variable path="/jointset/free/free_coord_0/speed" type="double">(0,7.81, ...)</variable>
    ...
  </continuous>
  <discrete>
    <variable path="/forceset/EC0/anchor" type="Vec3">(~[2.1,-1.1,0],~[1.82,-1.1,0], ...)</variable>
    <variable path="/forceset/EC0/mu_kinetic" type="double">(0.5,0.5, ...)</variable>
    <variable path="/forceset/EC0/mu_static" type="double">(0.7,0.7, ...)</variable>
    <variable path="/forceset/EC0/sliding" type="double">(1,1, ...)</variable>
    ...
  </discrete>
  <modeling>
    <option path="/jointset/free/free_coord_0/is_clamped" type="int">(0,0, ...)</option>
    <option path="/jointset/free/free_coord_1/is_clamped" type="int">(0,0, ...)</option>
    ...
  </modeling>
</ostates>
```

### Deserialization Requirements
Successful deserialization of a .ostates file and full initialization of a
states trajectory for an OpenSim::Model requires the following:

    1) The name of the ```OpenSim::Model``` must match the value of the
    ```model``` attribute of the top-level ```ostates``` element.

    2) The number of values recorded for each ```variable``` and each
    ```option``` in the ```.ostates``` file must be equal to the value of the
    ```nTime``` attribute of the top-level ```ostates``` element.

    3) All ```variable``` and ```option``` paths must be found in the model
    OpenSim::Component heirarchy.

    4) The type must be supported. As of January 2024, the following types are
    supported:

            SimTK::State Category    | Supported Type(s)
            ------------------------ | -----------------------
            Continuous Variables     | double
                                     |
            Discrete Variables       | bool, int, float, double,
                                     | Vec2, Vec3, Vec4, Vec5, Vec6
                                     |
            Modeling Options         | int


--------------------------
Using Class StatesDocument
--------------------------
Below you will find several code snippets that show how the StatesDocument
class can be used to serialize (Example 1) and subsequently deserialize
(Example 2) the states of a model.



The first two examples (Examples 1 & 2) use a relatively high-level OpenSim
class, OpenSim::StatesTrajectory, to hold the state trajectories. The
advantage of using the StatesTrajectory class is that it can be used to create
both OpenSim::Storage and OpenSim::TimeSeriesTable objects in addition to a
StatesDocument object. Note that Storage objects and TimeSeriesTable objects
are currently suitable for handling continuous states but not discrete states.

The second two examples (Examples 3 & 4) use a relatively low-level class,
SimTK::Array_<SimTK::State>, to hold the state trajectories. This approach
is suitable for handling all categories of the system state (continuous,
discrete, and modeling) and allows the trajectory-oriented methods of the
OpenSim::Component API to be called directly.

### Example 1: Serializing Simulated States
```
    // ---------------
    // Build the Model
    // ---------------
    // Building a model can be done in many ways. The most common approach is
    // to construct a model from an OpenSim model file. Here, an empty model is
    // constructed with place holders for components that are typically added.
    OpenSim::Model model();
    model.setGravity( Vec3(0.0,-9.8,0.0) );
    model.setName("BouncingBlock");
    // Add bodies...
    // Add joints...
    // Add actuators & contact elements...

    // -------------------------------
    // Add a StatesTrajectory Reporter
    // -------------------------------
    // The reporter records the SimTK::State in a SimTK::Array_<> at a
    // time interval.
    OpenSim::StatesTrajectoryReporter
        reporter = new StatesTrajectoryReporter();
    reporter->setName("states_reporter");
    double interval = 0.01;
    reporter->set_report_time_interval(interval);
    model->addComponent(statesReporter);

    // -----------------------------------------
    // Build the System and Initialize the State
    // -----------------------------------------
    model.buildSystem();
    SimTK::State& state = model.initializeState();

    // ---------
    // Integrate
    // ---------
    Manager manager(*model);
    manager.getIntegrator().setMaximumStepSize(0.01);
    manager.setIntegratorAccuracy(1.0e-5);
    double ti = 0.0;
    double tf = 5.0;
    state.setTime(ti);
    manager.initialize(state);
    state = manager.integrate(tf);

    // -----------------------
    // Create a StatesDocument
    // -----------------------
    // Note that if the reporter were to return an unencapsulated trajectory
    // (i.e., Array_<State>), the export from class StatesTrajectory below can
    // be side stepped and the document created directly by using the following
    // constructor:
    // 
    // StatesDocument(const Model& model, const Array_<State>& trajectory,
    //      int precision = LosslessNumDigitsReal)
    //
    const StatesTrajectory& trajectory = reporter->getStates();
    StatesDocument doc = trajectory.exportToStatesDocument(model);

    // ----------------------------
    // Serialize the States to File
    // ----------------------------
    // For the file name, any string supported by the file system can be used.
    // The recommended convention, however, is for the file name to carry the
    // suffix ".ostates". Below, the suffix ".ostates" is simply added to the
    // name of the model.
    SimTK::String statesFileName = model.getName() + ".ostates";
    doc.serializeToFile(statesFileName);

    // ----------------------
    // Save the Model to File
    // ----------------------
    SimTK::String modelFileName = model.getName() + ".osim";
    model->print(modelFileName);

```

### Example 2: Deserializing States
```
    // ---------------------------
    // Construct a Model from File
    // ---------------------------
    SimTK::String name = "BouncingBlock";
    SimTK::String modelFileName = name + ".osim";
    OpenSim::Model model(modelFileName);
    model.buildSystem();
    SimTK::State& initState = model->initializeState();

    // -----------------------------------------------
    // Construct a StatesDocument Instance from File
    // -----------------------------------------------
    SimTK::String statesFileName = name + ".ostates";
    StatesDocument doc(statesFileName);

    // ----------------------
    // Deserialize the States
    // ----------------------
    // Note that model and document must be entirely consistent with each
    // other. In particular,
    // 
    // 1) The value of the "model" attribute of the root XML element of the
    // StatesDocument must match the name of the OpenSim::Model (i.e., the
    // string returned by model.getName()).
    //
    // 2) For every state in the model, there must be a coresponding node
    // in the StatesDocument that has a matching "path" attribute.
    //
    // If either of the above requirements is not met, an exception will
    // be thrown.
    SimTK::Array_<SimTK::State> traj;  // "traj" is short for "trajectory"
    doc.deserialize(model, traj);

    // Below are some things that can be done once a deserialized state
    // trajectory has been obtained.

    // ---------------------------------------------------
    // Iterate through the State Trajectory Getting Values
    // ---------------------------------------------------
    std::string path;
    const SimTK::State* iter;
    for(iter = traj.cbegin(); iter!=traj.cend(); ++iter) {
        
        // Get time
        double t = iter->getTime();

        // Get the value of a continuous state
        path = "/jointset/free/free_coord_0/value";
        double x = model.getStateVariableValue(*iter, path);

        // Get the value of a discrete state of type double
        path = "/forceset/EC0/sliding";
        double sliding = model.getDiscreteVariableValue(*iter, path);

        // Get the value of a discrete state of type Vec3
        path = "/forceset/EC0/anchor"
        const SimTK::AbstractValue& valAbs =
            model.getDiscreteVariableAbstractValue(*iter, path);
        SimTK::Value<Vec3> val = SimTK::Value<Vec3>::downcast( valAbs );
        Vec3 anchor = val.get();

        // Get the value of a modeling option
        path = "/jointset/free/free_coord_0/is_clamped";
        int clamped = model.getModelingOption(*iter, path);

    }

    // ------------------------------------------
    // Extract Trajectories for Individual States
    // ------------------------------------------
    // Continuous type double
    path = "/jointset/free/free_coord_0/value";
    SimTK::Array_<double> xTraj;
    model.getStateVariableTrajectory<double>(path, traj, xTraj);

    // Discrete type Vec3
    path = "/forceset/EC0/anchor";
    SimTK::Array_<Vec3> anchorTraj;
    model.getDiscreteVariableTrajectory<Vec3>(path, traj, anchorTraj);

    // Modeling type int
    path = "/jointset/free/free_coord_0/is_clamped";
    SimTK::Array_<int> clampedTraj;
    model.getModelingOptionTrajectory(path, traj, clampedTraj);

    // ----------------------
    // Form a TimeSeriesTable
    // ----------------------
    // Note that the table will only include the continuous states.
    // This might be done for plotting, post analysis, etc.
    StatesTrajectory trajectory(model, doc);
    OpenSim::TimesSeriesTable table = traj.exportToTable(model);

```

### A Final Note
Because Storage files (*.sto) and TimeSeriesTable files (*.??) typically
capture only the continuous states of a system, using these files as the basis
for deserialization runs the risk of leaving discrete variables and modeling
options in the SimTK::State uninitialized. In such an approach, additional
steps may need to be taken to properly initialize all variables in the
SimTK::State (e.g., by relying on OpenSim::Properties and/or on supplemental
input files).

In contrast, the StatesDocument class can be relied upon to yield a complete
serialization and deserialization of the SimTK::State. If the StatesDocument
class is used to serialize and then deserialize a state trajectory
(SimTK::Array_<SimTK::State>) that was recorded during a simulation, all
state variables in the SimTK::State (continuous, discrete, and modeling)
will be saved to a single file during serizaliztion and initialized upon
deserialization of that file.

@authors F. C. Anderson **/
class OSIMSIMULATION_API StatesDocument {

public:
    //-------------------------------------------------------------------------
    // Construction
    //-------------------------------------------------------------------------
    /** Construct from file. */
    StatesDocument(const SimTK::String& filename) {
        doc.readFromFile(filename);
    }

    /** Construct from a document string. */
    StatesDocument(const char* xmlDocument) {
        doc.readFromString(xmlDocument);
    }

    /** Construct from states trajectory. */
    StatesDocument(const OpenSim::Model& model,
        const SimTK::Array_<SimTK::State>& trajectory,
        int precision = SimTK::LosslessNumDigitsReal);

    //-------------------------------------------------------------------------
    // Serialization and Deserialization
    //-------------------------------------------------------------------------
    /** Serialize to a file. */
    void serializeToFile(const SimTK::String& filename) {
        doc.writeToFile(filename);
    }

    /** Serialize to a document string. */
    void serializeToString(SimTK::String& xmlDocument, bool compact = false) {
        doc.writeToString(xmlDocument, compact);
    }

    /** Deserialize to a states trajectory. */
    void deserialize(const OpenSim::Model& model,
        SimTK::Array_<SimTK::State>& trajectory);

    //-------------------------------------------------------------------------
    // Testing
    //-------------------------------------------------------------------------
    void test();

protected:
    // Serialization Helpers.
    void formDoc(const Model& model,
        const SimTK::Array_<SimTK::State>& traj);
    void formRootElement(const Model& model,
        const SimTK::Array_<SimTK::State>& traj);
    void formTimeElement(const Model& model,
        const SimTK::Array_<SimTK::State>& traj);
    void formContinuousElement(const Model& model,
        const SimTK::Array_<SimTK::State>& traj);
    void formDiscreteElement(const Model& model,
        const SimTK::Array_<SimTK::State>& traj);
    void formModelingElement(const Model& model,
        const SimTK::Array_<SimTK::State>& traj);

    // Deserialization Helpers.
    void parseDoc(const Model& m, SimTK::Array_<SimTK::State>& t);
    void checkDocConsistencyWithModel(const Model& model) const;
    void prepareStatesTrajectory(const Model& model,
        SimTK::Array_<SimTK::State> &traj);
    void initializeTime(SimTK::Array_<SimTK::State> &traj);
    void initializeContinuousVariables(const Model& model,
        SimTK::Array_<SimTK::State> &traj);
    void initializeDiscreteVariables(const Model& model,
        SimTK::Array_<SimTK::State> &traj);
    void initializeModelingOptions(const Model& model,
        SimTK::Array_<SimTK::State> &traj);

    // Testing
    void prototype();

private:
    // Member Variables
    int precision{SimTK::LosslessNumDigitsReal};
    SimTK::Xml::Document doc;

}; // END of class StatesDocument

} // end of namespace OpenSim

#endif OPENSIM_STATES_DOCUMENT_H_