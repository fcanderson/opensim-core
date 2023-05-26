#ifndef OPENSIM_STATES_DOCUMENT_H_
#define OPENSIM_STATES_DOCUMENT_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  OStatesDoc.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2023 Stanford University and the Authors                     *
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
// INCLUDE
#include <SimTKsimbody.h>
#include "osimSimulationDLL.h"
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

//=============================================================================
// StatesDocument
//=============================================================================
/** Class StatesDocument provides a means of serializing and deserializing a
complete time sequence of states.
Basic XML is used as the serialization format (@See SimTK::Xml).

Upon construction, a StatesDocument object can be relied upon to contain
a complete XML document.

An OpenSim::StatesTrajectory is a time-ordered SimTK::Vector of SimTK::State%s
(i.e., Vector<State>), along with a number of helpful methods for accessing
those State%s and ensuring compatibility with a particular OpenSim::Model.

A SimTK::State object contains all the independent variables associated with
a SimTK::System that change (or can change) during the course of a simulation.
By definition, a state variable cannot be computed from other state variables.
At the very least, initial values of all state variables must be provided as
input to conduct a simulation. (If a quantity can be computed from the states,
that quantity is treated as a data cache variable.)

--------------------------
%State Categories
--------------------------
The SimTK::State is comprised of variables that fall into two categories:

    1) Continuous
    Continous states are governed by differential equations and
    rely upon numerical integration to obtain their time histories. Examples
    include generalized coordinates and speeds, among many other possibilities.
    In OpenSim, a continuous state is referred to as a State Variable and,
    in fact, must be wrapped by concrete classes derived from class
    OpenSim::StateVariable. See class OpenSim::Coordinate as an example.
    StateVariable%s can also be accessed via the OpenSim::Component API
    (e.g., see Component::setStateVariableValue()). They are uniformly assumed
    to be type double.

    2) Discrete
    Discrete states are not governed by differential equations
    and so can change discontinuously during a simulation. They come in three
    subcategories:

        A) Modeling
        Modeling discrete states are flags, usually of type bool
        or int, that are used to choose between viable ways to model the
        System. An example is a flag that specifies whether Euler angles or
        quaternions are used to represent rotation. When the value of a
        modeling discrete state is changed, low-level aspects of the System
        must be reconstituted or, to use Simbody terminology, re-realized.
        See SimTK::Stage, SimTK::State, and SimTK::System for details
        concerning realization. Typically, modeling discrete states are
        specified up front and do not change during the course of an
        integration. In OpenSim, modeling discrete states are referred to as
        Modeling Options. They can be accessed via the OpenSim::Component API
        (e.g., see Component::setModelingOption()) and are uniformly assumed
        to be type int.

        B) Regular
        Regular discrete states are variables that are likely to
        change during the course of a simulation. These changes, however, do
        not require low-level re-realization of the System. Typically, regular
        discrete states are used to represent the controls or independent
        inputs of a System (e.g., torque motor voltages, muscle excitation,
        etc.). In OpenSim, regular discrete states are referred to as Discrete
        Variables. They can be accessed via the OpenSim::Component API
        (e.g., see Component::updDiscreteVariableAbstractValue()). Discrete
        Variales are usually type double, but such is not always the case.
        They can be any type that can be represented by a SimTK::Value<T>
        (e.g., bool, int, SimTK::Vec3, etc.).

        C) Auto-Update
        Like regular discrete states, auto-update discrete
        states are expected to change during the course of a simulation and
        these changes do not require low-level re-realiztion of the System.
        Unlike regular discrete states, however, auto-update discrete states
        are more accurately thought of as ouputs rather than inputs. In many
        ways, auto-update discrete states are like continuous states. They
        are updated to new values at the end of every successful integration
        step. Their update values are just not governed by a set of
        differential equations. For a concrete example of an auto-update
        discrete state, see the elastic anchor point in class
        SimTK::ExponentialSpringForce. In OpenSim, no distinction is made
        between regular and auto-update discrete states. Auto-update discrete
        states are treated simply as Diescrete Variables. They can be accessed
        using the same OpenSim::Component API and conform to the same liberal
        type restriction (see Subcategory B directly above).

--------------------------
Choosing a Serialization Method
--------------------------
In OpenSim, there are three ways to serialize the time history of the
SimTK::State. Each has advantages and disadvantages.

--- Serialiation via OpenSim::Storage (.sto)
Before a simulation begins, you can set a flag on the OpenSim::Manager
requesting that it gather state information in an OpenSim::Storage object:

        manager.setWritetoStorage(true)

After the simulation completes, you can get a reference to the Storage
object and write its contents to file:

        Storage& store = manager.getStateStorage();
        store.print("model_name_states.sto");

All continuous states, but no discrete states, will be written to file in
tab-delimited columns in time-stamped rows. All quantities are type double.

Advantage
- Because Storage files can be imported directly into MatLab and most
spreadsheet programs, the time histories of continuous states can be readily
plotted.

Disadvantage
- Because discrete variables and modeling options are not included in the
output, a full SimTK::State cannot be reconstructed upon deserialization of
the Storage file. Additional information, such as control histories, must
additionally be imported from other sources.


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
    // Accessors
    //-------------------------------------------------------------------------


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
    SimTK::Array_<SimTK::AbstractValue*> supportedTypes;
    SimTK::Xml::Document doc;

}; // END of class StatesDocument

} // end of namespace OpenSim

#endif OPENSIM_STATES_DOCUMENT_H_