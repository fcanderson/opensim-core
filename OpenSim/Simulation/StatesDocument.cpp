/* -------------------------------------------------------------------------- *
 *                   OpenSim:  StatesDocument.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2022-20232 Stanford University and the Authors               *
 * Author(s):  F. C. Anderson                                                 *
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
#include "StatesDocument.h"

using namespace SimTK;
using namespace SimTK::Xml;
using namespace std;
using namespace OpenSim;

//_____________________________________________________________________________
StatesDocument::
StatesDocument(const Model& model, const StatesTrajectory& traj) {

    formRootElement(model, traj);
    formTimeElement(model, traj);
    formContinuousElement(model, traj);
    formDiscreteElement(model, traj);
    formModelingElement(model, traj);
}

//_____________________________________________________________________________
StatesTrajectory
StatesDocument::
createStatesTrajectory(const Model& model, bool assemble) {

    bool success{false};

    // Create a local model.
    Model localModel(model);

    // Create an empty StatesTrajectory.
    StatesTrajectory traj;

    // How many State objects should there be?
    Element root = doc.getRootElement();
    Attribute numStateAttr = root.getOptionalAttribute("numStateObjects");
    int numStateObjects;
    success = numStateAttr.getValue().tryConvertTo<int>(numStateObjects);
    if (!success) {
        cout << "Unable to get numStateObjects. Throw." << endl;
    }

    // Fill the StatesTrajectory with the correct number of State objects.
    // At this point, none of the State objects has been initialized.
    SimTK::State state = localModel.initSystem();
    traj.reserve(numStateObjects);
    for (int i = 0; i < numStateObjects; ++i) traj.append(state);

    // Initialize the variables in each State object based on the contents
    // of the XML document.
    initializeContinuousVariables(model, traj);
    initializeDiscreteVariables(model, traj);
    initializeModelingVariables(model, traj);

    // No assembly needed?
    if (!assemble) return traj;

    // Assemble
    for (auto it = traj.begin_nonconst(); it!=traj.end_nonconst(); ++it) {
        localModel.assemble(*it);
    }

    return traj;
}


//-----------------------------------------------------------------------------
// Helper Methods for Construction
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
formRootElement(const Model& model, const StatesTrajectory &traj) {
    // Set the tag of the root element and get an iterator to it.
    doc.setRootTag("ostates");
    rootElt = doc.getRootElement();

    // Insert a comment at the top level, just before the root node.
    string info = "OpenSim States Document (Version ";
    info += std::to_string(model.getDocumentFileVersion());
    info += ")";
    Xml::Comment comment(info);
    Xml::node_iterator root_it = doc.node_begin(Xml::ElementNode);
    doc.insertTopLevelNodeBefore(root_it, comment);

    // Add attributes to the root node
    rootElt.setAttributeValue("model", model.getName());
    rootElt.setAttributeValue("numStateObjects", std::to_string(traj.getSize()));
}
//_____________________________________________________________________________
void
StatesDocument::
formTimeElement(const Model& model, const StatesTrajectory &traj) {
    // Form time element.
    timeElt = Element("time");
    rootElt.appendNode(timeElt);

    // Get time values from the StatesTrajectory
    int n = (int)traj.getSize();
    SimTK::Vector_<double> time(n);
    for (int i = 0; i < n; ++i) {
        time[i] = traj[i].getTime();
    }

    // Set the text value on the element
    int precision = 14;
    timeElt.setValueAs<Vector_<double>>(time, precision);
}
//_____________________________________________________________________________
void
StatesDocument::
formContinuousElement(const Model& model, const StatesTrajectory &traj) {
    // Form continuous element.
    continuousElt = Element("continuous");
    rootElt.appendNode(continuousElt);

    // Get a list of all state variables (continuous states) from the model.
    // Each state variable will have its own element.
    OpenSim::Array<std::string> paths = model.getStateVariableNames();

    // Append each variable element as a child to continuousElt.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        appendVariableElement(model, traj, paths[i], continuousElt);
    }

}
//_____________________________________________________________________________
void
StatesDocument::
formDiscreteElement(const Model& model, const StatesTrajectory &traj) {

}
//_____________________________________________________________________________
void
StatesDocument::
formModelingElement(const Model& model, const StatesTrajectory &traj) {

}
//_____________________________________________________________________________
void
StatesDocument::
appendVariableElement(const Model& model, const StatesTrajectory &traj,
    std::string& path, Element& parent)
{
    // Create the variable element.
    Element varElt("variable");
    varElt.setAttributeValue("path", path);

    // Get the values from the StateTrajectory
    Vector_<double> val;


    // Append the variable element
    varElt.setValueAs<Vector_<double>>(val, SimTK::LosslessNumDigitsReal);
    parent.appendNode(varElt);
}


//-----------------------------------------------------------------------------
// Helper Methods for StatesTrajectory Creation
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
initializeContinuousVariables(const Model& model, StatesTrajectory &traj) {

}
//_____________________________________________________________________________
void
StatesDocument::
initializeDiscreteVariables(const Model& model, StatesTrajectory &traj) {

}
//_____________________________________________________________________________
void
StatesDocument::
initializeModelingVariables(const Model& model, StatesTrajectory &traj) {

}


//-----------------------------------------------------------------------------
// Testing
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
test() {
    // See directly below this method
    prototype();

    // To String
    SimTK::String docStr;
    writeToString(docStr);
    cout << endl << "Prototype StatesDocument -----" << endl;
    cout << docStr << endl;

    // To File
    writeToFile("C:/Users/fcand/Documents/GitHub/Work/Testing/OpenSim/test.ostates");
}

//_____________________________________________________________________________
void
StatesDocument::
prototype() {

    // Set the tag of the root element and get an iterator to it.
    doc.setRootTag("ostates");
    Xml::Element& root = doc.getRootElement();
    Xml::node_iterator root_it = doc.node_begin(Xml::ElementNode);

    // Insert a comment at the top level, just before the root node.
    string info = "Developing class StatesDocument. Version 0.0.1.";
    Xml::Comment comment(info);
    doc.insertTopLevelNodeBefore(root_it, comment);

    // Add attributes to the root node
    root.setAttributeValue("model", "BouncingBlocks");

    // Add time and state category elements
    Xml::Element timeElt("time");
    root.appendNode(timeElt);
    Xml::Element continuousElt("continuous");
    root.appendNode(continuousElt);
    Xml::Element discreteElt("discrete");
    root.appendNode(discreteElt);
    Xml::Element modelingElt("modeling");
    root.appendNode(modelingElt);

    // Number of State% Objects
    int i;
    int num = 11;
    std::string numStr = std::to_string(num);
    root.setAttributeValue("numStateObjects", numStr);

    // Time
    SimTK::Vector_<double> time(num);
    for (i = 0; i < num; ++i) { time[i] = 0.1 * SimTK::Pi * (double)i; }
    timeElt.setValueAs<Vector_<double>>(time, SimTK::LosslessNumDigitsReal);

    // Experiment with output precision
    cout.unsetf(std::ios::floatfield);
    cout << setprecision(SimTK::LosslessNumDigitsReal);
    cout << endl << time << endl << endl;

    // Hip Flexion
    SimTK::Vector_<double> q(num);
    for (i = 0; i < num; ++i) { q[i] = 1.0e-10 * SimTK::Pi * (double)i; }
    Xml::Element hipElt("variable");
    hipElt.setAttributeValue("path", "/jointset/hip/flexion/value");
    hipElt.setValueAs<Vector_<double>>(q, SimTK::LosslessNumDigitsReal);
    continuousElt.appendNode(hipElt);

    // Elastic Anchor Point
    SimTK::Vector_<Vec3> anchor(num);
    for (i = 0; i < num; ++i) {
        Vec3 val(0.0, 1.10000000001, 1.200000000000002);
        anchor[i] = ((double)i) * val;
    }
    Xml::Element anchorElt("variable");
    anchorElt.setAttributeValue("path", "/forceset/EC0/anchor");
    anchorElt.setValueAs<Vector_<Vec3>>(anchor, SimTK::LosslessNumDigitsReal);
    discreteElt.appendNode(anchorElt);

    // Now -- Getting Vectors out!
    // Time
    Vector_<double> timeOut;
    timeElt.getValueAs<Vector_<double>>(timeOut);
    cout << endl << "timeOut: " << timeOut << endl;
    // Hip Flexion
    Vector_<double> qOut;
    hipElt.getValueAs<Vector_<double>>(qOut);
    cout << endl << "hipOut: " << qOut << endl;
    // Anchor
    Vector_<Vec3> anchorOut;
    anchorElt.getValueAs<Vector_<Vec3>>(anchorOut);
    cout << endl << "anchorOut: " << anchorOut << endl;

    // Asserts
    SimTK_ASSERT_ALWAYS(anchor[0] == anchorOut[0],
            "Deserialized value not equal to original value.");
}
