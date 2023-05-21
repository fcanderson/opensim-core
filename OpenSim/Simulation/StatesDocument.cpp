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

//-----------------------------------------------------------------------------
// Local Utility Functions
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
template<class T>
void appendVarElt(const string& path, const Vector_<T>& val, Element& parent)
{
    // Create the variable element.
    Element varElt("variable");
    varElt.setAttributeValue("path", path);

    // Append the variable element
    varElt.setValueAs<Vector_<T>>(val, SimTK::LosslessNumDigitsReal);
    parent.appendNode(varElt);
}


//-----------------------------------------------------------------------------
// Serialize
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
serializeToFile(const String& filename,
    const Model& model, const Array_<State> & traj)
{
    formDoc(model, traj);
    doc.writeToFile(filename);
}
//_____________________________________________________________________________
void
StatesDocument::
formDoc(const Model& model, const Array_<State>& traj) {
    formRootElement(model, traj);
    formTimeElement(model, traj);
    formContinuousElement(model, traj);
    formDiscreteElement(model, traj);
    formModelingElement(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
formRootElement(const Model& model, const Array_<State>& traj) {
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
    rootElt.setAttributeValue("numStateObjects", std::to_string(traj.size()));
}
//_____________________________________________________________________________
void
StatesDocument::
formTimeElement(const Model& model, const Array_<State>& traj) {
    // Form time element.
    timeElt = Element("time");
    rootElt.appendNode(timeElt);

    // Get time values from the StatesTrajectory
    int n = (int)traj.size();
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
formContinuousElement(const Model& model, const Array_<State>& traj) {
    // Form continuous element.
    continuousElt = Element("continuous");
    rootElt.appendNode(continuousElt);

    // Get a list of all state variables names from the model.
    OpenSim::Array<std::string> paths = model.getStateVariableNames();

    // Loop over the names.
    // Get the vector of values of each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        Vector_<double> val;
        model.getStateVariableTrajectory<double>(paths[i], traj, val);
        appendVarElt<double>(paths[i], val, continuousElt);
    }
}
//_____________________________________________________________________________
void
StatesDocument::
formDiscreteElement(const Model& model, const Array_<State>& traj) {
    // Form discrete element.
    continuousElt = Element("discrete");
    rootElt.appendNode(discreteElt);

    // Get a list of all discrete variable names from the model.
    OpenSim::Array<std::string> paths = model.getDiscreteVariableNames();

    // Loop over the names.
    // Get the vector of values for each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        Vector_<double> val;
        model.getDiscreteVariableTrajectory<double>(paths[i], traj, val);
        appendVarElt<double>(paths[i], val, continuousElt);
    }
}
//_____________________________________________________________________________
void
StatesDocument::
formModelingElement(const Model& model, const Array_<State>& traj) {

}


//-----------------------------------------------------------------------------
// Deserialize
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
deserializeFromFile(const SimTK::String& filename,
    const Model& model, Array_<State>& traj)
{
    doc.readFromFile(filename);
    parseDoc(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
deserializeFromString(const SimTK::String& document,
    const Model& model, Array_<State>& traj)
{
    doc.readFromString(document);
    parseDoc(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
parseDoc(const Model& model, Array_<State>& traj) {
    findKeyDocElements();
    checkDocConsistencyWithModel(model);
    initializeContinuousVariables(model, traj);
    initializeDiscreteVariables(model, traj);
    initializeModelingVariables(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
findKeyDocElements() {
    // Root
    rootElt = doc.getRootElement();

    // Time
    Array_<Element> timeElts = rootElt.getAllElements("time");
    SimTK_ASSERT1_ALWAYS(timeElts.size() == 1,
        "%d time elements found. Should only be 1.", timeElts.size());
    timeElt = timeElts[0];

    // Continuous
    Array_<Element> contElts = rootElt.getAllElements("continuous");
    SimTK_ASSERT1_ALWAYS(contElts.size() == 1,
        "%d continuous elements found. Should only be 1.", contElts.size());
    continuousElt = contElts[0];

    // Discrete
    Array_<Element> discElts = rootElt.getAllElements("discrete");
    SimTK_ASSERT1_ALWAYS(discElts.size() == 1,
        "%d discrete elements found. Should only be 1.", discElts.size());
    discreteElt = discElts[0];

    // Modeling
    Array_<Element> modlElts = rootElt.getAllElements("modeling");
    SimTK_ASSERT1_ALWAYS(modlElts.size() == 1,
        "%d modeling elements found. Should only be 1.", modlElts.size());
    modelingElt = modlElts[0];
}
//_____________________________________________________________________________
void
StatesDocument::
checkDocConsistencyWithModel(const Model& model) const {

}
//_____________________________________________________________________________
void
StatesDocument::
prepareStatesTrajectory(const Model& model, Array_<State>& traj) {
    // Create a local copy of the Model and get a default State.
    Model localModel(model);
    SimTK::State state = localModel.initSystem();

    // How many State objects should there be?
    Attribute numStateAttr = rootElt.getOptionalAttribute("numStateObjects");
    int numStateObjects;
    bool success = numStateAttr.getValue().tryConvertTo<int>(numStateObjects);
    SimTK_ASSERT_ALWAYS(success,
        "Unable to acquire number of State objects from root element.");
    SimTK_ASSERT1_ALWAYS(numStateObjects > 0,
        "Root element attribute numStateObjects=%d; should be > 0.",
        numStateObjects);


}
//_____________________________________________________________________________
void
StatesDocument::
initializeContinuousVariables(const Model& model,
    SimTK::Array_<State>& traj) const
{

}
//_____________________________________________________________________________
void
StatesDocument::
initializeDiscreteVariables(const Model& model,
    SimTK::Array_<State>& traj) const
{

}
//_____________________________________________________________________________
void
StatesDocument::
initializeModelingVariables(const Model& model,
    SimTK::Array_<State>& traj) const
{

}


//-----------------------------------------------------------------------------
// Testing
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
test() {
    // Make up some data and elements.
    prototype();

    // Write to String
    SimTK::String docStr;
    doc.writeToString(docStr);
    cout << endl << "Prototype StatesDocument -----" << endl;
    cout << docStr << endl;

    // Write to File
    doc.writeToFile(
        "C:/Users/fcand/Documents/GitHub/Work/Testing/OpenSim/test.ostates");
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
