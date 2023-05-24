﻿/* -------------------------------------------------------------------------- *
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
using std::cout;

//-----------------------------------------------------------------------------
// Local Utility Functions
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
template<class T>
void
appendVarElt(const string& path, const string& tag, const string& type,
    const Array_<T>& valArr, Element& parent, int precision)
{
    // Create the variable element.
    Element varElt(tag);
    varElt.setAttributeValue("path", path);
    varElt.setAttributeValue("type", type);

    // Append the variable element
    varElt.setValueAs<Array_<T>>(valArr, precision);
    parent.appendNode(varElt);
}

//-----------------------------------------------------------------------------
// Construction
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
StatesDocument::
StatesDocument(const Model& model, const Array_<State>& trajectory, int p)
{
    // Set ouput precisison (1 <= precision <= SimTK::LosslessNumDigitsReal)
    precision = clamp(1, p, SimTK::LosslessNumDigitsReal);
    formDoc(model, trajectory);
}

//-----------------------------------------------------------------------------
// Serialize
//-----------------------------------------------------------------------------
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
    Element rootElt = doc.getRootElement();

    // Insert a comment at the top level, just before the root node.
    string info = "OpenSim States Document (Version ";
    info += std::to_string(model.getDocumentFileVersion());
    info += ")";
    Xml::Comment comment(info);
    Xml::node_iterator root_it = doc.node_begin(Xml::ElementNode);
    doc.insertTopLevelNodeBefore(root_it, comment);

    // Date and time
    const std::time_t now = std::time(nullptr);
    const char *localeName = "C";
    std::locale::global(std::locale(localeName));
    char buf[64];
    strftime(buf, sizeof buf, "%a %b %e %Y %H:%M:%S", std::localtime(&now));
    SimTK::String datetime = buf;
    cout << "datetime = " << datetime;

    // Add attributes to the root node
    rootElt.setAttributeValue("model", model.getName());
    rootElt.setAttributeValue("nTime", std::to_string(traj.size()));
    rootElt.setAttributeValue("precision", std::to_string(precision));
    rootElt.setAttributeValue("date", buf);
}
//_____________________________________________________________________________
void
StatesDocument::
formTimeElement(const Model& model, const Array_<State>& traj) {
    // Form time element.
    Element timeElt = Element("time");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(timeElt);

    // Get time values from the StatesTrajectory
    int n = (int)traj.size();
    SimTK::Vector_<double> time(n);
    for (int i = 0; i < n; ++i) {
        time[i] = traj[i].getTime();
    }

    // Set the text value on the element
    timeElt.setValueAs<Vector_<double>>(time, precision);
}
//_____________________________________________________________________________
void
StatesDocument::
formContinuousElement(const Model& model, const Array_<State>& traj) {
    // Form continuous element.
    Element contElt = Element("continuous");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(contElt);

    // Get a list of all state variables names from the model.
    OpenSim::Array<std::string> paths = model.getStateVariableNames();

    // Loop over the names.
    // Get the vector of values of each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        Array_<double> val;
        model.getStateVariableTrajectory<double>(paths[i], traj, val);
        appendVarElt<double>(paths[i], "variable", "double", val, contElt, precision);
    }
}
//_____________________________________________________________________________
void
StatesDocument::
formDiscreteElement(const Model& model, const Array_<State>& traj) {
    // Form discrete element.
    Element discreteElt = Element("discrete");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(discreteElt);

    // Get a list of all discrete variable names from the model.
    OpenSim::Array<std::string> paths = model.getDiscreteVariableNames();

    // Loop over the names.
    // Get the vector of values for each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        // Get a single discrete variable so that its type can be discerned
        const AbstractValue &v =
            model.getDiscreteVariableAbstractValue(traj[0],paths[i]);

        // Append the vector according to type
        if (SimTK::Value<bool>::isA(v)) {
            Array_<bool> vArr;
            model.getDiscreteVariableTrajectory<bool>(
                paths[i], traj, vArr);
            appendVarElt<bool>(paths[i], "variable", "bool", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<int>::isA(v)) {
            Array_<int> vArr;
            model.getDiscreteVariableTrajectory<int>(
                paths[i], traj, vArr);
            appendVarElt<int>(paths[i], "variable", "int", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<float>::isA(v)) {
            Array_<float> vArr;
            model.getDiscreteVariableTrajectory<float>(
                paths[i], traj, vArr);
            appendVarElt<float>(paths[i], "variable", "float", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<double>::isA(v)) {
            Array_<double> vArr;
            model.getDiscreteVariableTrajectory<double>(
                paths[i], traj, vArr);
            appendVarElt<double>(paths[i], "variable", "double", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<Vec2>::isA(v)) {
            Array_<Vec2> vArr;
            model.getDiscreteVariableTrajectory<Vec2>(
                paths[i], traj, vArr);
            appendVarElt<Vec2>(paths[i], "variable", "Vec2", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<Vec3>::isA(v)) {
            Array_<Vec3> vArr;
            model.getDiscreteVariableTrajectory<Vec3>(
                paths[i], traj, vArr);
            appendVarElt<Vec3>(paths[i], "variable", "Vec3", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<Vec4>::isA(v)) {
            Array_<Vec4> vArr;
            model.getDiscreteVariableTrajectory<Vec4>(
                paths[i], traj, vArr);
            appendVarElt<Vec4>(paths[i], "variable", "Vec4", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<Vec5>::isA(v)) {
            Array_<Vec5> vArr;
            model.getDiscreteVariableTrajectory<Vec5>(
                paths[i], traj, vArr);
            appendVarElt<Vec5>(paths[i], "variable", "Vec5", vArr,
                discreteElt, precision);
        }
        else if(SimTK::Value<Vec6>::isA(v)) {
            Array_<Vec6> vArr;
            model.getDiscreteVariableTrajectory<Vec6>(
                paths[i], traj, vArr);
            appendVarElt<Vec6>(paths[i], "variable", "Vec6", vArr,
                discreteElt, precision);
        }
        else {
            string msg = "Unrecognized type: " + v.getTypeName();
            SimTK_ASSERT(false, msg.c_str());
        }
    }

}
//_____________________________________________________________________________
void
StatesDocument::
formModelingElement(const Model& model, const Array_<State>& traj) {
    // Form continuous element.
    Element modelingElt = Element("modeling");
    Element rootElt = doc.getRootElement();
    rootElt.appendNode(modelingElt);

    // Get a list of all modeling option names from the model.
    OpenSim::Array<std::string> paths = model.getModelingOptionNames();

    // Loop over the names.
    // Get the vector of values of each and append as a child element.
    int n = paths.getSize();
    for (int i = 0; i < n; ++i) {
        Array_<int> val;
        model.getModelingOptionTrajectory<int>(paths[i], traj, val);
        appendVarElt<int>(paths[i], "option", "int", val, modelingElt,
            precision);
    }
}


//-----------------------------------------------------------------------------
// Deserialize
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
void
StatesDocument::
deserialize(const Model& model, Array_<State>& traj) {
    parseDoc(model, traj);
}
//_____________________________________________________________________________
void
StatesDocument::
parseDoc(const Model& model, Array_<State>& traj) {
    checkDocConsistencyWithModel(model);
    prepareStatesTrajectory(model, traj);
    initializeTime(traj);
    initializeContinuousVariables(model, traj);
    initializeDiscreteVariables(model, traj);
    initializeModelingVariables(model, traj);
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
    Element rootElt = doc.getRootElement();
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
initializeTime(Array_<State>& traj) {
    // Find the element
    Element rootElt = doc.getRootElement();
    Array_<Element> timeElts = rootElt.getAllElements("time");
    SimTK_ASSERT1_ALWAYS(timeElts.size() == 1,
        "%d time elements found. Should only be 1.", timeElts.size());
    Element timeElt = timeElts[0];

    // Get the values
    Array_<double> timeArr;
    timeElt.getValueAs<Array_<double>>(timeArr);

    // Initialize the State objects
    int n = traj.size();
    for (int i = 0; i < n; ++i) traj[i].setTime(timeArr[i]);
}
//_____________________________________________________________________________
void
StatesDocument::
initializeContinuousVariables(const Model& model,
    SimTK::Array_<State>& traj)
{
    // Find the element
    Element rootElt = doc.getRootElement();
    Array_<Element> contElts = rootElt.getAllElements("discrete");
    SimTK_ASSERT1_ALWAYS(contElts.size() == 1,
        "%d continuous elements found. Should only be 1.", contElts.size());
    Element discElt = contElts[0];

}
//_____________________________________________________________________________
void
StatesDocument::
initializeDiscreteVariables(const Model& model,
    SimTK::Array_<State>& traj)
{
    Element rootElt = doc.getRootElement();
    Array_<Element> discElts = rootElt.getAllElements("discrete");
    SimTK_ASSERT1_ALWAYS(discElts.size() == 1,
        "%d discrete elements found. Should only be 1.", discElts.size());
    Element discElt = discElts[0];

}
//_____________________________________________________________________________
void
StatesDocument::
initializeModelingVariables(const Model& model,
    SimTK::Array_<State>& traj)
{
    // Find the element
    Element rootElt = doc.getRootElement();
    Array_<Element> modlElts = rootElt.getAllElements("modeling");
    SimTK_ASSERT1_ALWAYS(modlElts.size() == 1,
        "%d modeling elements found. Should only be 1.", modlElts.size());
    Element modlElt = modlElts[0];

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
    timeElt.setValueAs<Vector_<double>>(time, precision);

    // Experiment with output precision
    cout.unsetf(std::ios::floatfield);
    cout << setprecision(precision);
    cout << endl << time << endl << endl;

    // Hip Flexion
    SimTK::Vector_<double> q(num);
    for (i = 0; i < num; ++i) { q[i] = 1.0e-10 * SimTK::Pi * (double)i; }
    Xml::Element hipElt("variable");
    hipElt.setAttributeValue("path", "/jointset/hip/flexion/value");
    hipElt.setValueAs<Vector_<double>>(q, precision);
    continuousElt.appendNode(hipElt);

    // Elastic Anchor Point
    SimTK::Array_<Vec3> anchor(num);
    for (i = 0; i < num; ++i) {
        Vec3 val(0.0, 1.10000000001, 1.200000000000002);
        anchor[i] = ((double)i) * val;
    }
    Xml::Element anchorElt("variable");
    anchorElt.setAttributeValue("path", "/forceset/EC0/anchor");
    anchorElt.setValueAs<Array_<Vec3>>(anchor, precision);
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
    Array_<Vec3> anchorOut;
    anchorElt.getValueAs<Array_<Vec3>>(anchorOut);
    cout << endl << "anchorOut: " << anchorOut << endl;

    // Asserts
    SimTK_ASSERT_ALWAYS(anchor[0] == anchorOut[0],
            "Deserialized value not equal to original value.");
}
