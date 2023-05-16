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
using namespace std;
using namespace OpenSim;

//_____________________________________________________________________________
void
StatesDocument::
initialize() {

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

	// Number of State%s
    int i;
    int num = 11;
    std::string numStr = std::to_string(num);
    root.setAttributeValue("n", numStr);

	// Time
	SimTK::Vector_<double> time(num);
    for (i = 0; i < num; ++i) {
		time[i] = 0.1 * (double)i;
	}
    timeElt.setValueAs<Vector_<double>>(time);

	// Hip Flexion
    SimTK::Vector_<double> q(num);
    for (i = 0; i < num; ++i) {
		q[i] = 100.0 * (double)i;
	}
    Xml::Element hipElt("variable");
    hipElt.setAttributeValue("path", "/jointset/hip/flexion/value");
    hipElt.setValueAs<Vector_<double>>(q);
    continuousElt.appendNode(hipElt);

	// Elastic Anchor Point
    SimTK::Vector_<Vec3> anchor(num);
    for (i = 0; i < num; ++i) {
        Vec3 val(0.0, 1.10000000001, 1.200000000000002);
		anchor[i] = ((double)i) * val;
	}
    Xml::Element anchorElt("variable");
    anchorElt.setAttributeValue("path", "/forceset/EC0/anchor");
    anchorElt.setValueAs<Vector_<Vec3>>(anchor);
    discreteElt.appendNode(anchorElt);
}


//_____________________________________________________________________________
void
StatesDocument::
writeToFile(const std::string& fileName) {


}



//_____________________________________________________________________________
void
StatesDocument::
writeToString() {
	SimTK::String docStr;
    doc.writeToString(docStr);
    cout << endl << docStr << endl;
}



//_____________________________________________________________________________
void
StatesDocument::
test() {
    initialize();
    writeToString();
}
