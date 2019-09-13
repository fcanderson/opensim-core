/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DeGrooteFregly2016Muscle.cpp                                 *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "DeGrooteFregly2016Muscle.h"

#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;

const std::string DeGrooteFregly2016Muscle::STATE_ACTIVATION_NAME("activation");
const std::string DeGrooteFregly2016Muscle::STATE_NORMALIZED_TENDON_FORCE_NAME(
        "normalized_tendon_force");
const std::string
        DeGrooteFregly2016Muscle::DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME(
                "implicitderiv_normalized_tendon_force");
const std::string
        DeGrooteFregly2016Muscle::RESIDUAL_NORMALIZED_TENDON_FORCE_NAME(
                "implicitresidual_normalized_tendon_force");

// We must define these variables in some compilation unit (pre-C++17).
// https://stackoverflow.com/questions/40690260/undefined-reference-error-for-static-constexpr-member?noredirect=1&lq=1
constexpr double DeGrooteFregly2016Muscle::b11;
constexpr double DeGrooteFregly2016Muscle::b21;
constexpr double DeGrooteFregly2016Muscle::b31;
constexpr double DeGrooteFregly2016Muscle::b41;
constexpr double DeGrooteFregly2016Muscle::b12;
constexpr double DeGrooteFregly2016Muscle::b22;
constexpr double DeGrooteFregly2016Muscle::b32;
constexpr double DeGrooteFregly2016Muscle::b42;
constexpr double DeGrooteFregly2016Muscle::b13;
constexpr double DeGrooteFregly2016Muscle::b23;
constexpr double DeGrooteFregly2016Muscle::b33;
constexpr double DeGrooteFregly2016Muscle::b43;

void DeGrooteFregly2016Muscle::constructProperties() {
    constructProperty_activation_time_constant(0.015);
    constructProperty_deactivation_time_constant(0.060);
    constructProperty_default_activation(0.5);
    constructProperty_default_normalized_tendon_force(0.5);
    constructProperty_active_force_width_scale(1.0);
    constructProperty_fiber_damping(0.01);
    constructProperty_tendon_strain_at_one_norm_force(0.049);
    constructProperty_ignore_passive_fiber_force(false);
    constructProperty_tendon_compliance_dynamics_mode("explicit");
}

void DeGrooteFregly2016Muscle::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    OPENSIM_THROW_IF_FRMOBJ(!getProperty_optimal_force().getValueIsDefault(),
            Exception,
            "The optimal_force property is ignored for this Force; "
            "use max_isometric_force instead.");

    SimTK_ERRCHK2_ALWAYS(get_activation_time_constant() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: activation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_activation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_deactivation_time_constant() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: deactivation_time_constant must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_deactivation_time_constant());

    SimTK_ERRCHK2_ALWAYS(get_default_activation() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_activation must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_default_activation());

    SimTK_ERRCHK2_ALWAYS(get_default_normalized_tendon_force() >= 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_normalized_tendon_force must be >= 0, but it is %g.",
            getName().c_str(), get_default_normalized_tendon_force());

    SimTK_ERRCHK2_ALWAYS(get_default_normalized_tendon_force() <= 5,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: default_normalized_tendon_force must be <= 5, but it is %g.",
            getName().c_str(), get_default_normalized_tendon_force());

    SimTK_ERRCHK2_ALWAYS(get_active_force_width_scale() >= 1,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: active_force_width_scale must be greater than or equal to "
            "1.0, "
            "but it is %g.",
            getName().c_str(), get_active_force_width_scale());

    SimTK_ERRCHK2_ALWAYS(get_fiber_damping() >= 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: fiber_damping must be greater than or equal to zero, "
            "but it is %g.",
            getName().c_str(), get_fiber_damping());

    SimTK_ERRCHK2_ALWAYS(get_tendon_strain_at_one_norm_force() > 0,
            "DeGrooteFregly2016Muscle::extendFinalizeFromProperties",
            "%s: tendon_strain_at_one_norm_force must be greater than zero, "
            "but it is %g.",
            getName().c_str(), get_tendon_strain_at_one_norm_force());

    OPENSIM_THROW_IF_FRMOBJ(
            get_pennation_angle_at_optimal() < 0 ||
                    get_pennation_angle_at_optimal() >
                            SimTK::Pi / 2.0 - SimTK::SignificantReal,
            InvalidPropertyValue,
            getProperty_pennation_angle_at_optimal().getName(),
            "Pennation angle at optimal fiber length must be in the range [0, "
            "Pi/2).");

    using SimTK::square;
    const auto normFiberWidth = sin(get_pennation_angle_at_optimal());
    m_fiberWidth = get_optimal_fiber_length() * normFiberWidth;
    m_squareFiberWidth = square(m_fiberWidth);
    m_maxContractionVelocityInMetersPerSecond =
            get_max_contraction_velocity() * get_optimal_fiber_length();
    m_kT = log((1.0 + c3) / c1) /
           (1.0 + get_tendon_strain_at_one_norm_force() - c2);
    m_isTendonDynamicsExplicit =
            get_tendon_compliance_dynamics_mode() == "explicit";
}

void DeGrooteFregly2016Muscle::extendAddToSystem(
        SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    if (!get_ignore_activation_dynamics()) {
        addStateVariable(STATE_ACTIVATION_NAME, SimTK::Stage::Dynamics);
    }
    if (!get_ignore_tendon_compliance()) {
        addStateVariable(
                STATE_NORMALIZED_TENDON_FORCE_NAME, SimTK::Stage::Dynamics);
        if (!m_isTendonDynamicsExplicit) {
            addDiscreteVariable(DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME,
                    SimTK::Stage::Dynamics);
            addCacheVariable(RESIDUAL_NORMALIZED_TENDON_FORCE_NAME, double(0),
                    SimTK::Stage::Dynamics);
        }
    }
}

void DeGrooteFregly2016Muscle::extendInitStateFromProperties(
        SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    if (!get_ignore_activation_dynamics()) {
        setActivation(s, get_default_activation());
    }
    if (!get_ignore_tendon_compliance()) {
        setNormalizedTendonForce(s, get_default_normalized_tendon_force());
    }
}

void DeGrooteFregly2016Muscle::extendSetPropertiesFromState(
        const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
    if (!get_ignore_activation_dynamics()) {
        set_default_activation(getActivation(s));
    }
    if (!get_ignore_tendon_compliance()) {
        set_default_normalized_tendon_force(getNormalizedTendonForce(s));
    }
}

void DeGrooteFregly2016Muscle::computeStateVariableDerivatives(
        const SimTK::State& s) const {

    // Activation dynamics.
    // --------------------
    if (!get_ignore_activation_dynamics()) {
        const auto& activation = getActivation(s);
        const auto& excitation = getControl(s);
        static const double actTimeConst = get_activation_time_constant();
        static const double deactTimeConst = get_deactivation_time_constant();
        static const double tanhSteepness = 0.1;
        //     f = 0.5 tanh(b(e - a))
        //     z = 0.5 + 1.5a
        // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
        const SimTK::Real timeConstFactor = 0.5 + 1.5 * activation;
        const SimTK::Real tempAct = 1.0 / (actTimeConst * timeConstFactor);
        const SimTK::Real tempDeact = timeConstFactor / deactTimeConst;
        const SimTK::Real f =
                0.5 * tanh(tanhSteepness * (excitation - activation));
        const SimTK::Real timeConst =
                tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
        const SimTK::Real derivative = timeConst * (excitation - activation);
        setStateVariableDerivativeValue(s, STATE_ACTIVATION_NAME, derivative);
    }

    // Tendon compliance dynamics.
    // ---------------------------
    if (!get_ignore_tendon_compliance()) {
        double normTendonForceDerivative;
        if (m_isTendonDynamicsExplicit) {
            const auto& normTendonForce = getNormalizedTendonForce(s);
            const auto& mli = getMuscleLengthInfo(s);
            const auto& muscleTendonVelocity = getLengtheningSpeed(s);
            const auto& activation = getActivation(s);

            SimTK::Real fiberForceVelocityMultiplier;
            SimTK::Real normFiberVelocity;
            SimTK::Real fiberVelocity;
            SimTK::Real fiberVelocityAlongTendon;
            SimTK::Real tendonVelocity;
            SimTK::Real normTendonVelocity;
            calcTendonVelocityInfoHelper(mli, muscleTendonVelocity, activation,
                    normTendonForce, fiberForceVelocityMultiplier,
                    normFiberVelocity, fiberVelocity, fiberVelocityAlongTendon,
                    tendonVelocity, normTendonVelocity);

            normTendonForceDerivative =
                    normTendonVelocity *
                    calcTendonForceMultiplierDerivative(mli.normTendonLength);
        } else {
            normTendonForceDerivative = getDiscreteVariableValue(
                    s, DERIVATIVE_NORMALIZED_TENDON_FORCE_NAME);
        }

        setStateVariableDerivativeValue(s, STATE_NORMALIZED_TENDON_FORCE_NAME,
                normTendonForceDerivative);
    }
}

double DeGrooteFregly2016Muscle::computeActuation(const SimTK::State& s) const {
    const auto& mdi = getMuscleDynamicsInfo(s);
    return mdi.tendonForce;
}

void DeGrooteFregly2016Muscle::calcMuscleLengthInfoHelper(
        const SimTK::Real& muscleTendonLength,
        const SimTK::Real& normTendonForce, MuscleLengthInfo& mli) const {

    // Tendon.
    // -------
    if (get_ignore_tendon_compliance()) {
        mli.normTendonLength = 1.0;
    } else {
        mli.normTendonLength =
                calcTendonForceLengthInverseCurve(normTendonForce);
    }
    mli.tendonStrain = mli.normTendonLength - 1.0;
    mli.tendonLength = get_tendon_slack_length() * mli.normTendonLength;

    // Fiber.
    // ------
    mli.fiberLengthAlongTendon = muscleTendonLength - mli.tendonLength;
    mli.fiberLength = sqrt(
            SimTK::square(mli.fiberLengthAlongTendon) + m_squareFiberWidth);
    mli.normFiberLength = mli.fiberLength / get_optimal_fiber_length();

    // Pennation.
    // ----------
    mli.cosPennationAngle = mli.fiberLengthAlongTendon / mli.fiberLength;
    mli.sinPennationAngle = m_fiberWidth / mli.fiberLength;
    mli.pennationAngle = asin(mli.sinPennationAngle);

    // Multipliers.
    // ------------
    mli.fiberPassiveForceLengthMultiplier =
            calcPassiveForceMultiplier(mli.normFiberLength);
    mli.fiberActiveForceLengthMultiplier =
            calcActiveForceLengthMultiplier(mli.normFiberLength);
}

void DeGrooteFregly2016Muscle::calcTendonVelocityInfoHelper(
        const MuscleLengthInfo& mli, const SimTK::Real& muscleTendonVelocity,
        const SimTK::Real& activation, const SimTK::Real& normTendonForce,
        SimTK::Real& fiberForceVelocityMultiplier,
        SimTK::Real& normFiberVelocity, SimTK::Real& fiberVelocity,
        SimTK::Real& fiberVelocityAlongTendon, SimTK::Real& tendonVelocity,
        SimTK::Real& normTendonVelocity) const {

    const auto& normFiberForce = normTendonForce / mli.cosPennationAngle;
    fiberForceVelocityMultiplier =
            (normFiberForce - mli.fiberPassiveForceLengthMultiplier) /
            (activation * mli.fiberActiveForceLengthMultiplier);
    normFiberVelocity =
            calcForceVelocityInverseCurve(fiberForceVelocityMultiplier);
    fiberVelocity =
            normFiberVelocity * m_maxContractionVelocityInMetersPerSecond;
    fiberVelocityAlongTendon = fiberVelocity / mli.cosPennationAngle;
    tendonVelocity = muscleTendonVelocity - fiberVelocityAlongTendon;
    normTendonVelocity = tendonVelocity / get_tendon_slack_length();
}

void DeGrooteFregly2016Muscle::calcFiberVelocityInfoHelper(
        const SimTK::Real& muscleTendonVelocity, const SimTK::Real& activation,
        const SimTK::Real& normTendonForce,
        const SimTK::Real& normTendonForceDerivative,
        const bool& isTendonDynamicsExplicit,
        const MuscleLengthInfo& mli, FiberVelocityInfo& fvi) const {

    if (isTendonDynamicsExplicit && !get_ignore_tendon_compliance()) {
        calcTendonVelocityInfoHelper(mli, muscleTendonVelocity, activation,
                normTendonForce, fvi.fiberForceVelocityMultiplier,
                fvi.normFiberVelocity, fvi.fiberVelocity,
                fvi.fiberVelocityAlongTendon, fvi.tendonVelocity,
                fvi.normTendonVelocity);
    } else {
        if (get_ignore_tendon_compliance()) {
            fvi.normTendonVelocity = 0.0;
        } else {
            fvi.normTendonVelocity =
                    calcTendonForceLengthInverseCurveDerivative(
                            normTendonForceDerivative, mli.normTendonLength);
        }
        fvi.tendonVelocity = get_tendon_slack_length() * fvi.normTendonVelocity;
        fvi.fiberVelocityAlongTendon =
                muscleTendonVelocity - fvi.tendonVelocity;
        fvi.fiberVelocity =
                fvi.fiberVelocityAlongTendon * mli.cosPennationAngle;
        fvi.normFiberVelocity =
                fvi.fiberVelocity / m_maxContractionVelocityInMetersPerSecond;
        fvi.fiberForceVelocityMultiplier =
                calcForceVelocityMultiplier(fvi.normFiberVelocity);
    }

    const SimTK::Real tanPennationAngle =
            m_fiberWidth / mli.fiberLengthAlongTendon;
    fvi.pennationAngularVelocity =
            -fvi.fiberVelocity / mli.fiberLength * tanPennationAngle;
}

void DeGrooteFregly2016Muscle::calcMuscleDynamicsInfoHelper(
        const SimTK::Real& activation, const SimTK::Real& normTendonForce,
        const SimTK::Real& muscleTendonVelocity, const MuscleLengthInfo& mli,
        const FiberVelocityInfo& fvi, MuscleDynamicsInfo& mdi) const {

    mdi.activation = activation;

    SimTK::Real activeFiberForce;
    SimTK::Real conPassiveFiberForce;
    SimTK::Real nonConPassiveFiberForce;
    SimTK::Real totalFiberForce;
    calcFiberForce(mdi.activation, mli.fiberActiveForceLengthMultiplier,
            fvi.fiberForceVelocityMultiplier,
            mli.fiberPassiveForceLengthMultiplier, fvi.normFiberVelocity,
            activeFiberForce, conPassiveFiberForce, nonConPassiveFiberForce,
            totalFiberForce);

    SimTK::Real passiveFiberForce =
            conPassiveFiberForce + nonConPassiveFiberForce;

    // TODO revisit this if compressive forces become an issue.
    //// When using a rigid tendon, avoid generating compressive fiber forces by
    //// saturating the damping force produced by the parallel element.
    //// Based on Millard2012EquilibriumMuscle::calcMuscleDynamicsInfo().
    // if (get_ignore_tendon_compliance()) {
    //    if (totalFiberForce < 0) {
    //        totalFiberForce = 0.0;
    //        nonConPassiveFiberForce = -activeFiberForce -
    //        conPassiveFiberForce; passiveFiberForce = conPassiveFiberForce +
    //        nonConPassiveFiberForce;
    //    }
    //}

    // Compute force entries.
    // ----------------------
    const auto maxIsometricForce = get_max_isometric_force();
    mdi.fiberForce = totalFiberForce;
    mdi.activeFiberForce = activeFiberForce;
    mdi.passiveFiberForce = passiveFiberForce;
    mdi.normFiberForce = mdi.fiberForce / maxIsometricForce;
    mdi.fiberForceAlongTendon = mdi.fiberForce * mli.cosPennationAngle;

    if (get_ignore_tendon_compliance()) {
        mdi.normTendonForce = mdi.normFiberForce * mli.cosPennationAngle;
        mdi.tendonForce = mdi.fiberForceAlongTendon;
    } else {
        mdi.normTendonForce = normTendonForce;
        mdi.tendonForce = maxIsometricForce * mdi.normTendonForce;
    }

    // Compute stiffness entries.
    // --------------------------
    mdi.fiberStiffness = calcFiberStiffness(mdi.activation, mli.normFiberLength,
            fvi.fiberForceVelocityMultiplier);
    const auto& partialPennationAnglePartialFiberLength =
            calcPartialPennationAnglePartialFiberLength(mli.fiberLength);
    const auto& partialFiberForceAlongTendonPartialFiberLength =
            calcPartialFiberForceAlongTendonPartialFiberLength(mdi.fiberForce,
                    mdi.fiberStiffness, mli.sinPennationAngle,
                    mli.cosPennationAngle,
                    partialPennationAnglePartialFiberLength);
    mdi.fiberStiffnessAlongTendon = calcFiberStiffnessAlongTendon(
            mli.fiberLength, partialFiberForceAlongTendonPartialFiberLength,
            mli.sinPennationAngle, mli.cosPennationAngle,
            partialPennationAnglePartialFiberLength);
    mdi.tendonStiffness = calcTendonStiffness(mli.normTendonLength);
    mdi.muscleStiffness = calcMuscleStiffness(
            mdi.tendonStiffness, mdi.fiberStiffnessAlongTendon);

    const auto& partialTendonForcePartialFiberLength =
            calcPartialTendonForcePartialFiberLength(mdi.tendonStiffness,
                    mli.fiberLength, mli.sinPennationAngle,
                    mli.cosPennationAngle);

    mdi.userDefinedDynamicsExtras = SimTK::Vector(3);
    mdi.userDefinedDynamicsExtras[0] = partialPennationAnglePartialFiberLength;
    mdi.userDefinedDynamicsExtras[1] =
            partialFiberForceAlongTendonPartialFiberLength;
    mdi.userDefinedDynamicsExtras[2] = partialTendonForcePartialFiberLength;

    // Compute power entries.
    // ----------------------
    // In order for the fiberPassivePower to be zero work, the non-conservative
    // passive fiber force is lumped into active fiber power. This is based on
    // the implementation in Millard2012EquilibriumMuscle (and verified over
    // email with Matt Millard).
    mdi.fiberActivePower = -(mdi.activeFiberForce + nonConPassiveFiberForce) *
                           fvi.fiberVelocity;
    mdi.fiberPassivePower = -conPassiveFiberForce * fvi.fiberVelocity;
    mdi.tendonPower = -mdi.tendonForce * fvi.tendonVelocity;
    mdi.musclePower = -mdi.tendonForce * muscleTendonVelocity;
}

void DeGrooteFregly2016Muscle::calcMusclePotentialEnergyInfoHelper(
        const MuscleLengthInfo& mli, MusclePotentialEnergyInfo& mpei) const {

    // Based on Millard2012EquilibriumMuscle::calcMusclePotentialEnergyInfo().

    // Fiber potential energy.
    // -----------------------
    mpei.fiberPotentialEnergy =
            calcPassiveForceMultiplierIntegral(mli.normFiberLength) *
            get_optimal_fiber_length() * get_max_isometric_force();

    // Tendon potential energy.
    // ------------------------
    mpei.tendonPotentialEnergy = 0;
    if (!get_ignore_tendon_compliance()) {
        mpei.tendonPotentialEnergy =
                calcTendonForceMultiplierIntegral(mli.normTendonLength) *
                get_tendon_slack_length() * get_max_isometric_force();
    }

    // Total potential energy.
    // -----------------------
    mpei.musclePotentialEnergy =
            mpei.fiberPotentialEnergy + mpei.tendonPotentialEnergy;
}

void DeGrooteFregly2016Muscle::calcMuscleLengthInfo(
        const SimTK::State& s, MuscleLengthInfo& mli) const {

    const auto& muscleTendonLength = getLength(s);
    SimTK::Real normTendonForce = 0.0;
    if (!get_ignore_tendon_compliance()) {
        normTendonForce = getNormalizedTendonForce(s);
    }
    calcMuscleLengthInfoHelper(muscleTendonLength, normTendonForce, mli);

    if (mli.tendonLength < get_tendon_slack_length() && getPrintWarnings()) {
        // TODO the Millard model sets fiber velocity to zero when the
        //       tendon is buckling, but this may create a discontinuity.
        std::cout << "Warning: DeGrooteFregly2016Muscle '" << getName()
                  << "' is buckling (length < tendon_slack_length) at time "
                  << s.getTime() << " s." << std::endl;
    }
}

void DeGrooteFregly2016Muscle::calcFiberVelocityInfo(
        const SimTK::State& s, FiberVelocityInfo& fvi) const {

    const auto& mli = getMuscleLengthInfo(s);
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& activation = getActivation(s);

    SimTK::Real normTendonForce = 0.0;
    SimTK::Real normTendonForceDerivative = 0.0;
    if (!get_ignore_tendon_compliance()) {
        if (m_isTendonDynamicsExplicit) {
            normTendonForce = getNormalizedTendonForce(s);
        } else {
            normTendonForceDerivative = getNormalizedTendonForceDerivative(s);
        }
    }

    calcFiberVelocityInfoHelper(muscleTendonVelocity, activation,
            normTendonForce, normTendonForceDerivative,
            m_isTendonDynamicsExplicit, mli, fvi);
}

void DeGrooteFregly2016Muscle::calcMuscleDynamicsInfo(
        const SimTK::State& s, MuscleDynamicsInfo& mdi) const {
    const auto& activation = getActivation(s);
    SimTK::Real normTendonForce = 0.0;
    if (!get_ignore_tendon_compliance()) {
        normTendonForce = getNormalizedTendonForce(s);
    }
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& mli = getMuscleLengthInfo(s);
    const auto& fvi = getFiberVelocityInfo(s);

    calcMuscleDynamicsInfoHelper(
            activation, normTendonForce, muscleTendonVelocity, mli, fvi, mdi);
}

void DeGrooteFregly2016Muscle::calcMusclePotentialEnergyInfo(
        const SimTK::State& s, MusclePotentialEnergyInfo& mpei) const {
    const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
    calcMusclePotentialEnergyInfoHelper(mli, mpei);
}

void DeGrooteFregly2016Muscle::computeInitialFiberEquilibrium(
        SimTK::State& s) const {
    if (get_ignore_tendon_compliance()) return;

    const auto& muscleTendonLength = getLength(s);
    const auto& muscleTendonVelocity = getLengtheningSpeed(s);
    const auto& activation = getActivation(s);

    // We have to use the implicit form of the model since the explicit form
    // will produce a zero residual for any guess of normalized tendon force.
    // The implicit form requires a guess for normalized tendon force 
    // derivative, so we'll assume it's zero for simplicity.
    const SimTK::Real normTendonForceDerivative = 0.0;

    MuscleLengthInfo mli;
    FiberVelocityInfo fvi;
    MuscleDynamicsInfo mdi;

    auto calcResidual = [this, &muscleTendonLength, &muscleTendonVelocity,
                                &normTendonForceDerivative, &activation, &mli,
                                &fvi,
                                &mdi](const SimTK::Real& normTendonForce) {
        calcMuscleLengthInfoHelper(muscleTendonLength, normTendonForce, mli);
        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation,
                normTendonForce, normTendonForceDerivative, false, mli, fvi);
        calcMuscleDynamicsInfoHelper(activation, normTendonForce,
                muscleTendonVelocity, mli, fvi, mdi);

        return calcEquilibriumResidual(
                mdi.tendonForce, mdi.fiberForceAlongTendon);
    };

    const auto equilNormTendonForce = solveBisection(calcResidual,
            m_minNormTendonForce, m_maxNormTendonForce, 1e-10, 1e-10, 1000);

    setNormalizedTendonForce(s, equilNormTendonForce);

    // TODO not working as well as bisection, revisist later.
    //const double tolerance = std::max(
    //        1e-8 * get_max_isometric_force(), SimTK::SignificantReal * 10);
    //int maxIterations = 1000;

    //try {
    //    auto result = estimateMuscleFiberState(activation, 
    //            muscleTendonLength, muscleTendonVelocity, 
    //            normTendonForceDerivative, tolerance, maxIterations);

    //    switch (result.first) {

    //    case Success_Converged:
    //        setNormalizedTendonForce(s, result.second["norm_tendon_force"]);
    //        setActuation(s, get_max_isometric_force() *
    //                                result.second["norm_tendon_force"]);
    //        break;

    //    case Warning_FiberAtLowerBound:
    //        printf("\n\nDeGrooteFregly2016Muscle static solution:"
    //               " %s is at its minimum fiber length of %f\n",
    //                getName().c_str(), result.second["fiber_length"]);
    //        setNormalizedTendonForce(s, result.second["norm_tendon_force"]);
    //        setActuation(s, get_max_isometric_force() *
    //                                result.second["norm_tendon_force"]);
    //        break;

    //    case Warning_FiberAtUpperBound:
    //        printf("\n\nDeGrooteFregly2016Muscle static solution:"
    //               " %s is at its maximum fiber length of %f\n",
    //                getName().c_str(), result.second["fiber_length"]);
    //        setNormalizedTendonForce(s, result.second["norm_tendon_force"]);
    //        setActuation(s, get_max_isometric_force() *
    //                                result.second["norm_tendon_force"]);
    //        break;

    //    case Failure_MaxIterationsReached:
    //        std::ostringstream ss;
    //        ss << "\n  Solution error " << abs(result.second["solution_error"])
    //           << " exceeds tolerance of " << tolerance << "\n"
    //           << "  Newton iterations reached limit of " << maxIterations
    //           << "\n"
    //           << "  Activation is " << activation << "\n"
    //           << "  Fiber length is " << result.second["fiber_length"] << "\n";
    //        OPENSIM_THROW_FRMOBJ(MuscleCannotEquilibrate, ss.str());
    //    }

    //} catch (const std::exception& x) {
    //    OPENSIM_THROW_FRMOBJ(MuscleCannotEquilibrate,
    //            "Internal exception encountered.\n" + std::string{x.what()});
    //}
    
}

SimTK::Real DeGrooteFregly2016Muscle::solveBisection(
        std::function<SimTK::Real(const SimTK::Real&)> calcResidual,
        SimTK::Real left, SimTK::Real right, const SimTK::Real& xTolerance,
        const SimTK::Real& yTolerance, int maxIterations) const {
    SimTK::Real midpoint = left;

    OPENSIM_THROW_IF_FRMOBJ(maxIterations < 0, Exception,
            format("Expected maxIterations to be positive, but got %i.",
                    maxIterations));

    const bool sameSign = calcResidual(left) * calcResidual(right) >= 0;
    if (sameSign) {
        const auto x = createVectorLinspace(1000, left, right);
        TimeSeriesTable table;
        table.setColumnLabels({"residual"});
        SimTK::RowVector row(1);
        for (int i = 0; i < x.nrow(); ++i) {
            row[0] = calcResidual(x[i]);
            table.appendRow(x[i], row);
        }
        writeTableToFile(table, "DEBUG_solveBisection_residual.sto");
    }
    OPENSIM_THROW_IF_FRMOBJ(sameSign, Exception,
            format("Function has same sign at bounds of %f and %f.", left,
                    right));

    SimTK::Real residualMidpoint;
    SimTK::Real residualLeft = calcResidual(left);
    int iterCount = 0;
    while (iterCount < maxIterations && (right - left) >= xTolerance) {
        midpoint = 0.5 * (left + right);
        residualMidpoint = calcResidual(midpoint);
        if (std::abs(residualMidpoint) < yTolerance) {
            break;
        } else if (residualMidpoint * residualLeft < 0) {
            // The solution is to the left of the current midpoint.
            right = midpoint;
        } else {
            left = midpoint;
            residualLeft = calcResidual(left);
        }
        ++iterCount;
    }
    if (iterCount == maxIterations)
        printMessage("Warning: bisection reached max iterations "
                     "at x = %g (%s %s).\n",
                midpoint, getConcreteClassName(), getName());
    return midpoint;
}

std::pair<DeGrooteFregly2016Muscle::StatusFromEstimateMuscleFiberState,
        DeGrooteFregly2016Muscle::ValuesFromEstimateMuscleFiberState>
DeGrooteFregly2016Muscle::estimateMuscleFiberState(const double activation,
        const double muscleTendonLength, const double muscleTendonVelocity,
        const double normTendonForceDerivative, const double tolerance,
        const int maxIterations) const {

    MuscleLengthInfo mli;
    FiberVelocityInfo fvi;
    MuscleDynamicsInfo mdi;

    double normTendonForce = get_default_normalized_tendon_force();
    calcMuscleLengthInfoHelper(muscleTendonLength, normTendonForce, mli);

    double fiberLength = mli.fiberLength;
    double residual = SimTK::MostPositiveReal;
    double partialFiberForceAlongTendonPartialFiberLength = 0.0;
    double partialTendonForcePartialFiberLength = 0.0;
    double partialResidualPartialFiberLength = 0.0;
    double deltaFiberLength = 0.0;

    // Helper functions
    // ----------------
    // Update position level quantities.
    auto positionFunc = [&] {
        const auto& fiberLengthAlongTendon =
                sqrt(SimTK::square(fiberLength) - m_squareFiberWidth);
        const auto& tendonLength = muscleTendonLength - fiberLengthAlongTendon;
        const auto& normTendonLength = tendonLength / get_tendon_slack_length();
        normTendonForce = calcTendonForceMultiplier(normTendonLength);
        calcMuscleLengthInfoHelper(muscleTendonLength, normTendonForce, mli);
    };
    // Update velocity and dynamics level quantities and compute residual.
    auto dynamicsFunc = [&] {
        calcFiberVelocityInfoHelper(muscleTendonVelocity, activation,
                normTendonForce, normTendonForceDerivative, false, mli, fvi);
        calcMuscleDynamicsInfoHelper(activation, normTendonForce,
                muscleTendonVelocity, mli, fvi, mdi);

        partialFiberForceAlongTendonPartialFiberLength =
                mdi.userDefinedDynamicsExtras[1];
        partialTendonForcePartialFiberLength = mdi.userDefinedDynamicsExtras[2];

        residual = calcEquilibriumResidual(
                mdi.tendonForce, mdi.fiberForceAlongTendon);
    };

    // Initialize the loop.
    int iter = 0;
    positionFunc();
    dynamicsFunc();
    double residualPrev = residual;
    double fiberLengthPrev = fiberLength;
    double h = 1.0;

    while ((abs(residual) > tolerance) && (iter < maxIterations)) {
        // Compute the search direction.
        partialResidualPartialFiberLength =
                partialFiberForceAlongTendonPartialFiberLength -
                partialTendonForcePartialFiberLength;
        h = 1.0;

        while (abs(residual) >= abs(residualPrev)) {
            // Compute the Newton step.
            deltaFiberLength =
                    -h * residualPrev / partialResidualPartialFiberLength;

            // Take a Newton step if the step is nonzero.
            if (abs(deltaFiberLength) > SimTK::SignificantReal)
                fiberLength = fiberLengthPrev + deltaFiberLength;
            else {
                // We've stagnated or hit a limit; assume we are hitting local
                // minimum and attempt to approach from the other direction.
                fiberLength = fiberLengthPrev -
                              SimTK::sign(deltaFiberLength) * SimTK::SqrtEps;
                h = 0;
            }

            if (fiberLength / get_optimal_fiber_length() <
                    m_minNormFiberLength) {
                fiberLength = m_minNormFiberLength * get_optimal_fiber_length();
            }
            if (fiberLength / get_optimal_fiber_length() >
                    m_maxNormFiberLength) {
                fiberLength = m_maxNormFiberLength * get_optimal_fiber_length();
            }

            positionFunc();
            dynamicsFunc();

            if (h <= SimTK::SqrtEps) { break; }
            h = 0.5 * h;
        }

        residualPrev = residual;
        fiberLengthPrev = fiberLength;

        iter++;
    }

    // Populate the result map.
    ValuesFromEstimateMuscleFiberState resultValues;

    if (abs(residual) < tolerance) { // The solution converged.

        resultValues["solution_error"] = residual;
        resultValues["iterations"] = (double)iter;
        resultValues["fiber_length"] = fiberLength;
        resultValues["fiber_velocity"] = fvi.fiberVelocity;
        resultValues["norm_tendon_force"] = mdi.normTendonForce;

        return std::pair<StatusFromEstimateMuscleFiberState,
                ValuesFromEstimateMuscleFiberState>(
                Success_Converged, resultValues);
    }

    // Fiber length is at or exceeds its lower bound.
    if (fiberLength / get_optimal_fiber_length() <= m_minNormFiberLength) {

        fiberLength = m_minNormFiberLength * get_optimal_fiber_length();
        positionFunc();
        normTendonForce = calcTendonForceMultiplier(mli.normTendonLength);

        resultValues["solution_error"] = residual;
        resultValues["iterations"] = (double)iter;
        resultValues["fiber_length"] = fiberLength;
        resultValues["fiber_velocity"] = 0;
        resultValues["norm_tendon_force"] = normTendonForce;

        return std::pair<StatusFromEstimateMuscleFiberState,
                ValuesFromEstimateMuscleFiberState>(
                Warning_FiberAtLowerBound, resultValues);
    }

    // Fiber length is at or exceeds its upper bound.
    if (fiberLength / get_optimal_fiber_length() >= m_maxNormFiberLength) {

        fiberLength = m_maxNormFiberLength * get_optimal_fiber_length();
        positionFunc();
        normTendonForce = calcTendonForceMultiplier(mli.normTendonLength);

        resultValues["solution_error"] = residual;
        resultValues["iterations"] = (double)iter;
        resultValues["fiber_length"] = fiberLength;
        resultValues["fiber_velocity"] = 0;
        resultValues["norm_tendon_force"] = normTendonForce;

        return std::pair<StatusFromEstimateMuscleFiberState,
                ValuesFromEstimateMuscleFiberState>(
                Warning_FiberAtUpperBound, resultValues);
    }

    // Max iterations reached.
    resultValues["solution_error"] = residual;
    resultValues["iterations"] = (double)iter;
    resultValues["fiber_length"] = SimTK::NaN;
    resultValues["fiber_velocity"] = SimTK::NaN;
    resultValues["norm_tendon_force"] = SimTK::NaN;

    return std::pair<StatusFromEstimateMuscleFiberState,
            ValuesFromEstimateMuscleFiberState>(
            Failure_MaxIterationsReached, resultValues);
}

double DeGrooteFregly2016Muscle::getImplicitResidualNormalizedTendonForce(
        const SimTK::State& s) const {
    // TODO: What to do if implicit is disabled?
    // Recompute residual if cache is invalid.
    if (!isCacheVariableValid(
                s, "implicitresidual_" + STATE_NORMALIZED_TENDON_FORCE_NAME)) {
        // Compute muscle-tendon equilibrium residual value to update the
        // cache variable.
        setCacheVariableValue(s,
                "implicitresidual_" + STATE_NORMALIZED_TENDON_FORCE_NAME,
                getEquilibriumResidual(s));
        markCacheVariableValid(
                s, "implicitresidual_" + STATE_NORMALIZED_TENDON_FORCE_NAME);
    }

    return getCacheVariableValue<double>(
            s, "implicitresidual_" + STATE_NORMALIZED_TENDON_FORCE_NAME);
}

DataTable DeGrooteFregly2016Muscle::exportFiberLengthCurvesToTable(
        const SimTK::Vector& normFiberLengths) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normFiberLengths.nrow()) {
        x = &normFiberLengths;
    } else {
        def = createVectorLinspace(
                200, m_minNormFiberLength, m_maxNormFiberLength);
        x = &def;
    }

    DataTable table;
    table.setColumnLabels(
            {"active_force_length_multiplier", "passive_force_multiplier"});
    SimTK::RowVector row(2);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normFiberLength = x->get(irow);
        row[0] = calcActiveForceLengthMultiplier(normFiberLength);
        row[1] = calcPassiveForceMultiplier(normFiberLength);
        table.appendRow(normFiberLength, row);
    }
    return table;
}

DataTable DeGrooteFregly2016Muscle::exportTendonForceMultiplierToTable(
        const SimTK::Vector& normTendonLengths) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normTendonLengths.nrow()) {
        x = &normTendonLengths;
    } else {
        // Evaluate the inverse of the tendon curve at y = 1.
        def = createVectorLinspace(
                200, 0.95, 1.0 + get_tendon_strain_at_one_norm_force());
        x = &def;
    }

    DataTable table;
    table.setColumnLabels({"tendon_force_multiplier"});
    SimTK::RowVector row(1);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normTendonLength = x->get(irow);
        row[0] = calcTendonForceMultiplier(normTendonLength);
        table.appendRow(normTendonLength, row);
    }
    return table;
}

DataTable DeGrooteFregly2016Muscle::exportFiberVelocityMultiplierToTable(
        const SimTK::Vector& normFiberVelocities) const {
    SimTK::Vector def;
    const SimTK::Vector* x = nullptr;
    if (normFiberVelocities.nrow()) {
        x = &normFiberVelocities;
    } else {
        def = createVectorLinspace(200, -1.1, 1.1);
        x = &def;
    }

    DataTable table;
    table.setColumnLabels({"force_velocity_multiplier"});
    SimTK::RowVector row(1);
    for (int irow = 0; irow < x->nrow(); ++irow) {
        const auto& normFiberVelocity = x->get(irow);
        row[0] = calcForceVelocityMultiplier(normFiberVelocity);
        table.appendRow(normFiberVelocity, row);
    }
    return table;
}

void DeGrooteFregly2016Muscle::printCurvesToSTOFiles(
        const std::string& directory) const {
    std::string prefix =
            directory + SimTK::Pathname::getPathSeparator() + getName();
    writeTableToFile(exportFiberLengthCurvesToTable(),
            prefix + "_fiber_length_curves.sto");
    writeTableToFile(exportFiberVelocityMultiplierToTable(),
            prefix + "_fiber_velocity_multiplier.sto");
    writeTableToFile(exportTendonForceMultiplierToTable(),
            prefix + "_tendon_force_multiplier.sto");
}

void DeGrooteFregly2016Muscle::replaceMuscles(
        Model& model, bool allowUnsupportedMuscles) {

    model.finalizeConnections();

    // Create path actuators from muscle properties and add to the model. Save
    // a list of pointers of the muscles to delete.
    std::vector<Muscle*> musclesToDelete;
    auto& muscleSet = model.updMuscles();
    for (int im = 0; im < muscleSet.getSize(); ++im) {
        auto& muscBase = muscleSet.get(im);

        // pre-emptively create a default DeGrooteFregly2016Muscle
        // (not ideal to do this)
        auto actu = OpenSim::make_unique<DeGrooteFregly2016Muscle>();

        // peform muscle-model-specific mappings or throw exception if muscle
        // not supported
        if (auto musc = dynamic_cast<Millard2012EquilibriumMuscle*>(
                    &muscBase)) {

            // TODO: There is a bug in Millard2012EquilibriumMuscle
            // where the default fiber length is 0.1 by default instead
            // of optimal fiber length.
            // if (!SimTK::isNumericallyEqual(
            //            musc->get_default_fiber_length(), 0.1)) {
            //    actu->set_default_normalized_fiber_length(
            //            musc->get_default_fiber_length() /
            //            musc->get_optimal_fiber_length());
            //}
            // TODO how to set normalized tendon force default?
            actu->set_default_normalized_tendon_force(0.5);
            actu->set_default_activation(musc->get_default_activation());
            actu->set_activation_time_constant(
                    musc->get_activation_time_constant());
            actu->set_deactivation_time_constant(
                    musc->get_deactivation_time_constant());

            // TODO
            actu->set_fiber_damping(0);
            // actu->set_fiber_damping(musc->get_fiber_damping());
            actu->set_tendon_strain_at_one_norm_force(
                    musc->get_TendonForceLengthCurve()
                            .get_strain_at_one_norm_force());

        } else if (auto musc = dynamic_cast<Thelen2003Muscle*>(&muscBase)) {

            // actu->set_default_normalized_fiber_length(
            //        musc->get_default_fiber_length() /
            //        musc->get_optimal_fiber_length());
            // TODO how to set normalized tendon force default?
            actu->set_default_normalized_tendon_force(0.5);
            actu->set_default_activation(musc->getDefaultActivation());
            actu->set_activation_time_constant(
                    musc->get_activation_time_constant());
            actu->set_deactivation_time_constant(
                    musc->get_deactivation_time_constant());

            // Fiber damping needs to be hardcoded at zero since it is not a
            // property of the Thelen2003 muscle.
            actu->set_fiber_damping(0);
            actu->set_tendon_strain_at_one_norm_force(
                    musc->get_FmaxTendonStrain());

        } else {
            OPENSIM_THROW_IF(!allowUnsupportedMuscles, Exception,
                    format("Muscle '%s' of type %s is unsupported and "
                           "allowUnsupportedMuscles=false.",
                            muscBase.getName(),
                            muscBase.getConcreteClassName()));
            continue;
        }

        // Perform all the common mappings at base class level (OpenSim::Muscle)

        actu->setName(muscBase.getName());
        muscBase.setName(muscBase.getName() + "_delete");
        actu->setMinControl(muscBase.getMinControl());
        actu->setMaxControl(muscBase.getMaxControl());

        actu->setMaxIsometricForce(muscBase.getMaxIsometricForce());
        actu->setOptimalFiberLength(muscBase.getOptimalFiberLength());
        actu->setTendonSlackLength(muscBase.getTendonSlackLength());
        actu->setPennationAngleAtOptimalFiberLength(
                muscBase.getPennationAngleAtOptimalFiberLength());
        actu->setMaxContractionVelocity(muscBase.getMaxContractionVelocity());
        actu->set_ignore_tendon_compliance(
                muscBase.get_ignore_tendon_compliance());
        actu->set_ignore_activation_dynamics(
                muscBase.get_ignore_activation_dynamics());

        const auto& pathPointSet = muscBase.getGeometryPath().getPathPointSet();
        auto& geomPath = actu->updGeometryPath();
        for (int ipp = 0; ipp < pathPointSet.getSize(); ++ipp) {
            auto* pathPoint = pathPointSet.get(ipp).clone();
            const auto& socketNames = pathPoint->getSocketNames();
            for (const auto& socketName : socketNames) {
                pathPoint->updSocket(socketName)
                        .connect(pathPointSet.get(ipp)
                                         .getSocket(socketName)
                                         .getConnecteeAsObject());
            }
            geomPath.updPathPointSet().adoptAndAppend(pathPoint);
        }
        std::string actname = actu->getName();
        model.addForce(actu.release());

        // Workaround for a bug in prependComponentPathToConnecteePath().
        for (auto& comp : model.updComponentList()) {
            const auto& socketNames = comp.getSocketNames();
            for (const auto& socketName : socketNames) {
                auto& socket = comp.updSocket(socketName);
                auto connecteePath = socket.getConnecteePath();
                std::string prefix = "/forceset/" + actname;
                if (startsWith(connecteePath, prefix)) {
                    connecteePath = connecteePath.substr(prefix.length());
                    socket.setConnecteePath(connecteePath);
                }
            }
        }
        musclesToDelete.push_back(&muscBase);
    }

    // Delete the muscles.
    for (const auto* musc : musclesToDelete) {
        int index = model.getForceSet().getIndex(musc, 0);
        OPENSIM_THROW_IF(index == -1, Exception,
                format("Muscle with name %s not found in ForceSet.",
                        musc->getName()));
        bool success = model.updForceSet().remove(index);
        OPENSIM_THROW_IF(!success, Exception,
                format("Attempt to remove muscle with "
                       "name %s was unsuccessful.",
                        musc->getName()));
    }

    model.finalizeFromProperties();
    model.finalizeConnections();
}