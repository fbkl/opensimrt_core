/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 */
#include "GRFMPredictionInsole.h"
#include "Exception.h"
#include "GaitPhaseDetector.h"
#include "OpenSimUtils.h"
#include "Utils.h"

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
GRFMPredictionInsole::GRFMPredictionInsole(const Model& otherModel,
                               const Parameters& aParameters,
                               GaitPhaseDetector* detector)
        :  GRFMNonSmooth(otherModel, aParameters.pelvisBodyName) , gaitPhaseDetector(detector),
          parameters(aParameters) {
    cout << "initializing GRFMPredictionInsole" << endl;
	// reserve memory size for computing the mean gait direction
    //TODO: Read from file the Insole Values!!!!!!
    //TODO: Remove Unused stuff!!!
    gaitDirectionBuffer.setSize(parameters.directionWindowSize);

    // add station points to the model for the CoP trajectory
    heelStationR =
            new Station(model.getBodySet().get(parameters.rStationBodyName),
                        parameters.rHeelStationLocation);
    heelStationL =
            new Station(model.getBodySet().get(parameters.lStationBodyName),
                        parameters.lHeelStationLocation);
    toeStationR =
            new Station(model.getBodySet().get(parameters.rStationBodyName),
                        parameters.rToeStationLocation);
    toeStationL =
            new Station(model.getBodySet().get(parameters.lStationBodyName),
                        parameters.lToeStationLocation);
    heelStationR->setName("heel_station_point_r");
    heelStationL->setName("heel_station_point_l");
    toeStationR->setName("toe_station_point_r");
    toeStationL->setName("toe_station_point_l");
    model.addModelComponent(heelStationR.get());
    model.addModelComponent(heelStationL.get());
    model.addModelComponent(toeStationR.get());
    model.addModelComponent(toeStationL.get());

    // disable muscles, otherwise they apply passive forces
    OpenSimUtils::disableActuators(model);

    // initialize system
    state = model.initSystem();

    // define STA functions by Ren et al.
    // https://doi.org/10.1016/j.jbiomech.2008.06.001
    // NOTE: anteriorForceTransition is replaced with
    // reactionComponentTransition due to innacurate results.
    reactionComponentTransition = [&](const double& t) -> double {
        // STA functions depend on class' private member Tds. Clip return values
        // in order to not exceed the range [0,1] in case of inaccurate
        // estimation of Tds.
        return clip(std::exp(-std::pow((2.0 * t / Tds), 3)), 0.0, 1.0);
    };

    // define CoP trajectory (linear transition from heel -> metatarsal)
    // Source: https://doi.org/10.1016/j.jbiomech.2013.09.012
    copPosition = [&](const double& t, const SimTK::Vec3& d) -> SimTK::Vec3 {
        const auto omega = 2.0 * SimTK::Pi / Tss;
        // clip the scale factor if exceeds the range [0,1].
        const auto scale =
                clip(-2.0 / (3 * SimTK::Pi) *
                             (sin(omega * t) - sin(2 * omega * t) / 8 -
                              3.0 / 4.0 * omega * t),
                     0.0, 1.0);
        // scale the distance vector d (heel -> metatarsal)
        return scale * d;
    };
    cout << "GRFMPredictionInsole initialized ok" << endl;
}

SimTK::Rotation
GRFMPredictionInsole::computeGaitDirectionRotation(const std::string& bodyName) {
    const auto& body = model.getBodySet().get(bodyName);
    const auto& mob = model.getMatterSubsystem().getMobilizedBody(
            body.getMobilizedBodyIndex());

    // get body transformation
    const auto& R_GB = mob.getBodyTransform(state).R();

    // append direction to buffer (x-component in rotation matrix)
    gaitDirectionBuffer.insert((~R_GB).col(0).asVec3());

    // compute the average heading direction
    auto gaitDirection = projectionOnPlane(gaitDirectionBuffer.mean(), SimTK::Vec3(0),
                                           SimTK::Vec3(0, 1, 0));

    // rotation about the vertical axis to transform the reaction components
    // from the opensim global reference frame to the gait-direction
    // reference frame
    auto crossProd =
            SimTK::cross(gaitDirection, SimTK::Vec3(1, 0, 0));      // |a|.|b|.sin(q).n
    auto dotProd = SimTK::dot(gaitDirection, SimTK::Vec3(1, 0, 0)); // |a|.|b|.cos(q)
    auto q = std::atan(crossProd.norm() / dotProd);

    return SimTK::Rotation(q, SimTK::Vec3(0, 1, 0));
}

GRFMPredictionInsole::Output
GRFMPredictionInsole::solve(const GRFMPredictionInsole::Input& input) {
    Output output;
    output.t = input.t;
    output.right.force = SimTK::Vec3(0.0);
    output.right.torque = SimTK::Vec3(0.0);
    output.right.point = SimTK::Vec3(0.0);
    output.left.force = SimTK::Vec3(0.0);
    output.left.torque = SimTK::Vec3(0.0);
    output.left.point = SimTK::Vec3(0.0);

    if (gaitPhaseDetector->isDetectorReady()) {
        // update model state and realize state
        OpenSimUtils::updateState(model, state, input.q, input.qDot);
        model.realizeDynamics(state);

        // compute the transformation to the average heading direction
        auto R = computeGaitDirectionRotation(parameters.pelvisBodyName);

        // compute total reaction force/moment
        SimTK::Vec3 totalReactionForce(0), totalReactionMoment(0);
        computeTotalReactionComponents(input, totalReactionForce,
                                       totalReactionMoment);

        // express total reaction loads in heading direction frame
        totalReactionForce = R * totalReactionForce;
        totalReactionMoment = R * totalReactionMoment;
        // totalReactionMoment[0] = 0;
        // totalReactionMoment[2] = 0;

        // time since last HS
        double time = input.t - gaitPhaseDetector->getHeelStrikeTime();
        if (time == 0.0) {
            totalForceAtThs = totalReactionForce;
            totalMomentAtThs = totalReactionMoment;
        }

        // previous double-support time period
        Tds = gaitPhaseDetector->getDoubleSupportDuration();

        // forces
        SimTK::Vec3 rightReactionForce, leftReactionForce;
        separateReactionComponents(time, totalReactionForce, totalForceAtThs,
                                   reactionComponentTransition,
                                   reactionComponentTransition,
                                   reactionComponentTransition,
                                   rightReactionForce, leftReactionForce);

        // moments
        SimTK::Vec3 rightReactionMoment, leftReactionMoment;
        separateReactionComponents(time, totalReactionMoment, totalMomentAtThs,
                                   reactionComponentTransition,
                                   reactionComponentTransition,
                                   reactionComponentTransition,
                                   rightReactionMoment, leftReactionMoment);

        // cop
        SimTK::Vec3 rightPoint, leftPoint;
        computeReactionPoint(input.t, rightPoint, leftPoint);

        // results
        output.right.force = rightReactionForce;
        output.right.torque = rightReactionMoment;
        output.right.point = rightPoint;
        output.left.force = leftReactionForce;
        output.left.torque = leftReactionMoment;
        output.left.point = leftPoint;
    }
    return output;
}

void GRFMPredictionInsole::separateReactionComponents(
        const double& time, const SimTK::Vec3& totalReactionComponent,
        const SimTK::Vec3& totalReactionAtThs,
        const TransitionFuction& anteriorComponentFunction,
        const TransitionFuction& verticalComponentFunction,
        const TransitionFuction& lateralComponentFunction,
        SimTK::Vec3& rightReactionComponent, SimTK::Vec3& leftReactionComponent) {
    switch (gaitPhaseDetector->getPhase()) {
    case GaitPhaseState::GaitPhase::DOUBLE_SUPPORT: {
        // compute the trailing and leading leg reaction components
        SimTK::Vec3 trailingReactionComponent, leadingReactionComponent;

        // trailing leg component
        trailingReactionComponent[0] =
                totalReactionAtThs[0] * anteriorComponentFunction(time);
        trailingReactionComponent[1] =
                totalReactionAtThs[1] * verticalComponentFunction(time);
        trailingReactionComponent[2] =
                totalReactionAtThs[2] * lateralComponentFunction(time);

        // leading leg component
        leadingReactionComponent =
                totalReactionComponent - trailingReactionComponent;

        // assign to output based on the current leading/trailing leg
        switch (gaitPhaseDetector->getLeadingLeg()) {
        case GaitPhaseState::LeadingLeg::RIGHT: {
            rightReactionComponent = leadingReactionComponent;
            leftReactionComponent = trailingReactionComponent;
        } break;
        case GaitPhaseState::LeadingLeg::LEFT: {
            rightReactionComponent = trailingReactionComponent;
            leftReactionComponent = leadingReactionComponent;
        } break;
        case GaitPhaseState::LeadingLeg::INVALID: {
            cerr << "STA: invalid LeadingLeg state!" << endl;
        } break;
        }
    } break;

    case GaitPhaseState::GaitPhase::LEFT_SWING: {
        rightReactionComponent = totalReactionComponent;
        leftReactionComponent = SimTK::Vec3(0);
    } break;

    case GaitPhaseState::GaitPhase::RIGHT_SWING: {
        rightReactionComponent = SimTK::Vec3(0);
        leftReactionComponent = totalReactionComponent;
    } break;

    default: {
        rightReactionComponent = SimTK::Vec3(0);
        leftReactionComponent = SimTK::Vec3(0);
    } break;
    }
}

void GRFMPredictionInsole::computeReactionPoint(const double& t,
                                          SimTK::Vec3& rightPoint,
                                          SimTK::Vec3& leftPoint) {
    // get previous SS time-period
    Tss = gaitPhaseDetector->getSingleSupportDuration();

    // determine gait phase
    switch (gaitPhaseDetector->getPhase()) {
    case GaitPhaseState::GaitPhase::DOUBLE_SUPPORT: {
        // first determine leading / trailing leg
        switch (gaitPhaseDetector->getLeadingLeg()) {
        case GaitPhaseState::LeadingLeg::RIGHT: {
            rightPoint = heelStationR->getLocationInGround(state);
            leftPoint = toeStationL->getLocationInGround(state);
        } break;
        case GaitPhaseState::LeadingLeg::LEFT: {
            rightPoint = toeStationR->getLocationInGround(state);
            leftPoint = heelStationL->getLocationInGround(state);
        } break;
        case GaitPhaseState::LeadingLeg::INVALID: {
            cerr << "CoP: invalid LeadingLeg state!" << endl;
        } break;
        }
    } break;

    case GaitPhaseState::GaitPhase::LEFT_SWING: {
        // distance between heel and toe station points on foot
        const auto d = toeStationR->getLocationInGround(state) -
                       heelStationR->getLocationInGround(state);

        // time since last toe-off event
        auto time = t - gaitPhaseDetector->getToeOffTime();

        // result CoP
        leftPoint = SimTK::Vec3(0);
        rightPoint =
                heelStationR->getLocationInGround(state) + copPosition(time, d);
    } break;

    case GaitPhaseState::GaitPhase::RIGHT_SWING: {
        // distance between heel and toe station points on foot
        const auto d = toeStationL->getLocationInGround(state) -
                       heelStationL->getLocationInGround(state);

        // time since last toe-off event
        auto time = t - gaitPhaseDetector->getToeOffTime();

        // result CoP
        rightPoint = SimTK::Vec3(0);
        leftPoint =
                heelStationL->getLocationInGround(state) + copPosition(time, d);
    } break;

    default: {
        rightPoint = SimTK::Vec3(0.0);
        leftPoint = SimTK::Vec3(0.0);
    } break;
    }
}

void GRFMPredictionInsole::getInsoleMeasurements()
{
	cout << "Not implemnted!" << endl;

}