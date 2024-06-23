/*
 * RobotKinematics.cpp
 *
 *  Created on: 06 Aug 2023
 *      Author: JoergS5
 */


#include <Movement/Kinematics/RobotKinematics.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Platform/RepRap.h>
#include <Movement/DDA.h>

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(RobotKinematics, __VA_ARGS__)

constexpr ObjectModelTableEntry RobotKinematics::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. kinematics members
	{ "name",	OBJECT_MODEL_FUNC(self->GetName(true)), 	ObjectModelEntryFlags::none },
	//TODO lots more to be added here
};

constexpr uint8_t RobotKinematics::objectModelTableDescriptor[] = { 1, 1 };

DEFINE_GET_OBJECT_MODEL_TABLE_WITH_PARENT(RobotKinematics, ZLeadscrewKinematics)

#endif


RobotKinematics::RobotKinematics() noexcept
	: ZLeadscrewKinematics(KinematicsType::scara, SegmentationType(true, false, false))
{
}

RobotKinematics::~RobotKinematics() {
}

const char *RobotKinematics::GetName(bool forStatusReport) const noexcept {
	return "robot";
}

bool RobotKinematics::Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply,
		bool& error) THROWS(GCodeException) {

	if (mCode == 669)
	{
		bool seen = false;
		bool allowEmpty = false;
		bool letterUsed = false;

		if (gb.Seen('B')) {
			gb.TryGetQuotedString('B', tempS50.GetRef(), seen, allowEmpty);
			setB(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('A')) {
			gb.TryGetQuotedString('A', tempS50.GetRef(), seen, allowEmpty);
			setA(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('P')) {
			gb.TryGetQuotedString('P', tempS50.GetRef(), seen, allowEmpty);
			setP(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('C')) {
			gb.TryGetQuotedString('C', tempS50.GetRef(), seen, allowEmpty);
			setC(tempS50.c_str());
			seen = true; // => unhome
			letterUsed = true;
		}

		if (gb.Seen('R')) {
			float avgTime = sumOfTimes / (float) timeMeasurements;
			tempS50.Clear();
			tempS50.catf("avg Time inverse kin: %.2f microseconds", (double) avgTime);
			consoleMessage(tempS50.GetRef());
			tempS50.copy("\n");
			consoleMessage(tempS50.GetRef());
			sumOfTimes = 0;
			timeMeasurements = 0;
			letterUsed = true;
		}

		// set not to unhome
		if (gb.Seen('D')) {
			seen = false; // don't unhome
			letterUsed = true;
		}

		if(!letterUsed) {
			reportConfiguration(gb);
		}

		return seen;
	}
	else
	{
		return ZLeadscrewKinematics::Configure(mCode, gb, reply, error);
	}
}

bool RobotKinematics::CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept {


	if(specialMethod == 1) { // CoreXY
		int i = -1;
		bool isAC = true;
		if(abcType == 0) {
			i = getPositionOfLetterInChain('A');
			isAC = true;
		}
		else if(abcType == 1) {
			i = getPositionOfLetterInChain('B');
			isAC = false;
		}
		const float ABpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};
		i = getPositionOfLetterInChain('C');
		const float Cpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};

		float result[5];
		GACartesianToMotor(machinePos, ABpt, Cpt, result, isAC);

		float coreX = result[0];
		float coreY = result[1];
		if(specialMethod == 1) { // CoreXY
			coreX = result[0] + result[1];
			coreY = result[0] - result[1];
		}
		motorPos[0] = lrintf(coreX * stepsPerMm[0]); // X (A stepper)
		motorPos[1] = lrintf(coreY * stepsPerMm[1]); // Y (B stepper)
		motorPos[2] = lrintf(result[2] * stepsPerMm[2]); // Z
		motorPos[3] = lrintf(machinePos[3] * stepsPerMm[3]); // A/B
		motorPos[4] = lrintf(machinePos[4] * stepsPerMm[4]); // C
	}

	for (size_t axis = numOfAxes; axis < numVisibleAxes; ++axis)	{
		motorPos[axis] = lrintf(machinePos[axis] * stepsPerMm[axis]);
	}


	return true;
}

//called after connection established, called once for every segment move:

void RobotKinematics::MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[],
		size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept {
	startClock();

	if(specialMethod == 1) { // CoreXY
		float angles[numOfAxes];
		if(specialMethod == 1) { // CoreXY
			angles[0] = 0.5*((float) motorPos[0] / stepsPerMm[0] + (float) motorPos[1] / stepsPerMm[1]);
			angles[1] = 0.5*((float) motorPos[0] / stepsPerMm[0] - (float) motorPos[1] / stepsPerMm[1]);
		}
		else {
			angles[0] = (float) motorPos[0] / stepsPerMm[0]; // X
			angles[1] = (float) motorPos[1] / stepsPerMm[1]; // Y
		}
		angles[2] = (float) motorPos[2] / stepsPerMm[2]; // Z
		angles[3] = (float) motorPos[3] / stepsPerMm[3]; // A or B
		angles[4] = (float) motorPos[4] / stepsPerMm[4]; // C

		int i = -1;
		bool isAC = true;
		if(abcType == 0) {
			i = getPositionOfLetterInChain('A');
			isAC = true;
		}
		else if(abcType == 1) {
			i = getPositionOfLetterInChain('B');
			isAC = false;
		}
		const float ABpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};
		i = getPositionOfLetterInChain('C');
		const float Cpt[3] = { cache[i*3+ offsetScrewQ], cache[i*3+ offsetScrewQ+1], cache[i*3+ offsetScrewQ+2]};

		GAMotorToCartesian(angles, ABpt, Cpt, machinePos, isAC);
	}

	sumOfTimes += stopClock();
	timeMeasurements++;

}

bool RobotKinematics::IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept {
	return true;
}

LimitPositionResult RobotKinematics::LimitPosition(float coords[], const float * null initialCoords,
		size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept {

//	const bool m208Limited = applyM208Limits && Kinematics::LimitPositionFromAxis(coords, 0, numVisibleAxes, axesToLimit);
//	return (m208Limited) ? LimitPositionResult::adjusted : LimitPositionResult::ok;

	return LimitPositionResult::ok;
}

void RobotKinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept {

}

AxesBitmap RobotKinematics::AxesAssumedHomed(AxesBitmap g92Axes) const noexcept {
	return g92Axes;
}

AxesBitmap RobotKinematics::MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept {
	return axesMoving;
}


AxesBitmap RobotKinematics::GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed,
		size_t numVisibleAxes, const StringRef& filename) const noexcept {
	AxesBitmap ret = Kinematics::GetHomingFileName(toBeHomed, alreadyHomed, numVisibleAxes, filename);
	filename.copy(HomeRobotFileName);
	return ret;

}

bool RobotKinematics::QueryTerminateHomingMove(size_t axis) const noexcept {
	return false;
}

void RobotKinematics::OnHomingSwitchTriggered(size_t axis, bool highEnd,
		const float stepsPerMm[], DDA& dda) const noexcept {

	char letter = forwardChainCompressed[axis];
	int chainOrder = getPositionOfLetterInChain(letter);
	if(chainOrder >= 0) { // A parameter for letter found
		float home = cache[offsetAngleLimits + chainOrder*3 + 2];
		dda.SetDriveCoordinate(lrintf(home * stepsPerMm[axis]), axis);
	}
	else { // try M208
		const float hitPoint = ((highEnd) ? reprap.GetPlatform().AxisMaximum(axis) :
				reprap.GetPlatform().AxisMinimum(axis));
		dda.SetDriveCoordinate(lrintf(hitPoint * stepsPerMm[axis]), axis);
	}
}

bool RobotKinematics::IsContinuousRotationAxis(size_t axis) const noexcept {
	return false;
}

AxesBitmap RobotKinematics::GetLinearAxes() const noexcept {
	return AxesBitmap::MakeFromBits(Z_AXIS);
}

AxesBitmap RobotKinematics::GetConnectedAxes(size_t axis) const noexcept {
	return (axis == X_AXIS || axis == Y_AXIS)
			? XyAxes
				: AxesBitmap::MakeFromBits(axis);

}

void RobotKinematics::LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector,
		size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept {

}
