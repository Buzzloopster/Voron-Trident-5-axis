/*
 * RobotKinematics.h
 *
 *  Created on: 06 Aug 2023
 *      Author: JoergS5
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ROBOTKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ROBOTKINEMATICS_H_

#include "ZLeadscrewKinematics.h"

#if SUPPORT_ROBOT

#include <Movement/StepTimer.h> // for GetTimerTicks() of startClock(), stopClock() methods
#include <Platform/Tasks.h>

class RobotKinematics : public ZLeadscrewKinematics {
public:
	RobotKinematics() noexcept;
	virtual ~RobotKinematics();

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;
	LimitPositionResult LimitPosition(float coords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	AxesBitmap GetLinearAxes() const noexcept override;
	AxesBitmap GetConnectedAxes(size_t axis) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }

	void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector,
			size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;


	const static size_t MAXNUMOFAXES = 6;
	const static size_t CACHESIZE = 200;
	const static size_t FORWPROC = 10;
	const static size_t INVPROC = 10;

	static constexpr const char *HomeRobotFileName = "homeRobot.g";
	const float radiansToDegrees = 57.295784f; // exact float representation


	// temp variables, to be used only inside a method:
	mutable String<StringLength20> tempS20; // to read values from strings
	mutable String<StringLength50> tempS50; // to read values from strings
	mutable float mxTemp[12];	// for short temp storage
	mutable float temp4[4];


	// robot and axis information
	mutable char forwardChain[20]; // subkinematics type, eg CAZ_corexy(XY)
	mutable char forwardChainCompressed[MAXNUMOFAXES+1]; // letter of normal part, .: placeholder special handling
	mutable char forwardChainSpecial[4+1];		// letter of special kinematics part
	mutable char axisTypes[MAXNUMOFAXES+1]; // in order of chain: R rotational P prismatic p palletized
	mutable int numOfAxes = 5; // will be set by B or PaxisTypes
	mutable float screw_M[12];
	mutable float screw_MInv[12];

	// special kinematics settings
	mutable int specialMethod = 0;	// 1 CoreXY 2 CoreXZ 9 5barParScara 10 RotaryDelta 14 Palletized 15 LinearDelta
	mutable size_t currentWorkmode = 0; // for 5 bar scara
	mutable int abSign = 0; // for 5 axis. 0=A,B positive angle, 1=negative for AC/BC systems, 2 = don't change
	mutable int abcType = 0; // for 5 axis: 0=AC, 1=BC, other types can be added later

	mutable size_t forwardProc[FORWPROC]; // forward processor
	mutable size_t inverseProc[INVPROC]; // inverse processor
	mutable float cache[CACHESIZE];
	mutable int offsetScrewOmega = -1; // axis orientation *3
	mutable int offsetScrewQ = -1; // point on axis  *3
	mutable int offsetScrewOmega2 = -1; // only for R axes *12 // omega*omegaT
	mutable int offsetScrewV = -1; // only for R axes *3 // cross product V = - omega x Q
	mutable int offsetMreference = -1; // size numOfAxes *1  // angles/positions used for Mnoeap endpoint pos/ori
	mutable int offsetAngleLimits = -1; // size numOfAxes*3 // min, max, home. -999.9 for NaN
	mutable int offsetStartOfInverse = -1;		// where does inverse (Paden-Kahan) used cache start
	mutable int offsetCurrentEnd = -1; // point to end of currently used cache


	// Config
	void setB(const char* _robottype) const noexcept;
	void setC(const char *value) const noexcept;
	void setA(const char *value) const noexcept;
	void setP(const char *value) const noexcept;
	void reportConfiguration(GCodeBuffer& gb) const noexcept;


	void setNumOfAxes() const noexcept;
	void setForwardChain(const char* value) const noexcept;
	void checkAndSetSpecial(const char *checkfor, int specialnr) const noexcept;
	void addNormalLetters(int start, int stop,int &idx) const noexcept;
	int extractSpecial(int start, int &idx) const noexcept;

	int getValueCount(const char *value, const char *marker, char separator, int &startpos) const noexcept;
	float getFloatOfElement(const char *value, char separator, int startpos, int idx) const noexcept;
	int getPositionOfLetterInChain(char letter) const noexcept;
	char getLetterInChain(int pos) const noexcept;

	void multiplyTrmatrixWithVector(const float *mx, const float *vec, float *vecTo) const noexcept;

	void initCache() const noexcept;
	void setAxisTypes(const char* types) const noexcept;
	void setSkew_M(float *values) const noexcept;
	void recalcMInv() const noexcept;


	// core methods helpers
	void anglesToMotorPos(const float *angles, const float *stepsPerMm, int32_t *motorPos) const noexcept;
	void motorPosToAngles(const int32_t *motorPos, const float *stepsPerMm, float *angles) const noexcept;
	void getMaxVelocity(float *maxAngleChanges) const noexcept;


	void multiplyMatrix(int m, int r, const float *mx1, int n, const float *mx2, float *mxTo)  const noexcept;
	void multiplyRotationMatrix(const float *m1, const float *m2, float *result) const noexcept;
	void rotationMatrixInverse(const float *mx, float *mxInverse) const noexcept;
	void multiplyRotationMatrixInplaceLeft(float *m1, const float *m2) const noexcept;


	// performance measure
	mutable uint32_t starttime = (uint32_t) 0;
	void startClock() const noexcept;
	float stopClock() const noexcept;
	void consoleMessage(const StringRef &msg) const noexcept;
	void errorMessage(const StringRef &msg) const noexcept;
	mutable float sumOfTimes = 0.0; // microseconds
	mutable int timeMeasurements = 0; // sumOfTimes/timeMeasurements give the average



	void debugMatrix(const char *title, const float*mx) const noexcept;
	void debugList(const char *title, int size, const float *list) const noexcept;
	void debugList(const char *title, int size, int32_t *list) const noexcept;



	void getRodrigues(float *screwO, float *screwQ, float theta, float *mx, bool isRotational) const noexcept;
	void setRodrigues1(float *screwO, float *screwQ, float *omega2, float *v) const noexcept;
	void getRodrigues2_Rot(const float *screwO, const float *screwQ, const float *omega2, const float *v, float theta,
			float *mx) const noexcept;
	void getRodrigues2_Pris(float *screwOmega, float dist, float *_mxTemp) const noexcept;
	int getRotaryIndex(int axisnr) const noexcept;
	void getRodriguesByLetter(char drive, float theta, float *mx) const noexcept;
	void getRodriguesByLetterInv(char drive, float theta, float *mxInv) const noexcept;
	void removeMatrixNearZero(float *mxTemp) const noexcept;

	// Paden-Kahan
//	mutable float pk1cache[1][12]; // 1 * 12 * 4 bytes
//	mutable float pk2cache[2][33]; // 2 * 33 * 4 bytes
//	mutable float pk3cache[1][12]; // 2 * 12 * 4 bytes
/*
	void getPadenKahan1_1(int idx, const float *om, const float *screwV) const noexcept;
	int getPadenKahan1_2(int idx, const float *om, const float *p, const float *k, float &theta) const noexcept;
	void getPadenKahan2_1(int idx, const float *q1, const float *v1, const float *om1,
			const float *q2, const float *v2, const float *om2) const noexcept;
	int getPadenKahan2_2(int idx, const float *om1, const float *om2,
			const float *p, const float *k, float *thetas12) const noexcept;
	void getPadenKahan3_1(int idx, const float *om, const float *screwV) const noexcept;
	int getPadenKahan3_2(int idx, const float *om, const float *p, const float *k, float delta,
			float *thetas) const noexcept;
	void multiplyRotationMatrixWithPoint(const float *mx, float *point, float *resultPoint) const noexcept;
	void multiplyRotationMatrixWithPointInplaceRight(const float *mx, float *point) const noexcept;
	void multiplyRotmatRotmatPoint(const float *mx1, const float *mx2, float *point, float *resultPoint) const noexcept;
	int getAxisIntersection(const float *omega1, const float *screwQ1, const float *omega2,
			const float *screwQ2, float *r) const noexcept;
	float getNorm(const float *v) const noexcept;
	void getPK_projection_fromCache(const float *cacheWithIdx, int cacheOffset, const float *u, float *up) const noexcept;
	bool getPK_intersection_planes_toLine(const float *om1, const float *q1, const float *p1,
			const float *om2, const float *q2, const float *p2, float *line, float *point) const noexcept;
	int getPK_intersection_circle_line_toPoints(const float *om, const float *axispoint, float radius, const float *line,
			const float *linepoint, float *resultPoints[2][3]) const noexcept;
	void getCrossProduct(const float *v1, const float *v2, float *v3) const noexcept;
	void getCrossProductMin(const float *v1, const float *v2, float *v3) const noexcept;
	void multiplyVectorsTNT(const float *v1,const float *v2,const float *v3, float *result) const noexcept;
	void multiplyVectorsTN(const float *v1,const float *v2, float *result) const noexcept;
	void multiplyVectorsNT(const float *v1,const float *v2, float *result) const noexcept;
*/
	void normalizeVector(float &x, float &y, float &z) const noexcept;
	// GA related:

	// the values used by the objects and rotators
/*
	size_t gaObjectPattern[20]; // 0 MV, 1 point, 2 vector, ... Access through the index of CGAType

	// idx: 0...31 position, 32 means 0. patt: char pattern of e123info for binary calculations.
	char idxToPatt[33] = {0,1,2,4,8,16,3,5,9,17,6,10,18,12,20,24,7,11,19,13,21,25,14,22,26,28,15,23,27,29,30,31,32};
	char pattToIdx[33] = {0,1,2,6,3,7,10,16,4,8,11,17,13,19,22,26,5,9,12,18,14,20,23,27,15,21,24,28,25,29,30,31,32};

	const char * pattToName(char pattern) const noexcept;
	char nameToPatt(const char *name) const noexcept;


	void getWedgeMV(const size_t a32, const float *valuesA, const size_t b32, const float *valuesB,
			size_t &result32) const noexcept; // result is stored in result32

	void getInner(char a, char b, bool &plusI, char &inner) const noexcept;
	void getGeom(char a, char b, bool &plusG1, bool &plusG2, char &geom1, char &geom2) const noexcept;

	inline void getWedge(const char a, const char b, bool &plusW, char &wedge) const noexcept {
		plusW = true;
		if(a == 32 || b == 32) wedge = 32; // one is 0
		else if(a == 0) wedge = b;  // scalar. Includes case that both are scalar
		else if(b == 0) wedge = a; // scalar
		else if((a & b) != 0) wedge = 32; // blade part shared, result is 0
		else {
			wedge = a | b;
			for(int i=0; i < 5; i++) { // calculate sign
				if(((b >> i) & 1U) != 0) { // b has bit set
					int ct = 0;
					for(int j=i+1; j < 5; j++) { // count in a which bits must be permutated
						if(((a >> j) & 1U) != 0) ct++;
					}
					if(ct % 2 == 1) plusW = !plusW;
				}
			}
		}
	}
*/
	// forward, inverse by screw
	void getForwardBySkew(const float *angles, float *mxTo) const noexcept;

	void XYZACTomx(const float *xyzac, float *mx) const noexcept;
	void XYZBCTomx(const float *xyzbc, float *mx) const noexcept;
	void mxToXYZAC(const float *mx, float *xyzac, float cAngle) const noexcept;
	void mxToXYZBC(const float *mx, float *xyzbc, float cAngle) const noexcept;

	void getInverseBySkew(const float *mxTo, float *anglesResult, float cAngle) const noexcept;
	void getInverseAC(const float *mxTo, float *anglesResult, float cAngle) const noexcept;
	void getInverseCoreXY_XYZ(const float *mxTo, float *anglesResult, bool iscorexy) const noexcept;
	void initNeutralMatrix(float *mx) const noexcept;

	// GA methods
	void GACartesianToMotor(const float *cartXYZAC, const float *ABpt, const float *Cpt, float *motor_xyz,
			bool isAC) const noexcept;
	void GAMotorToCartesian(const float *motor_xyzac, const float *ABpt, const float *Cpt,
			float *cartXYZAC, bool isAC) const noexcept;
	void GAcalculateRotorInplace(const float *rotor, float *point) const noexcept;


protected:
	DECLARE_OBJECT_MODEL

};

#endif // SUPPORT_ROBOT



#endif /* SRC_MOVEMENT_KINEMATICS_ROBOTKINEMATICS_H_ */
