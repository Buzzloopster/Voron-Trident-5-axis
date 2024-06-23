/*
 * RobotKinematics2.cpp
 *
 *  Created on: 06 Aug 2023
 *      Author: JoergS5
 */

#include <Movement/Kinematics/RobotKinematics.h>

#if SUPPORT_ROBOT

/*
 * angles is in degrees (rotary axis) or mm (linear axis)
 * angles are sorted by chain, i.e. CAZXY
 */

void RobotKinematics::getForwardBySkew(const float *angles, float *mxTo) const noexcept {
	for(int i=0; i < numOfAxes; i++) {
		if(axisTypes[i] == 'R') {
			int rotarynr = getRotaryIndex(i);
			getRodrigues2_Rot(&cache[i*3 + offsetScrewOmega], &cache[i*3+ offsetScrewQ],
					&cache[rotarynr*12 + offsetScrewOmega2], &cache[rotarynr*3 + offsetScrewV],
					(angles[i] - cache[offsetMreference+i]) / radiansToDegrees, mxTemp);
		}
		else {
			getRodrigues2_Pris(&cache[i*3+ offsetScrewOmega], angles[i] - cache[offsetMreference+i], mxTemp);
		}
		if(i == 0) {
			memcpy(mxTo, mxTemp, sizeof(float) * 12);
		}
		else {
			multiplyRotationMatrixInplaceLeft(mxTo, mxTemp);
		}
	}
	multiplyRotationMatrixInplaceLeft(mxTo, screw_M);
}

void RobotKinematics::getRodrigues2_Pris(float *screwOmega, float dist, float *_mxTemp) const noexcept {
	_mxTemp[0] = 1.0;
	_mxTemp[4] = 0.0;
	_mxTemp[8] = 0.0;
	_mxTemp[1] = 0.0;
	_mxTemp[5] = 1.0;
	_mxTemp[9] = 0.0;
	_mxTemp[2] = 0.0;
	_mxTemp[6] = 0.0;
	_mxTemp[10] = 1.0;

	_mxTemp[3] = screwOmega[0] * dist;
	_mxTemp[7] = screwOmega[1] * dist;
	_mxTemp[11] = screwOmega[2] * dist;

}

/*
 * create transformation matrix based on exponential screw/skew values
 * version without using cache
 *
 * \param screw 6 parameters: 3 skew, 3 q positions on axis
 *
 * \param theta in radians
 *
 * \param T resulting transformation matrix 3x4
 */

void RobotKinematics::getRodrigues(float *screwO, float *screwQ, float theta,
		float *mx, bool isRotational) const noexcept {
	if(isRotational) {
		float v[3], omega2[9];
		setRodrigues1(screwO, screwQ, omega2, v);
		getRodrigues2_Rot(screwO, screwQ, omega2, v, theta, mx);
	}
	else {
		getRodrigues2_Pris(screwO, theta, mx);
	}
}

/*
 * prepare Rodrigues part 1, the parts which do not depend on theta
 *
 * the result is cached:
 * v is the cross product of w and q
 * omega2 is omega squared, a 3x3 matrix
 */

void RobotKinematics::setRodrigues1(float *screwO, float *screwQ, float *omega2, float *v) const noexcept {
	// v = - w x q
	v[0] = -screwO[1]*screwQ[2] + screwO[2]*screwQ[1];
	v[1] = -screwO[2]*screwQ[0] + screwO[0]*screwQ[2];
	v[2] = -screwO[0]*screwQ[1] + screwO[1]*screwQ[0];

	float omega[9];
	omega[0] = 0.0;
	omega[4] = 0.0;
	omega[8] = 0.0;

	omega[1] = - screwO[2];
	omega[2] = screwO[1];
	omega[3] = screwO[2];
	omega[5] = - screwO[0];
	omega[6] = - screwO[1];
	omega[7] = screwO[0];

	multiplyMatrix(3, 3, omega, 3, omega, omega2);
}

/*
 * Rodrigues part 2, calculating T from known properties and theta
 */

void RobotKinematics::getRodrigues2_Rot(const float *screwO, const float *screwQ_notused, const float *omega2,
		const float *v, float theta, float *mx) const noexcept {
	float sinTheta = 0.0, oneMinCos = 0.0, thetaMinSinTheta = 0.0;
	if(theta != 0.0) {
		sinTheta = sinf(theta);
		oneMinCos = 1.0f - cosf(theta);
		thetaMinSinTheta = theta - sinTheta;
	}

	// calculate rotations:
	mx[0] = 1.0f + oneMinCos * omega2[0];
	mx[1] = sinTheta * (- screwO[2]) + oneMinCos * omega2[1];
	mx[2] = sinTheta * screwO[1] + oneMinCos * omega2[2];

	mx[4] = sinTheta * screwO[2] + oneMinCos * omega2[3];
	mx[5] = 1.0f + oneMinCos * omega2[4];
	mx[6] = sinTheta * (- screwO[0]) + oneMinCos * omega2[5];

	mx[8] = sinTheta * (- screwO[1]) + oneMinCos * omega2[6];
	mx[9] = sinTheta * screwO[0] + oneMinCos * omega2[7];
	mx[10] = 1.0f + oneMinCos * omega2[8];

	// calculate positions: Lynch/Park page 105, alternative method see Murray/Li/Sastry page 42
	// Pardos-Gotor page 42
	mx[3] = (thetaMinSinTheta * omega2[0] + theta) * v[0] +
			(oneMinCos * (- screwO[2]) + thetaMinSinTheta * omega2[1]) *  v[1] +
			(oneMinCos * screwO[1] + thetaMinSinTheta * omega2[2])  * v[2];
	mx[7] = (oneMinCos * screwO[2] + thetaMinSinTheta * omega2[3]) * v[0] +
			(thetaMinSinTheta * omega2[4] + theta) *  v[1] +
			(oneMinCos * (- screwO[0]) + thetaMinSinTheta * omega2[5])  * v[2];
	mx[11] = (oneMinCos * (- screwO[1]) + thetaMinSinTheta * omega2[6]) * v[0] +
			(oneMinCos * screwO[0] + thetaMinSinTheta * omega2[7]) *  v[1] +
			(thetaMinSinTheta * omega2[8] + theta)  * v[2];
}

/* store Mnoap and its inverse
 *
 * Mnaop is the endpoint for given actuator angles/positions and default tool length/orientation
 *
 * \param values: x-axis, y-axis, z-axis, pos values as 3-value vectors each, in this order
 *
 */

void RobotKinematics::setSkew_M(float *values) const noexcept {
	screw_M[0] = values[0];
	screw_M[4] = values[1];
	screw_M[8] = values[2];

	screw_M[1] = values[3];
	screw_M[5] = values[4];
	screw_M[9] = values[5];

	screw_M[2] = values[6];
	screw_M[6] = values[7];
	screw_M[10] = values[8];

	screw_M[3] = values[9];
	screw_M[7] = values[10];
	screw_M[11] = values[11];

	normalizeVector(screw_M[0], screw_M[4], screw_M[8]);
	normalizeVector(screw_M[1], screw_M[5], screw_M[9]);
	normalizeVector(screw_M[2], screw_M[6], screw_M[10]);

	recalcMInv();
}

void RobotKinematics::recalcMInv() const noexcept {
	//MT
	screw_MInv[0] = screw_M[0];	screw_MInv[4] = screw_M[1];	screw_MInv[8] = screw_M[2];
	screw_MInv[1] = screw_M[4];	screw_MInv[5] = screw_M[5];	screw_MInv[9] = screw_M[6];
	screw_MInv[2] = screw_M[8];	screw_MInv[6] = screw_M[9];	screw_MInv[10] = screw_M[10];
	//-MT*t
	screw_MInv[3] = -(screw_MInv[0]*screw_M[3] + screw_MInv[1]*screw_M[7] + screw_MInv[2]*screw_M[11]);
	screw_MInv[7] = -(screw_MInv[4]*screw_M[3] + screw_MInv[5]*screw_M[7] + screw_MInv[6]*screw_M[11]);
	screw_MInv[11] = -(screw_MInv[8]*screw_M[3] + screw_MInv[9]*screw_M[7] + screw_MInv[10]*screw_M[11]);
}

int RobotKinematics::getRotaryIndex(int axisnr) const noexcept {
	int ct = 0;
	for(int i=0; i < numOfAxes; i++) {
		if(axisTypes[i] == 'R') {
			if(i == axisnr) {
				return ct;
			}
			ct++;
		}
	}
	return -1; // not found or axisnr wrong
}

/*
 * multiply two matrices
 *
 * \param m, r, n rows first, cols first (== rows second), cols second
 * \param mx1 first matrix in format m x r
 * \param mx2 second matrix in format r x n
 * \param mxTo result in format m x n
 *
 * cost: m*n*r(A,M) (m*n*r additions and m*n*r multiplications), T would be for trigonometric
 */

void RobotKinematics::multiplyMatrix(int m, int r, const float *mx1, int n, const float *mx2, float *mxTo) const noexcept {
	for(int row = 0; row < m; row++) {
		for(int col = 0; col < n; col++) {
			mxTo[row * n + col] = 0.0f;
			for(int e = 0; e < r; e++) {
				mxTo[row * n + col] += mx1[row * r + e] * mx2[e * n + col];
			}
		}
	}
}

/*
 * multiply transformation matrix 3x4 inplace by using a temp array storing the original rows of m1
 * faster than multiplyRotationMatrix, because no memcpy and temp uses 4 instead of 12 floats
 * result is in m1
 *
 * \param m1 first 3x4 matrix which will contain the multiplication result
 *
 * \param m2 second 3x4 matrix
 *
 * \param temp4 temporary 4 element array to hold original row values of m1 which are overwritten
 *
 * unit tested
 */

void RobotKinematics::multiplyRotationMatrixInplaceLeft(float *m1, const float *m2) const noexcept {
	for(int row = 0; row < 3; row++) {
		for(int col=0; col < 4; col++) { // store current row of m1
			temp4[col] = m1[row * 4 + col];
		} // don't merge the two for loops
		for(int col = 0; col < 4; col++) {
			m1[row * 4 + col] = temp4[0]*m2[col] + temp4[1]*m2[4+col] + temp4[2]*m2[8+col];
			if(col == 3) { // add fourth line last column of (0 0 0 1)
				m1[row * 4 + col] += temp4[3];
			}
		}
	}
}

void RobotKinematics::rotationMatrixInverse(const float *mx, float *mxInverse) const noexcept {
	//MT
	mxInverse[0] = mx[0];	mxInverse[4] = mx[1];	mxInverse[8] = mx[2];
	mxInverse[1] = mx[4];	mxInverse[5] = mx[5];	mxInverse[9] = mx[6];
	mxInverse[2] = mx[8];	mxInverse[6] = mx[9];	mxInverse[10] = mx[10];
	//-MT*t
	mxInverse[3] =  -(mxInverse[0]*mx[3] + mxInverse[1]*mx[7] + mxInverse[2]*mx[11]);
	mxInverse[7] =  -(mxInverse[4]*mx[3] + mxInverse[5]*mx[7] + mxInverse[6]*mx[11]);
	mxInverse[11] = -(mxInverse[8]*mx[3] + mxInverse[9]*mx[7] + mxInverse[10]*mx[11]);
}

/*
 * multiply two rotation matrices. memory efficient by sparing fourth line and assume last line to be (0 0 0 1)
 * matrices have 3 rows, 4 cols each
 *
 * m1, m2 and result must exist and be 3x4 each
 *
 * \param m1, m2 matrices
 *
 * \param result resulting 3x4 matrix (in reality 4x4 matrix)
 */

void RobotKinematics::multiplyRotationMatrix(const float *m1, const float *m2, float *result) const noexcept {
	for(int row = 0; row < 3; row++) { // last row not calculated, is always 0 0 0 1
		for(int col = 0; col < 4; col++) {
			result[row*4 + col] = 0.0f;
			for(int e=0; e < 3; e++) { // last element handled special (0 0 0 1)
				result[row*4 + col] += m1[row*4 + e] * m2[e*4 + col];
			}
			if(col == 3) { // only for position col last element is * 1.0
				result[row*4 + col] += m1[row*4 + 3];
			}
		}
	}
}

/*
 * get inverse
 *
 * cAngle in case of A0, set C to this angle
 */
void RobotKinematics::getInverseBySkew(const float *mxTo, float *anglesTo, float cAngle) const noexcept {
	if(specialMethod == 1) { // CoreXY AC or BC
		getInverseAC(mxTo, anglesTo, cAngle);
		getInverseCoreXY_XYZ(mxTo, anglesTo, true);
	}
	else {
		tempS50.copy("only CoreXY5AC is implemented currently");
		errorMessage(tempS50.GetRef());
	}
}

/*
 * recover CA or CB
 * acMode true = AC, false = BC
 *
 * anglesCA: 0 has C, 1 has A/B. Array can be larger, but size 2 at least
 */

void RobotKinematics::getInverseAC(const float *mxTo, float *anglesCA, float cAngle) const noexcept {

	if(mxTo[2] == 0.0 && mxTo[6] == 0.0) { // Z axis is vertical, i.e. A/B is 0
		anglesCA[0] = cAngle;
	}
	else {
		if(abcType == 0) { // AC
			anglesCA[0] = atan2f(mxTo[2], -mxTo[6]) * radiansToDegrees; // C
		}
		else if(abcType == 1) { // BC
			anglesCA[0] = atan2f(mxTo[6], mxTo[2]) * radiansToDegrees; // C
		}
		else {
			tempS50.copy("error: CoreXY AC type is not defined");
			errorMessage(tempS50.GetRef());
		}
	}
	anglesCA[1] = acosf(mxTo[10]) * radiansToDegrees; // A/B

	if(abSign == 2 || (abSign == 0 && anglesCA[1] >= 0.0) || (abSign == 1 && anglesCA[1] <= 0.0)) {
		// this is ok
	}
	else { // take other solution
		if(anglesCA[0] < 0) {
			anglesCA[0] += 180.0;
			anglesCA[1] = - anglesCA[1];
		}
		else {
			anglesCA[0] -= 180.0;
			anglesCA[1] = - anglesCA[1];
		}
	}

}

void RobotKinematics::getInverseCoreXY_XYZ(const float *mxTo, float *anglesResult, bool iscorexy)
		const noexcept {

	startClock();

	// create matrix with correct orientation vectors and position from mxTo
	float mxInitAC[12];
	float mxC[12];
	getRodriguesByLetter('C', anglesResult[0], mxC);
	float mxA[12];
	getRodriguesByLetter('A', anglesResult[1], mxA);
	//multiplyRotationMatrix(mxA, mxC, mxInitAC);
	multiplyRotationMatrix(mxC, mxA, mxInitAC);
	mxInitAC[3] = mxTo[3];
	mxInitAC[7] = mxTo[7];
	mxInitAC[11] = mxTo[11];

	// e-A e-C noap M-1 = eZ eY eX
	float mxAinv[12];
	rotationMatrixInverse(mxA, mxAinv);
	float mxCinv[12];
	rotationMatrixInverse(mxC, mxCinv);
	multiplyRotationMatrix(mxAinv, mxCinv, mxTemp);
	multiplyRotationMatrixInplaceLeft(mxTemp, mxInitAC);
	multiplyRotationMatrixInplaceLeft(mxTemp, screw_MInv);

	anglesResult[0] -= cache[offsetMreference]; //C
	anglesResult[1] -= cache[offsetMreference+1]; //A
	anglesResult[2] =  mxTemp[11] - cache[offsetMreference+2]; // Z
	anglesResult[3] = mxTemp[3] - cache[offsetMreference+3]; // X
	anglesResult[4] = mxTemp[7] - cache[offsetMreference+4]; // Y

	sumOfTimes += stopClock();
	timeMeasurements++;
}

void RobotKinematics::normalizeVector(float &x, float &y, float &z) const noexcept {
	if(x == 0.0 && y == 0.0 && z == 1.0) return;
	if(x == 0.0 && y == 1.0 && z == 0.0) return;
	if(x == 1.0 && y == 0.0 && z == 0.0) return;
	float len = sqrtf(x*x + y*y + z*z);
	if(len != 0.0) {
		x /= len;
		y /= len;
		z /= len;
	}
}

/*
 * for inverse: translate G-Code into mx matrix (only z-axis and pos relevant)
 */

void RobotKinematics::XYZACTomx(const float *xyzac, float *mx) const noexcept {
	// rotation info:
	getRodriguesByLetter('C', xyzac[4], mx);
	getRodriguesByLetter('A', xyzac[3], mxTemp);
	multiplyRotationMatrixInplaceLeft(mx, mxTemp);
	// position info:
	mx[3] = xyzac[0];
	mx[7] = xyzac[1];
	mx[11] = xyzac[2];

	//	mx[2] = sinf(xyzac[3]/radiansToDegrees) * sinf(xyzac[4]/radiansToDegrees);
	//	mx[6] = cosf(xyzac[4]/radiansToDegrees) * sinf(xyzac[3]/radiansToDegrees);
	//	mx[10] = cosf(xyzac[3]/radiansToDegrees);	// cA
}

/*
 * for inverse: translate G-Code into mx matrix (only z-axis and pos relevant)
 */
void RobotKinematics::XYZBCTomx(const float *xyzbc, float *mx) const noexcept {
	mx[3] = xyzbc[0];
	mx[7] = xyzbc[1];
	mx[11] = xyzbc[2];

	mx[2] = cosf(xyzbc[4]/radiansToDegrees) * sinf(xyzbc[3]/radiansToDegrees);
	mx[6] = sinf(xyzbc[3]/radiansToDegrees) * sinf(xyzbc[4]/radiansToDegrees);
	mx[10] = cosf(xyzbc[3]/radiansToDegrees);	// cosB
}

/*
 * for forward: translate mx matrix into XYZAC pos/angles (only z axis and pos of mx relevant)
 */
void RobotKinematics::mxToXYZAC(const float *mx, float *xyzac, float cAngle) const noexcept {
	xyzac[0] = mx[3];
	xyzac[1] = mx[7];
	xyzac[2] = mx[11];

	float anglesCA[2];
	getInverseAC(mx, anglesCA, cAngle);
	xyzac[3] = anglesCA[1]; // A
	xyzac[4] = anglesCA[0]; // C
}

/*
 * for forward: translate mx matrix into XYZBC pos/angles (only z axis and pos of mx relevant)
 */
void RobotKinematics::mxToXYZBC(const float *mx, float *xyzbc, float cAngle) const noexcept {
	xyzbc[0] = mx[3];
	xyzbc[1] = mx[7];
	xyzbc[2] = mx[11];

	float anglesCB[2];
	getInverseAC(mx, anglesCB, cAngle);
	xyzbc[3] = anglesCB[1]; // B
	xyzbc[4] = anglesCB[0]; // C
}

void RobotKinematics::getRodriguesByLetter(char drive, float theta, float *mx) const noexcept {
	int pos = getPositionOfLetterInChain(drive);
	if(axisTypes[pos] == 'R') {
		int rotarynr = getRotaryIndex(pos);
		getRodrigues2_Rot(&cache[pos*3 + offsetScrewOmega], &cache[pos*3+ offsetScrewQ],
				&cache[rotarynr*12 + offsetScrewOmega2], &cache[rotarynr*3 + offsetScrewV],
				(theta - cache[offsetMreference+pos]) / radiansToDegrees, mx);
	}
	else { // 'P'
		getRodrigues2_Pris(&cache[pos*3+ offsetScrewOmega], theta - cache[offsetMreference+pos], mx);
	}
}

void RobotKinematics::getRodriguesByLetterInv(char drive, float theta, float *mxInv) const noexcept {
	getRodriguesByLetter(drive, theta, mxTemp);
	rotationMatrixInverse(mxTemp, mxInv);
}

void RobotKinematics::removeMatrixNearZero(float *mx) const noexcept {
	for(int i=0; i< 12; i++) {
		if(abs(mx[i]) < 1e-6) {
			mx[i] = 0.0;
		}
	}

}

void RobotKinematics::initNeutralMatrix(float *mx) const noexcept {
	for(int i=0; i < 12; i++) {
		if(i == 0 || i == 5 || i == 10) {
			mx[i] = 1.0f;
		}
		else {
			mx[i] = 0.0f;
		}
	}
}

/*
 * mx is in 3x4 form, 4th colum will be ignored
 * vec, vecTo are 3x1
 */

void RobotKinematics::multiplyTrmatrixWithVector(const float *mx, const float *vec, float *vecTo) const noexcept {
	vecTo[0] = mx[0]*vec[0] + mx[1]*vec[1] + mx[2]*vec[2];
	vecTo[1] = mx[4]*vec[0] + mx[5]*vec[1] + mx[6]*vec[2];
	vecTo[2] = mx[8]*vec[0] + mx[9]*vec[1] + mx[10]*vec[2];
}

/*
 * calculate rotation by 0,0,0
 *
 * rotor order of parameters: cos, - sin z, sin y, - sin x
 *
 * pt[3] and pt[4] are not used and omitted => array size 3 possible
 */

void RobotKinematics::GAcalculateRotorInplace(const float *rotor, float *pt) const noexcept {
	// R * pt

	temp4[0] = rotor[0] * pt[0] + rotor[1] * pt[1] + rotor[2] * pt[2];
	temp4[1] = rotor[0] * pt[1] - rotor[1] * pt[0] + rotor[3] * pt[2];
	temp4[2] = rotor[0] * pt[2] - rotor[2] * pt[0] - rotor[3] * pt[1];
	temp4[3] = rotor[1] * pt[2] - rotor[2] * pt[1] + rotor[3] * pt[0];

	// * ~R:

	pt[0] = temp4[0] * rotor[0] + temp4[1] * rotor[1] + temp4[2] * rotor[2] + temp4[3] * rotor[3];
	pt[1] = - temp4[0] * rotor[1] + temp4[1] * rotor[0] + temp4[2] * rotor[3] - temp4[3] * rotor[2];
	pt[2] = - temp4[0] * rotor[2] - temp4[1] * rotor[3] + temp4[2] * rotor[0] + temp4[3] * rotor[1];
}

void RobotKinematics::GAMotorToCartesian(const float *motor_xyzac, const float *ABpt, const float *Cpt,
		float *cartXYZAC_0, bool isAC) const noexcept {

	float point[3] = {motor_xyzac[0] - ABpt[0], motor_xyzac[1] - ABpt[1], motor_xyzac[2] - ABpt[2]};

	// rotate negative A:
	float angleA = motor_xyzac[3] / radiansToDegrees; // angle / 360 * 2 * pi
	float rotorA[4] = {cosf(-angleA / 2.0), 0, 0, 0};	// order zyx. by X, Z is negative, Y is positive sin
	if(isAC) { // AC axis direction 1,0,0
		rotorA[3] = - sinf(-angleA / 2.0); // X direction
	}
	else { // BC, other axis direction 0,1,0
		rotorA[2] = sinf(-angleA / 2.0); // Y direction
	}
	GAcalculateRotorInplace(rotorA, point); // ptTo=R*pt/R;

	// translate A back and C forward:
	point[0] += (ABpt[0] - Cpt[0]);
	point[1] += (ABpt[1] - Cpt[1]);
	point[2] += (ABpt[2] - Cpt[2]);

	// rotate negative C:
	float angleC = motor_xyzac[4] / radiansToDegrees; // angle / 360 * 2 * pi
	float rotorC[4] = {cosf(-angleC / 2.0), - sinf(-angleC / 2.0), 0, 0};	// 0, 6, 7, 10 cos/z/y/x
	GAcalculateRotorInplace(rotorC, point); // ptTo=R*pt/R;

	cartXYZAC_0[0] = point[0] + Cpt[0]; // translate C back
	cartXYZAC_0[1] = point[1] + Cpt[1]; // translate C back
	cartXYZAC_0[2] = point[2] + Cpt[2]; // translate C back
	cartXYZAC_0[3] = motor_xyzac[3];
	cartXYZAC_0[4] = motor_xyzac[4];

}

void RobotKinematics::GACartesianToMotor(const float *cartXYZAC, const float *ABpt, const float *Cpt, float *motor_xyz,
		bool isAC) const noexcept {

	// translate by C offset
	float point[3] = {cartXYZAC[0] - Cpt[0], cartXYZAC[1] - Cpt[1], cartXYZAC[2] - Cpt[2]};

	// rotate by C
	float angleC = cartXYZAC[4] / radiansToDegrees; // angle / 360 * 2 * pi
	float rotorC[4] = {0,0,0,0};	// 	// by X, Z is negative, Y is positive sin
	rotorC[0] = cosf(angleC / 2.0);
	rotorC[1] = -sinf(angleC / 2.0); // Z and X with minus sign
	GAcalculateRotorInplace(rotorC, point); // ptTo=R*pt/R;

	// translate C offsets back, A offsets:
	point[0] += (Cpt[0] - ABpt[0]);
	point[1] += (Cpt[1] - ABpt[1]);
	point[2] += (Cpt[2] - ABpt[2]);

	// rotate by A, store in point4:
	float angleA = cartXYZAC[3] / radiansToDegrees; // angle / 360 * 2 * pi
	float rotorA[4] = {cosf(angleA / 2.0), 0, 0, 0};	// 0, 6, 7, 10 cos/z/y/x
	if(isAC) { // AC
		rotorA[3] = - sinf(angleA / 2.0); // X direction
	}
	else { // BC
		rotorA[2] = sinf(angleA / 2.0); // Y direction
	}
	GAcalculateRotorInplace(rotorA, point); // ptTo=R*pt/R;

	motor_xyz[0] = point[0] + ABpt[0]; // translate A back
	motor_xyz[1] = point[1] + ABpt[1]; // translate A back
	motor_xyz[2] = point[2] + ABpt[2]; // translate A back
	motor_xyz[3] = cartXYZAC[3];
	motor_xyz[4] = cartXYZAC[4];

}


#endif


