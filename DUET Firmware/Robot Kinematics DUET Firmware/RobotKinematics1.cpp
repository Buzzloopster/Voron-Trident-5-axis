/*
 * RobotKinematics1.cpp config
 *
 *  Created on: 06 Aug 2023
 *      Author: JoergS5
 */

#include <Movement/Kinematics/RobotKinematics.h>
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Platform/Tasks.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>


void RobotKinematics::setB(const char* _robottype) const noexcept {
	tempS20.copy(_robottype);
	if(tempS20.Contains("CoreXY5AC") == 0 || tempS20.Contains("CAZ_corexy(YX)") == 0
			|| tempS20.Contains("CAZ_corexy(XY)") == 0) {
		setAxisTypes("RRPPP"); // sets numOfAxes also
		setForwardChain("CAZ_corexy(XY)");
		abcType = 0;
	}
	else if(tempS20.Contains("CoreXY5BC") == 0 || tempS20.Contains("CBZ_corexy(YX)") == 0
			|| tempS20.Contains("CBZ_corexy(XY)") == 0) {
		setAxisTypes("RRPPP"); // sets numOfAxes also
		setForwardChain("CBZ_corexy(XY)"); // sets specialMethod also
		abcType = 1;
	}
	else {
		tempS50.copy("currently only CoreXY5AC and CoreXY5BC are supported\n");
		consoleMessage(tempS50.GetRef());
	}
	initCache();
}

void RobotKinematics::initCache() const noexcept {
	int numOfRAxes = 0;
	for(int i=0; i < numOfAxes; i++) {
		if(axisTypes[i] == 'R') {
			numOfRAxes++;
		}
	}

	for(size_t i=0; i < CACHESIZE; i++) {
		cache[i] = 0.0;
	}
	offsetScrewOmega = 0;
	offsetScrewQ = numOfAxes * 3;
	offsetScrewOmega2 = offsetScrewQ + numOfAxes * 3;
	offsetScrewV = offsetScrewOmega2 + numOfRAxes*12;
	offsetMreference = offsetScrewV + numOfRAxes*3;
	offsetAngleLimits = offsetMreference + numOfAxes;
	offsetStartOfInverse = offsetAngleLimits + numOfAxes*3;

	offsetCurrentEnd = offsetStartOfInverse;
}

/*
 * calculated values of forwardChainCompressed and forwardChainSpecial, set specialMethod
 */
// tested
void RobotKinematics::setForwardChain(const char* value) const noexcept {
	tempS20.copy(value);
	for(int i=0; i < 20; i++) {
		forwardChain[i] = value[i];
		if(value[i] == '\0') {
			break;
		}
	}
	checkAndSetSpecial("corexy(", 1);
	checkAndSetSpecial("corexz(", 2);
	checkAndSetSpecial("lindelta(", 15);
	checkAndSetSpecial("rotdelta(", 10);
	checkAndSetSpecial("5bar(", 9);
	checkAndSetSpecial("pall(", 14);
}

/*
 * set forwardChainCompressed: uppercase letters for normal operation, lowercase for special kinematics
 * specialMethod is also set, values are from K numbers if available
 */
void RobotKinematics::checkAndSetSpecial(const char *checkfor, int specialnr) const noexcept {
	int pos = tempS20.Contains(checkfor);

	int len = 0;
	for(int i=0; i < 20; i++) {
		if(checkfor[i] == '\0') {
			break;
		}
		len++;
	}

	int idx = 0;
	if(pos >= 0) {
		addNormalLetters(0,pos,idx);
		int offsetAfter = extractSpecial(pos+len, idx);
		addNormalLetters(offsetAfter,20,idx);
		forwardChainCompressed[idx] = '\0';
		specialMethod = 1;
	}

}

void RobotKinematics::addNormalLetters(int start, int stop,int &idx) const noexcept {
	for(int i=start; i < stop; i++) {
		if(forwardChain[i] == 'X' || forwardChain[i] == 'Y' || forwardChain[i] == 'Z'
				|| forwardChain[i] == 'A' || forwardChain[i] == 'B' || forwardChain[i] == 'C') {
			forwardChainCompressed[idx] = forwardChain[i];
			idx++;
		}
		else if(forwardChain[i] == '_') {
			// ignore
		}
		else if(forwardChain[i] == '\0') {
			break;
		}
	}
}

// return index after extraction
int RobotKinematics::extractSpecial(int start, int &idx) const noexcept {
	int ct = 0;
	int idxSpecial = 0;
	for(int i=start; i < 20; i++) {
		if(forwardChain[i] == ')' || forwardChain[i] == '\0') {
			if(forwardChain[i] == ')') {
				ct++;
			}
			break;
		}
		forwardChainCompressed[idx] = '.'; // placeholder for special
		forwardChainSpecial[idxSpecial] = forwardChain[i];
		idxSpecial++;
		ct++;
		idx++;
	}
	forwardChainSpecial[idxSpecial] = '\0';
	return start + ct;
}

// order is according to chain order (Open5x: BCYZX / UVYZX)
void RobotKinematics::setAxisTypes(const char* types) const noexcept {
	tempS20.copy(types);
	int len = int(tempS20.strlen());
	for(int i=0; i < len; i++) {
		axisTypes[i] = types[i];
	}
	axisTypes[len] = '\0';
	setNumOfAxes();
}

void RobotKinematics::errorMessage(const StringRef &msg) const noexcept {
	reprap.GetPlatform().Message(ErrorMessage, msg.c_str());
}

void RobotKinematics::consoleMessage(const StringRef &msg) const noexcept {
	reprap.GetPlatform().Message(LoggedGenericMessage, msg.c_str());
}

void RobotKinematics::debugMatrix(const char *title, const float*mx) const noexcept {
	tempS50.Clear();
	tempS50.catf("%s %.2f %.2f %.5f %.2f", title, (double) mx[0], (double) mx[1], (double) mx[2], (double) mx[3]);
	consoleMessage(tempS50.GetRef());
	tempS50.copy("\n");
	consoleMessage(tempS50.GetRef());

	tempS50.Clear();
	tempS50.catf("   %.2f %.2f %.5f %.2f", (double) mx[4], (double) mx[5], (double) mx[6], (double) mx[7]);
	consoleMessage(tempS50.GetRef());
	tempS50.copy("\n");
	consoleMessage(tempS50.GetRef());

	tempS50.Clear();
	tempS50.catf("   %.2f %.2f %.5f %.2f", (double) mx[8], (double) mx[9], (double) mx[10], (double) mx[11]);
	consoleMessage(tempS50.GetRef());
	tempS50.copy("\n");
	consoleMessage(tempS50.GetRef());

}

void RobotKinematics::debugList(const char *title, int size, const float *list) const noexcept {
	tempS50.Clear();
	tempS50.catf("%s %.2f %.2f %.2f %.2f %.2f", title, (double) list[0], (double) list[1], (double) list[2],
			(double) list[3], (double) list[4]);
	consoleMessage(tempS50.GetRef());
	tempS50.copy("\n");
	consoleMessage(tempS50.GetRef());

}

void RobotKinematics::debugList(const char *title, int size, int32_t *list) const noexcept {
	tempS50.Clear();
	tempS50.catf("%s %i %i %i %i %i", title, (int) list[0], (int) list[1], (int) list[2],
			(int) list[3], (int) list[4]);
	consoleMessage(tempS50.GetRef());
	tempS50.copy("\n");
	consoleMessage(tempS50.GetRef());

}

float RobotKinematics::getFloatOfElement(const char *value, char separator, int startpos, int idx) const noexcept {
	char v[20];
	int curSep = 0;
	int curPos = 0;
	for(int i=startpos; i < 200; i++) {
		if(value[i] == '\0' || curSep > idx) {
			v[curPos] = '\0';
			break;
		}
		else if(value[i] == separator) {
			curSep++;
		}
		else if(curSep == idx) {
			v[curPos] = value[i];
			curPos++;
		}
	}
	return SafeStrtof(v);
}

/*
 * set numOfAxes, calculated from axisTypes
 */
void RobotKinematics::setNumOfAxes() const noexcept {
	int ct = 0;
	for(size_t i=0; i < MAXNUMOFAXES+1; i++) {
		if(axisTypes[i] == '\0') {
			break;
		}
		ct++;
	}
	numOfAxes = ct;
}

void RobotKinematics::setC(const char *value) const noexcept {
	tempS20.copy(value); // will cut value if too long
	int startpos = -1; // position after the beginning marker "...="
	if(tempS20.Contains("Mnoap=") == 0) {
		int ct = getValueCount(value, "Mnoap=", ':', startpos);
		if(ct == 12) {
			float values[12];
			for(int i=0; i < 12; i++) {
				values[i] = getFloatOfElement(value, ':', startpos, i);
			}
			setSkew_M(values);
		}
	}
	else if(tempS20.Contains("Mreference=") == 0) {
		int ct = getValueCount(value, "Mreference=", ':', startpos);
		if(ct == numOfAxes) {
			for(int i=0; i < numOfAxes; i++) {
				cache[offsetMreference+i] = getFloatOfElement(value, ':', startpos, i);
			}
		}
	}
	else { // letter
		// C"letter=omega1:omega2:omega3:q1:q2:q3" axis orientation and a point on the axis
		char letter = value[0];
		int pos = getPositionOfLetterInChain(letter);

		char mark[3] = {letter, '=', '\0'};
		int ct = getValueCount(value, mark, ':', startpos);

		if(ct == 6) {
			cache[offsetScrewOmega + pos*3] = getFloatOfElement(value, ':', startpos, 0);
			cache[offsetScrewOmega + pos*3 + 1] = getFloatOfElement(value, ':', startpos, 1);
			cache[offsetScrewOmega + pos*3 + 2] = getFloatOfElement(value, ':', startpos, 2);
			normalizeVector(cache[offsetScrewOmega + pos*3], cache[offsetScrewOmega + pos*3 + 1],
					cache[offsetScrewOmega + pos*3 + 2]);

			cache[offsetScrewQ + pos*3] = getFloatOfElement(value, ':', startpos, 3);
			cache[offsetScrewQ + pos*3 + 1] = getFloatOfElement(value, ':', startpos, 4);
			cache[offsetScrewQ + pos*3 + 2] = getFloatOfElement(value, ':', startpos, 5);

			if(axisTypes[pos] == 'R') {
				int rotaryNr = getRotaryIndex(pos); // index of R axes. Analyses axisTypes
				if(rotaryNr > -1) {
					setRodrigues1(&cache[pos*3 + offsetScrewOmega], &cache[pos*3 + offsetScrewQ],
							&cache[rotaryNr*12 + offsetScrewOmega2], &cache[rotaryNr*3 + offsetScrewV]);
				}
			}

		}

	}
}

/*
 * get number of elements which are separated by separator (: in most cases)
 *
 * \param startpos is the position after the = (length of marker incl =)
 * \return the count is without the beginning text= marker
 */
// tested
int RobotKinematics::getValueCount(const char *value, const char *marker, char separator, int &startpos)
	const noexcept {

	int lenMarker = 0;
	for(int i=0; i < 500; i++) {
		if(marker[i]== '\0') {
			break;
		}
		lenMarker++;
	}
	startpos = lenMarker;

	int ct = 0;
	for(int i=0; i< 500; i++) {
		if(value[i]== '\0') {
			break;
		}
		else if(value[i] == separator) {
			ct++;
		}
	}
	return ct + 1;
}

void RobotKinematics::setA(const char *value) const noexcept {
	char letter = value[0];
	int chainOrder = getPositionOfLetterInChain(letter);
	char marker[3] = {letter, '=', '\0'};
	int startpos;
	int ct = getValueCount(value, marker, ':', startpos);
	if(ct == 3) {
		cache[offsetAngleLimits + chainOrder*3] = getFloatOfElement(value, ':', startpos, 0); // min
		cache[offsetAngleLimits + chainOrder*3 + 1] = getFloatOfElement(value, ':', startpos, 1); // max
		cache[offsetAngleLimits + chainOrder*3 + 2] = getFloatOfElement(value, ':', startpos, 2); // home
	}
}

int RobotKinematics::getPositionOfLetterInChain(char letter) const noexcept {
	int dotstart = -1;
	// letter of normal kin
	for(int i=0; i < numOfAxes; i++) {
		if(forwardChainCompressed[i] == letter) {
			return i;
		}
		if(forwardChainCompressed[i] == '.' && dotstart == -1) {
			dotstart = i;
		}
	}
	// special kin
	for(int i=0; i < 5; i++) {
		if(forwardChainSpecial[i] == '\0') {
			break;
		}
		else if(forwardChainSpecial[i] == letter) {
			if(dotstart != -1) {
				return dotstart + i;
			}
		}
	}
	return -1; // not found
}

void RobotKinematics::setP(const char *value) const noexcept {
	tempS20.copy(value);
	if(tempS20.Contains("axisTypes=") == 0) {
		int pos = 0;
		for(int i=10; i < 50; i++) {
			if(value[i] == '\0') {
				axisTypes[pos] = '\0';
				break;
			}
			axisTypes[pos] = value[i];
			pos++;
		}
		setNumOfAxes();
	}
	else if(tempS20.Contains("abSign=") == 0) {
		if(tempS20.EndsWith('1')) {
			abSign = 1;
		}
		else {
			abSign = 0;
		}
	}
	else {

	}

}

/*
 * \return letter. If special, look into forwardChainSpecial. \0 if not found
 */

char RobotKinematics::getLetterInChain(int pos) const noexcept {
	if(pos >= numOfAxes || pos < 0) {
		return '\0';
	}
	char letter = forwardChainCompressed[pos];
	if(letter == '.') { // special method letter
		int ctBefore = 0;
		for(int i=0; i < numOfAxes; i++) {
			if(forwardChainCompressed[i] == '.' && i < pos) {
				ctBefore++;
			}
			else if(i >= pos) {
				break;
			}
		}
		letter = forwardChainSpecial[ctBefore];
	}
	return letter;
}

void RobotKinematics::startClock() const noexcept {
	starttime = StepTimer::GetTimerTicks();
}

/*
 * return microseconds
 */

float RobotKinematics::stopClock() const noexcept {
	uint32_t stoptime = StepTimer::GetTimerTicks();
	uint32_t tickrate = StepTimer::GetTickRate(); // 750000 => 750 kHz => 1.33 microseconds for one tick
	float microsecondsForOneTick = 1e6f / (float) tickrate;
	if(starttime < stoptime) {
		return (float) (stoptime - starttime) * microsecondsForOneTick;
	}
	else {
		return 0.0; // todo handle overflow
	}
}

void RobotKinematics::reportConfiguration(GCodeBuffer& gb) const noexcept {
	const MessageType mt = (MessageType)(gb.GetResponseMessageType() | PushFlag);
	reprap.GetPlatform().Message(mt, "=== M669 K13 current config ===\n");

	// general information:
	reprap.GetPlatform().MessageF(mt, "numOfAxes %i axisTypes %s chain %s (normal %s special %s)\n", numOfAxes, axisTypes,
			forwardChain, forwardChainCompressed, forwardChainSpecial);

	// axis details:
	for(int i=0; i < numOfAxes; i++) {
		char lett[2];
		lett[0] = getLetterInChain(i);
		lett[1] = '\0';

		float orix = cache[offsetScrewOmega + i*3];
		float oriy = cache[offsetScrewOmega + i*3+1];
		float oriz = cache[offsetScrewOmega + i*3+2];
		float ptx = cache[offsetScrewQ + i*3];
		float pty = cache[offsetScrewQ + i*3+1];
		float ptz = cache[offsetScrewQ + i*3+2];
		float angMin = cache[offsetAngleLimits + i*3];
		float angMax = cache[offsetAngleLimits + i*3+1];
		float angHome = cache[offsetAngleLimits + i*3+2];

		reprap.GetPlatform().MessageF(mt,
				"axis %s ori: %.2f %.2f %.2f point: %.2f %.2f %.2f angles min/max/home: %.2f %.2f %.2f\n",
				lett, (double) orix,
				(double) oriy, (double) oriz, (double) ptx, (double) pty, (double) ptz,
				(double) angMin, (double) angMax, (double) angHome);
	}

	// screw values:
	reprap.GetPlatform().Message(mt,"Screw values:\n");
	reprap.GetPlatform().Message(mt,"   reference angles/positions: ");
	for(int i=0; i < numOfAxes; i++) {
		float refAnglePos = cache[offsetMreference + i];
		reprap.GetPlatform().MessageF(mt," %.2f", (double) refAnglePos);
	}
	reprap.GetPlatform().Message(mt,"\n");

	reprap.GetPlatform().MessageF(mt,"   endpoint axis X: %.2f %.2f %.2f\n", (double) screw_M[0],
			(double) screw_M[4], (double) screw_M[8]);
	reprap.GetPlatform().MessageF(mt,"   endpoint axis Y: %.2f %.2f %.2f\n", (double) screw_M[1],
			(double) screw_M[5], (double) screw_M[9]);
	reprap.GetPlatform().MessageF(mt,"   endpoint axis Z: %.2f %.2f %.2f\n", (double) screw_M[2],
			(double) screw_M[6], (double) screw_M[10]);
	reprap.GetPlatform().MessageF(mt,"   endpoint point: %.2f %.2f %.2f\n", (double) screw_M[3],
			(double) screw_M[7], (double) screw_M[11]);

	if(specialMethod != 0) {
		if(specialMethod == 1) { tempS20.copy("CoreXY"); }
		if(specialMethod == 2) { tempS20.copy("CoreXZ"); }
		if(specialMethod == 9) { tempS20.copy("5BarParScara"); }
		if(specialMethod == 10) { tempS20.copy("RotaryDelta"); }
		if(specialMethod == 14) { tempS20.copy("Palletized"); }
		if(specialMethod == 15) { tempS20.copy("LinearDelta"); }
		reprap.GetPlatform().MessageF(mt,"special kinematics set: %s\n", tempS20.c_str());
		if(specialMethod == 9) {
			reprap.GetPlatform().MessageF(mt,"   workmode: %i\n", (int) currentWorkmode);
		}
	}
	if(numOfAxes == 5) {
		reprap.GetPlatform().MessageF(mt,"abSign: %i ", abSign);
		reprap.GetPlatform().Message(mt," (A/B angle preference: 0 take >=0, 1 take <= 0, 2 don't change calculation\n");
	}

	// cache used:
	reprap.GetPlatform().MessageF(mt,"cache used: %i maximum: %i\n", offsetCurrentEnd, CACHESIZE);


}
