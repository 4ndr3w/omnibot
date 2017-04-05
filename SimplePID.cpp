#include "SimplePID.h"
#include "Timer.h"
#include "math.h"

SimplePID::SimplePID(double kP, double kI, double kD, double kF) :
  setpoint(0), error(0), lastT(0), errSum(0), prevError(0),
  minOut(-1), maxOut(1), maxInput(0), continous(false), resetD(false), antiWindup(false) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->kF = kF;
}

SimplePID::SimplePID() : SimplePID(0,0,0,0) {

}

void SimplePID::setP(double kP) {
  this->kP = kP;
}

void SimplePID::setI(double kI) {
  this->kI = kI;
}

void SimplePID::setD(double kD) {
  this->kD = kD;
}

void SimplePID::setF(double kF) {
  this->kF = kF;
}

void SimplePID::setPIDF(double kP, double kI, double kD, double kF) {
  this->kP = kP;
  this->kI = kI;
  this->kD = kD;
  this->kF = kF;
}

void SimplePID::setContinous(double maxInput) {
  this->continous = true;
  this->maxInput = maxInput;
}

void SimplePID::setSetpoint(double setpoint) {
  this->setpoint = setpoint;
}

double SimplePID::getSetpoint() {
  return setpoint;
}

double SimplePID::getError() {
  if (resetD)
    return 9999;
  return error;
}

double SimplePID::getErrSum() {
  return errSum;
}

bool SimplePID::isStable(double allowed) {
  return fabs(error) < allowed;
}

void SimplePID::reset() {
  lastT = Timer::GetFPGATimestamp();
  prevError = 0;
  errSum = 0;
  resetD = true;
}

double SimplePID::calculate(double input) {
  return calculate(input, NULL);
}

double SimplePID::calculate(double input, PIDSnapshot *snapshot) {
  double now = Timer::GetFPGATimestamp();
  double dT = now-lastT;

  error = setpoint-input;

  if ( continous && fabs(error) > (maxInput/2) )
	{
			if ( error > 0 )
			   error = error - maxInput;
			else
			   error = error + maxInput;
	}

  errSum += error*dT;

  double deltaPos = (error-prevError);
  if ( resetD ) {
    deltaPos = 0;
    resetD = false;
  }
  lastT = now;
  prevError = error;

  if ( antiWindup && ((errSum > 0 && error < 0) || (errSum < 0 && errSum > 0)) )
    errSum = 0;

  double pContrib = kP*error;
  double iContrib = kI*errSum;
  double dContrib = kD*(deltaPos/dT);
  double fContrib = kF*setpoint;

  double out = pContrib + iContrib + dContrib + fContrib;

  if ( snapshot != NULL )
  {
    snapshot->error = error;
    snapshot->setpoint = setpoint;
    snapshot->input = input;

    snapshot->p = pContrib;
    snapshot->i = iContrib;
    snapshot->d = dContrib;

    snapshot->f = fContrib;

    snapshot->out = out;
  }

  return out;
}
