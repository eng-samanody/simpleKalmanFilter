#import "kalman.h"

/**
* Create 1-dimensional kalman filter
* @param  {double} R -> processNoise
* @param  {double} Q -> measurementNoise
* @param  {double} A -> stateVector
* @param  {double} B -> controlVector
* @param  {double} C -> measurementVector
* @return {KalmanFilter}
*/


KalmanFilter::KalmanFilter(double R = 1, double Q = 1, double A = 1, double B = 0, double C = 1) {

processNoise = R; // noise power desirable
measurementNoise = Q; // noise power estimated

stateVector = A;
measurementVector = C;
controlVector = B;
cov = NULL;
estimatedSignal = NULL; // estimated signal without noise // x
}



/**
* Filter a new value
* @param  {double} z newSignal Measurement
* @param  {double} u Control
* @return {double} estimatedSignal
*/
double KalmanFilter::filter(double newSignal , double u = 0) {

if (estimatedSignal == NULL) {
  estimatedSignal = (1 / measurementVector) * newSignal;
  cov = (1 / measurementVector) * measurementNoise * (1 / measurementVector);
}
else {

  // Compute prediction
  const double predX = (stateVector * estimatedSignal) + (controlVector * u);
  const double predCov = ((stateVector * cov) * stateVector) + processNoise;

  // Kalman gain
  const double K = predCov * measurementVector * (1 / ((measurementVector * predCov * measurementVector) + measurementNoise));

  // Correction
  estimatedSignal = predX + K * (newSignal - (measurementVector * predX));
  cov = predCov - (K * measurementVector * predCov);
}

return estimatedSignal;
}

/**
* Return the last filtered measurement
* @return {Number}
*/
KalmanFilter::lastMeasurement(void) {
return estimatedSignal;
}

/**
* Set measurement noise Q
* @param {double} noise
*/
KalmanFilter::setMeasurementNoise(double noise) {
measurementNoise = noise;
}

/**
* Set the process noise R
* @param {double} noise
*/
KalmanFilter::setProcessNoise(double noise) {
processNoise = noise;
}

  /**
* Get measurement noise Q
* @return {double} noise
*/
double KalmanFilter::getMeasurementNoise(void) {
return measurementNoise;
}

/**
* Get the process noise R
* @return {double} noise
*/
double KalmanFilter::getProcessNoise(void) {
return processNoise;
}

