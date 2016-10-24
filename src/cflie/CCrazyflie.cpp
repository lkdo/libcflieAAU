// Original work Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
// Modified work Copyright (c) 2016, Luminita C. Totu <lct@es.aau.dk>, Aalborg University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


/* Original Author: Jan Winkler */
/* Modifications by: Luminita C. Totu, Aalborg University */


#include "cflie/CCrazyflie.h"
#include "cflie/clockgettime.h"

CCrazyflie::CCrazyflie(CCrazyRadio *crRadio) {
  m_crRadio = crRadio;
  
  // Review these values
  m_fMaxAbsRoll = 45.0f;
  m_fMaxAbsPitch = m_fMaxAbsRoll;
  m_fMaxYaw = 180.0f;
  m_nMaxThrust = 60000;
  m_nMinThrust = 0;//15000;

  m_fRoll = 0;
  m_fPitch = 0;
  m_fYaw = 0;
  m_nThrust = 0;

  m_fX = 0;
  m_fY = 0;
  m_fZ = 0;

  m_bSendsSetpoints = false;
  m_bSendsExtPosition = false;

  m_tocParameters = new CTOC(m_crRadio, 2);
  m_tocLogs = new CTOC(m_crRadio, 5);

  m_enumState = STATE_ZERO;

  m_dSendSetpointPeriod = 0.01; // Seconds
  m_dSendExtPositionMinPeriod = 0.01; // Seconds

  m_dSetpointLastSent = 0;
  m_dExtPositionLastSent = 0;
  m_bNewExtPosition = false;

 // m_nAckMissTolerance = ?
}

CCrazyflie::~CCrazyflie() {
  this->stopLogging();
}

bool CCrazyflie::readTOCParameters() {
	// Read the parameter block definitions on the firmware,
	// their ID number, and the subfields. Dump on console.
	std::cout << std::endl << std::endl <<  "Parameter List: " << std::endl;

	if(m_tocParameters->requestMetaData()) 
  {
    if(m_tocParameters->requestItems()) {
		return true;
	}
  } 

  return false;
}

bool CCrazyflie::readTOCLogs() {
  // Read the log block definitions on the firmware, 
  // their ID number, and the subfields. Dump on console.
  std::cout << std::endl << std::endl << "Log List: " << std::endl;

  if(m_tocLogs->requestMetaData()) {
    if(m_tocLogs->requestItems()) {
      return true;
    }
  }

  return false;
}

bool CCrazyflie::sendSetpoint(float fRoll, float fPitch, float fYaw, short sThrust) {
  fPitch = -fPitch;

  int nSize = 3 * sizeof(float) + sizeof(short);
  char *cBuffer = new char[nSize];
  memcpy(&cBuffer[0 * sizeof(float)], &fRoll, sizeof(float));
  memcpy(&cBuffer[1 * sizeof(float)], &fPitch, sizeof(float));
  memcpy(&cBuffer[2 * sizeof(float)], &fYaw, sizeof(float));
  memcpy(&cBuffer[3 * sizeof(float)], &sThrust, sizeof(short));

  CCRTPPacket *crtpPacket = new CCRTPPacket(cBuffer, nSize, 3);
  CCRTPPacket *crtpReceived = m_crRadio->sendPacket(crtpPacket);

  delete crtpPacket;
  if(crtpReceived != NULL) {
    delete crtpReceived;
    return true;
  } else {
    return false;
  }

}

bool CCrazyflie::sendExtPosition(float fX, float fY, float fZ) {
	
	int nSize = 3 * sizeof(float);
	char *cBuffer = new char[nSize];
	memcpy(&cBuffer[0 * sizeof(float)], &fX, sizeof(float));
	memcpy(&cBuffer[1 * sizeof(float)], &fY, sizeof(float));
	memcpy(&cBuffer[2 * sizeof(float)], &fZ, sizeof(float));

    CCRTPPacket *crtpPacket = new CCRTPPacket(cBuffer, nSize, 6); // EXTERNAL POSITION is to be sent on PORT 6 (find CRTP_PORT_POSITION in crtp.h in the firmware)
	CCRTPPacket *crtpReceived = m_crRadio->sendPacket(crtpPacket);

	delete crtpPacket;
	if (crtpReceived != NULL) {
		delete crtpReceived;
		return true;
	}
	else {
		return false;
	}

}


void CCrazyflie::setThrust(int nThrust) {
  m_nThrust = nThrust;

  if(m_nThrust < m_nMinThrust) {
    m_nThrust = m_nMinThrust;
  } else if(m_nThrust > m_nMaxThrust) {
    m_nThrust = m_nMaxThrust;
  }
}

bool CCrazyflie::cycle() {
  double dTimeNow = currentTime();

  switch(m_enumState) {
  case STATE_ZERO: {
#ifdef _DEBUG
    std::cout << "State: Zero"  << std::endl;
#endif
    m_enumState = STATE_READ_PARAMETERS_TOC;
  } break;

  case STATE_READ_PARAMETERS_TOC: {
#ifdef _DEBUG
	  std::cout << "State: Read parameters" << std::endl;
#endif
	  if(this->readTOCParameters()) {
      m_enumState = STATE_READ_LOGS_TOC;
    }

	 // send flight mode configuration
	  std::cout << std::endl << "Sending Preconfigured Flight Mode Parameters" << std::endl << std::endl;
	  sendFlightModeParameters();

  } break;

  case STATE_READ_LOGS_TOC: {
#ifdef _DEBUG
	  std::cout << "State: Read logs" << std::endl;
#endif
    if(this->readTOCLogs()) {
     m_enumState = STATE_START_LOGGING;
    }
  } break;

  case STATE_START_LOGGING: {
#ifdef _DEBUG
    std::cout << "State: start logging"  << std::endl;
#endif
    if(this->startLogging()) {
      m_enumState = STATE_ZERO_MEASUREMENTS;
    }
  } break;

  case STATE_ZERO_MEASUREMENTS: {
#ifdef _DEBUG
    std::cout << "State: zero measurements"  << std::endl;
#endif
    m_tocLogs->processPackets(m_crRadio->popLoggingPackets());

    // NOTE(winkler): Here, we can do measurement zero'ing. This is
    // not done at the moment, though. Reason: No readings to zero at
    // the moment. This might change when altitude becomes available.

    m_enumState = STATE_NORMAL_OPERATION;
  } break;

  case STATE_NORMAL_OPERATION: {
#ifdef _DEBUG
   //  std::cout << "State: normal operation"  << std::endl;
#endif

   // Shove over the sensor readings from the radio to the Logs TOC.
    m_tocLogs->processPackets(m_crRadio->popLoggingPackets());

	if (m_bSendsExtPosition) {
		// Check if there is new external position data
		if ((m_bNewExtPosition) && (dTimeNow - m_dExtPositionLastSent > m_dSendExtPositionMinPeriod))
		{
			this->sendExtPosition(m_fX, m_fY, m_fZ);
			m_dExtPositionLastSent = dTimeNow;
			m_bNewExtPosition = false;
		}
	}

    if(m_bSendsSetpoints) {
      // Check if it's time to send the setpoint
      if (dTimeNow - m_dSetpointLastSent > m_dSendSetpointPeriod)
	  {
	// Send the current set point based on the previous calculations
	this->sendSetpoint(m_fRoll, m_fPitch, m_fYaw, m_nThrust);
	m_dSetpointLastSent = dTimeNow;
      }
    } 
	
	if (!m_bSendsExtPosition && !m_bSendsSetpoints)
	{
      // Send a dummy packet for keepalive
      m_crRadio->sendDummyPacket();
    }

  } break;

  default: {
  } break;
  }

  if(m_crRadio->ackReceived()) {
    m_nAckMissCounter = 0;
  } else {
    m_nAckMissCounter++;
  }

  return m_crRadio->usbOK();
}

bool CCrazyflie::copterInRange() {
  return m_nAckMissCounter < m_nAckMissTolerance;
}

void CCrazyflie::setRoll(float fRoll) {
  m_fRoll = fRoll;
 
  if(std::fabs(m_fRoll) > m_fMaxAbsRoll) {
    m_fRoll = copysign(m_fMaxAbsRoll, m_fRoll);
  }
}

void CCrazyflie::setPitch(float fPitch) {
  m_fPitch = fPitch;

  if(std::fabs(m_fPitch) > m_fMaxAbsPitch) {
    m_fPitch = copysign(m_fMaxAbsPitch, m_fPitch);
  }
}

void CCrazyflie::setYaw(float fYaw) {
  m_fYaw = fYaw;

  if(std::fabs(m_fYaw) > m_fMaxYaw){
      m_fYaw = copysign(m_fMaxYaw, m_fYaw);
  }
}


void CCrazyflie::setExtPosition(float X, float Y, float Z)
{
	m_fX = X;
	m_fY = Y;
	m_fZ = Z;

	m_bNewExtPosition = true;
}

bool CCrazyflie::isInitialized() {
  return (m_enumState == STATE_NORMAL_OPERATION);
}


void CCrazyflie::sendFlightModeParameters()
{
	// LCT: make sure to check ID numbers of parameters, and 
	//      value conventions, as they depend on the firmware
	//      see the text dump to the console 


	char data;
	char ID;
	
	// flightmode.stabModeRoll ID : 10  RATE(0) vs ANGLE(1), see RPYType in firmware
	ID = 10;
	data = 1;
	m_tocParameters->sendParameter(ID, &data);

	// flightmode.stabModePitch ID : 11  RATE(0) vs ANGLE(1), see RPYType in firmware 
	ID = 11;
	data = 1;
	m_tocParameters->sendParameter(ID, &data);

	// flightmode.stabModeYaw ID : 12 RATE(0) vs ANGLE(1), see RPYType in firmware 
	ID = 12;
	data = 0;
	m_tocParameters->sendParameter(ID, &data);

	// flightmode.yawMode ID : 8 CAREFREE  = 0, see YawModeType in firmware
	ID = 8;
	data = 0;
	m_tocParameters->sendParameter(ID, &data);

	// Just for demo 
	// pid_attitude.roll_kd ID : 24 Type : 6 (float)
	/* ID = 24;
	char data_f[4];
	float fValue = 0.141516;
	memcpy(data_f,&fValue,4);
	m_tocParameters->sendParameter(ID, data_f);*/


}

bool CCrazyflie::startLogging() {

   m_tocLogs->resetLogCommand();

  // Register the desired sensor readings
 
  //this->enableGyroscopeLogging();
  //this->enableAccelerometerLogging();
  //this->enableMagnetometerLogging();
  //this->enableAltimeterLogging();
  //this->enableStabilizerLogging();
  //this->enableBatteryLogging();
  //this->enableActuatorLogging();
  //this->enableMotorLogging();
  //this->enableSMRM_rollLogging();

   this->enableExtPosLogging();
  
  return true;
}

bool CCrazyflie::stopLogging() {
  this->disableGyroscopeLogging();
  this->disableAccelerometerLogging();
  this->disableMagnetometerLogging();
  this->disableAltimeterLogging();
  this->disableStabilizerLogging();
  this->disableActuatorLogging();
  this->disableBatteryLogging();
  this->disableMotorLogging();
  this->disableSMRM_rollLogging();
  this->disableExtPosLogging();
  return true;
}

void CCrazyflie::setSendSetpoints(bool bSendSetpoints) {
  m_bSendsSetpoints = bSendSetpoints;
}

void CCrazyflie::setSendExtPosition(bool bSendExtPosition) {
	m_bSendsExtPosition = bSendExtPosition;
}

bool CCrazyflie::sendsSetpoints() {
  return m_bSendsSetpoints;
}

double CCrazyflie::sensorDoubleValue(std::string strName) {
  return m_tocLogs->doubleValue(strName);
}

uint32_t CCrazyflie::sensorTimestamp(std::string blockName) {
  return m_tocLogs->timestampValue(blockName);
}

double CCrazyflie::sensorLocalTimestamp(std::string blockName) {
	return m_tocLogs->localTimestampValue(blockName);
}

bool CCrazyflie::sensorNewData(std::string blockName)
{
	return m_tocLogs->newDataValue(blockName);
}

void CCrazyflie::disableLogging() {
  m_tocLogs->unregisterLoggingBlock("high-speed");
  m_tocLogs->unregisterLoggingBlock("low-speed");
}

void CCrazyflie::enableActuatorLogging() {
  m_tocLogs->registerLoggingBlock("actuator", 1);

  m_tocLogs->startLogging("actuator.aT", "actuator");
  m_tocLogs->startLogging("actuator.aP", "actuator");
  m_tocLogs->startLogging("actuator.aR", "actuator");
  m_tocLogs->startLogging("actuator.aY", "actuator");

}

uint32_t CCrazyflie::actuatorTimestamp() {
	return this->sensorTimestamp("actuator");
}

double CCrazyflie::actuatorLocalTimestamp() {
	return this->sensorLocalTimestamp("actuator");
}

bool CCrazyflie::actuatorNewData()
{
	return this->sensorNewData("actuator");
}

float CCrazyflie::actuatorRoll() {
	return this->sensorDoubleValue("actuator.aR");
}

float CCrazyflie::actuatorPitch() {
	return this->sensorDoubleValue("actuator.aP");
}

float CCrazyflie::actuatorYaw() {
	return this->sensorDoubleValue("actuator.aY");
}

float CCrazyflie::actuatorThrust() {
	return this->sensorDoubleValue("actuator.aT");
}

void CCrazyflie::disableActuatorLogging() {
	m_tocLogs->unregisterLoggingBlock("actuator");
}

void CCrazyflie::enableStabilizerLogging() {
	m_tocLogs->registerLoggingBlock("stabilizer", 1);

	m_tocLogs->startLogging("stabilizer.roll", "stabilizer");
	m_tocLogs->startLogging("stabilizer.pitch", "stabilizer");
	m_tocLogs->startLogging("stabilizer.yaw", "stabilizer");
}

uint32_t CCrazyflie::stabTimestamp() {
	return this->sensorTimestamp("stabilizer");
}

double CCrazyflie::stabLocalTimestamp() {
	return this->sensorLocalTimestamp("stabilizer");
}

bool CCrazyflie::stabNewData()
{
	return this->sensorNewData("stabilizer");
}

float CCrazyflie::stabRoll() {
	return this->sensorDoubleValue("stabilizer.roll");
}

float CCrazyflie::stabPitch() {
	return this->sensorDoubleValue("stabilizer.pitch");
}

float CCrazyflie::stabYaw() {
	return this->sensorDoubleValue("stabilizer.yaw");
}

int CCrazyflie::stabThrust() {
	return this->sensorDoubleValue("stabilizer.thrust");
}

void CCrazyflie::disableStabilizerLogging() {
	m_tocLogs->unregisterLoggingBlock("stabilizer");
}

void CCrazyflie::enableGyroscopeLogging() {
  m_tocLogs->registerLoggingBlock("gyroscope", 1);
  m_tocLogs->startLogging("gyro.x", "gyroscope");
  m_tocLogs->startLogging("gyro.y", "gyroscope");
  m_tocLogs->startLogging("gyro.z", "gyroscope");
}

uint32_t CCrazyflie::gyroTimestamp(){
  return this->sensorTimestamp("gyroscope");
}

double CCrazyflie::gyroLocalTimestamp() {
	return this->sensorLocalTimestamp("gyroscope");
}

bool CCrazyflie::gyroNewData()
{
	return this->sensorNewData("gyroscope");
}

float CCrazyflie::gyroX() {
  return this->sensorDoubleValue("gyro.x");
}

float CCrazyflie::gyroY() {
  return this->sensorDoubleValue("gyro.y");
}

float CCrazyflie::gyroZ() {
  return this->sensorDoubleValue("gyro.z");
}

void CCrazyflie::disableGyroscopeLogging() {
	m_tocLogs->unregisterLoggingBlock("gyroscope");
}

void CCrazyflie::enableAccelerometerLogging() {
  m_tocLogs->registerLoggingBlock("accelerometer", 1);

  m_tocLogs->startLogging("acc.x", "accelerometer");
  m_tocLogs->startLogging("acc.y", "accelerometer");
  m_tocLogs->startLogging("acc.z", "accelerometer");
 // m_tocLogs->startLogging("acc.zw", "accelerometer");
 // m_tocLogs->startLogging("acc.mag2", "accelerometer");
}

uint32_t CCrazyflie::accTimestamp(){
  return this->sensorTimestamp("accelerometer");
}

double CCrazyflie::accLocalTimestamp() {
	return this->sensorLocalTimestamp("accelerometer");
}

bool CCrazyflie::accNewData()
{
	return this->sensorNewData("accelerometer");
}

float CCrazyflie::accX() {
  return this->sensorDoubleValue("acc.x");
}

float CCrazyflie::accY() {
  return this->sensorDoubleValue("acc.y");
}

float CCrazyflie::accZ() {
  return this->sensorDoubleValue("acc.z");
}

float CCrazyflie::accZW() {
  return this->sensorDoubleValue("acc.zw");
}

float CCrazyflie::accMag2() {
	return this->sensorDoubleValue("acc.mag2");
}

void CCrazyflie::disableAccelerometerLogging() {
  m_tocLogs->unregisterLoggingBlock("accelerometer");
}

void CCrazyflie::enableBatteryLogging() {
  m_tocLogs->registerLoggingBlock("battery", 1);

  m_tocLogs->startLogging("pm.vbat", "battery");
  m_tocLogs->startLogging("pm.state", "battery");
  m_tocLogs->startLogging("pm.chargeCurrent", "battery");
}

uint32_t CCrazyflie::batTimestamp() {
	return this->sensorTimestamp("battery");
}

double CCrazyflie::batLocalTimestamp() {
	return this->sensorLocalTimestamp("battery");
}

bool CCrazyflie::batNewData()
{
	return this->sensorNewData("battery");
}

double CCrazyflie::batLevel() {
  return this->sensorDoubleValue("pm.vbat");
}

float CCrazyflie::batState() {
  return this->sensorDoubleValue("pm.state");
}

float CCrazyflie::batChargeCurrent() {
	return this->sensorDoubleValue("pm.chargeCurrent");
}

void CCrazyflie::disableBatteryLogging() {
  m_tocLogs->unregisterLoggingBlock("battery");
}

void CCrazyflie::enableMagnetometerLogging() {
  m_tocLogs->registerLoggingBlock("magnetometer", 1);

  m_tocLogs->startLogging("mag.x", "magnetometer");
  m_tocLogs->startLogging("mag.y", "magnetometer");
  m_tocLogs->startLogging("mag.z", "magnetometer");
}

uint32_t CCrazyflie::magTimestamp(){
  return this->sensorTimestamp("magnetometer");
}

double CCrazyflie::magLocalTimestamp() {
	return this->sensorLocalTimestamp("magnetometer");
}

bool CCrazyflie::magNewData()
{
	return this->sensorNewData("magnetometer");
}

float CCrazyflie::magX() {
  return this->sensorDoubleValue("mag.x");
}

float CCrazyflie::magY() {
  return this->sensorDoubleValue("mag.y");
}

float CCrazyflie::magZ() {
  return this->sensorDoubleValue("mag.z");
}

void CCrazyflie::disableMagnetometerLogging() {
  m_tocLogs->unregisterLoggingBlock("magnetometer");
}

void CCrazyflie::enableMotorLogging() {
  m_tocLogs->registerLoggingBlock("motor", 1);
  m_tocLogs->startLogging("motor.m1", "motor");
  m_tocLogs->startLogging("motor.m2", "motor");
  m_tocLogs->startLogging("motor.m3", "motor");
  m_tocLogs->startLogging("motor.m4", "motor");
}

uint32_t CCrazyflie::motorTimestamp(){
  return this->sensorTimestamp("motor");
}

double CCrazyflie::motorLocalTimestamp() {
	return this->sensorLocalTimestamp("motor");
}

bool CCrazyflie::motorNewData()
{
	return this->sensorNewData("motor");
}

float CCrazyflie::motor1() {
  return this->sensorDoubleValue("motor.m1");
}

float CCrazyflie::motor2() {
  return this->sensorDoubleValue("motor.m2");
}

float CCrazyflie::motor3() {
  return this->sensorDoubleValue("motor.m3");
}

float CCrazyflie::motor4() {
  return this->sensorDoubleValue("motor.m4");
}

void CCrazyflie::disableMotorLogging() {
  m_tocLogs->unregisterLoggingBlock("motor");
}

void CCrazyflie::enableAltimeterLogging() {
  m_tocLogs->registerLoggingBlock("barometer", 1);
  m_tocLogs->startLogging("baro.asl", "barometer");
 // m_tocLogs->startLogging("baro.aslLong", "barometer");
 // m_tocLogs->startLogging("baro.aslRaw", "barometer");
  m_tocLogs->startLogging("baro.pressure", "barometer");
  m_tocLogs->startLogging("baro.temp", "barometer");
}

uint32_t CCrazyflie::baroTimestamp(){
  return this->sensorTimestamp("barometer");
}

double CCrazyflie::baroLocalTimestamp() {
	return this->sensorLocalTimestamp("barometer");
}

bool CCrazyflie::baroNewData()
{
	return this->sensorNewData("barometer");
}

float CCrazyflie::baroAsl() {
  return this->sensorDoubleValue("baro.asl");
}

float CCrazyflie::baroAslRaw() {
  return this->sensorDoubleValue("baro.aslRaw");
}

float CCrazyflie::baroAslLong() {
  return this->sensorDoubleValue("baro.aslLong");
}

float CCrazyflie::baroPressure() {
  return this->sensorDoubleValue("baro.pressure");
}

float CCrazyflie::baroTemp() {
  return this->sensorDoubleValue("baro.temp");
}

void CCrazyflie::disableAltimeterLogging() {
  m_tocLogs->unregisterLoggingBlock("barometer");
}

void CCrazyflie::enableSMRM_rollLogging() {
	m_tocLogs->registerLoggingBlock("SMRM_roll", 1000);

	m_tocLogs->startLogging("SMRM_roll.x_hat", "SMRM_roll");
	m_tocLogs->startLogging("SMRM_roll.v_hat", "SMRM_roll");
	m_tocLogs->startLogging("SMRM_roll.th_hat", "SMRM_roll");
	m_tocLogs->startLogging("SMRM_roll.om_hat", "SMRM_roll");

}

uint32_t CCrazyflie::SMRM_rollTimestamp() {
	return this->sensorTimestamp("SMRM_roll");
}

double CCrazyflie::SMRM_rollLocalTimestamp() {
	return this->sensorLocalTimestamp("SMRM_roll");
}

bool CCrazyflie::SMRM_rollNewData()
{
	return this->sensorNewData("SMRM_roll");
}

float CCrazyflie::SMRM_rollX_hat() {
	return this->sensorDoubleValue("SMRM_roll.x_hat");
}

float CCrazyflie::SMRM_rollV_hat() {
	return this->sensorDoubleValue("SMRM_roll.v_hat");
}

float CCrazyflie::SMRM_rollTh_hat() {
	return this->sensorDoubleValue("SMRM_roll.th_hat");
}

float CCrazyflie::SMRM_rollOm_hat() {
	return this->sensorDoubleValue("SMRM_roll.om_hat");
}

void CCrazyflie::disableSMRM_rollLogging() {
	m_tocLogs->unregisterLoggingBlock("SMRM_roll");
}

void CCrazyflie::enableExtPosLogging()
{
	m_tocLogs->registerLoggingBlock("ext_pos", 1);
	m_tocLogs->startLogging("ext_pos.X", "ext_pos");
	m_tocLogs->startLogging("ext_pos.Y", "ext_pos");
	m_tocLogs->startLogging("ext_pos.Z", "ext_pos");	
}

uint32_t CCrazyflie::extPosTimestamp() {
	return this->sensorTimestamp("ext_pos");
}

double CCrazyflie::extPosLocalTimestamp() {
	return this->sensorLocalTimestamp("ext_pos");
}

bool CCrazyflie::extPosNewData()
{
	return this->sensorNewData("ext_pos");
}

float CCrazyflie::extPosX() {
	return this->sensorDoubleValue("ext_pos.X");
}

float CCrazyflie::extPosY() {
	return this->sensorDoubleValue("ext_pos.Y");
}

float CCrazyflie::extPosZ() {
	return this->sensorDoubleValue("ext_pos.Z");
}

void CCrazyflie::disableExtPosLogging()
{
	m_tocLogs->unregisterLoggingBlock("ext_pos");
}


