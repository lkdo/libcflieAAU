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


#ifndef __C_CRAZYFLIE_H__
#define __C_CRAZYFLIE_H__

// System
#include <cmath>

// Private
#include "CCrazyRadio.h"
#include "CTOC.h"


enum State {
  STATE_ZERO = 0,
  STATE_READ_PARAMETERS_TOC = 1,
  STATE_READ_LOGS_TOC = 2,
  STATE_START_LOGGING = 3,
  STATE_ZERO_MEASUREMENTS = 4,
  STATE_NORMAL_OPERATION = 5
};

/*! \brief Crazyflie Nano convenience controller class

  The class containing the mechanisms for starting sensor readings,
  ordering set point setting, selecting and running controllers and
  calculating information based on the current sensor readings. */
class CCrazyflie {
 private:
  // Variables
  int m_nAckMissTolerance;
  int m_nAckMissCounter;
  /*! \brief Internal pointer to the initialized CCrazyRadio radio
      interface instance. */
  CCrazyRadio *m_crRadio;
  /*! \brief The current thrust to send as a set point to the
      copter. */
  int m_nThrust;
  /*! \brief The current roll to send as a set point to the copter. */
  float m_fRoll;
  /*! \brief The current pitch to send as a set point to the
      copter. */
  float m_fPitch;
  /*! \brief The current yaw to send as a set point to the copter. */
  float m_fYaw;
  /*! \brief The current desired control set point (position/yaw to
      reach) */
  /*! \brief The current external position data (to be sent to 
       the quadrotor) */
  float m_fX; 
  float m_fY;
  float m_fZ;

  // Control related parameters
  /*! \brief Maximum absolute value for the roll that will be sent to
      the copter. */
  float m_fMaxAbsRoll;
  /*! \brief Maximum absolute value for the pitch that will be sent to
      the copter. */
  float m_fMaxAbsPitch;
  /*! \brief Maximum absolute value for the yaw that will be sent to
      the copter. */
  float m_fMaxYaw;
  /*! \brief Maximum thrust that will be sent to the copter. */
  int m_nMaxThrust;
  /*! \brief Minimum thrust that will be sent to the copter. */
  int m_nMinThrust;
  double m_dSendSetpointPeriod;
  double m_dSetpointLastSent;
  bool m_bSendsSetpoints;
  bool m_bSendsExtPosition;
  double m_dExtPositionLastSent;
  double m_dSendExtPositionMinPeriod;
  bool m_bNewExtPosition;
  CTOC *m_tocParameters;
  CTOC *m_tocLogs;
  enum State m_enumState;

  // Functions
  bool readTOCParameters();
  bool readTOCLogs();

  /*! \brief Send a set point to the copter controller

    Send the set point for the internal copter controllers. The
    copter will then try to achieve the given roll, pitch, yaw and
    thrust. These values can be set manually but are managed by the
    herein available controller(s) if one is switched on to reach
    desired positions.

    \param fRoll The desired roll value.
    \param fPitch The desired pitch value.
    \param fYaw The desired yaw value.
    \param sThrust The desired thrust value.
    \return Boolean value denoting whether or not the command could be sent successfully. */
  bool sendSetpoint(float fRoll, float fPitch, float fYaw, short sThrust);
  
  bool sendExtPosition(float fX, float fY, float fZ);

  void disableLogging();
  bool startLogging();
  bool stopLogging();

  void sendFlightModeParameters();

  void enableAccelerometerLogging();
  void disableAccelerometerLogging();

  void enableStabilizerLogging();
  void disableStabilizerLogging();

  void enableActuatorLogging();
  void disableActuatorLogging();

  void enableGyroscopeLogging();
  void disableGyroscopeLogging();

  void enableBatteryLogging();
  void disableBatteryLogging();
  
  void enableMagnetometerLogging();
  void disableMagnetometerLogging();

  void enableAltimeterLogging();
  void disableAltimeterLogging();

  void enableMotorLogging();
  void disableMotorLogging();
  
  void enableSMRM_rollLogging();
  void disableSMRM_rollLogging();

  void enableExtPosLogging();
  void disableExtPosLogging();

 public:
  /*! \brief Constructor for the copter convenience class

    Constructor for the CCrazyflie class, taking a CCrazyRadio radio
    interface instance as a parameter.

    \param crRadio Initialized (and started) instance of the
    CCrazyRadio class, denoting the USB dongle to communicate
    with. */
  CCrazyflie(CCrazyRadio *crRadio);
  /*! \brief Destructor for the copter convenience class

    Destructor, deleting all internal variables (except for the
    CCrazyRadio radio instance given in the constructor). */
  ~CCrazyflie();

  /*! \brief Set the thrust control set point

    The thrust value that will be sent to the internal copter
    controller as a set point.

    \param nThrust The thrust value to send (> 10000) */
  void setThrust(int nThrust);
  
  /*! \brief Set the roll control set point

    The roll value that will be sent to the internal copter
    controller as a set point.

    \param fRoll The roll value to send */
  void setRoll(float fRoll);
  
  /*! \brief Set the pitch control set point

    The pitch value that will be sent to the internal copter
    controller as a set point.

    \param fPitch The pitch value to send */
  void setPitch(float fPitch);
  
  /*! \brief Set the yaw control set point

    The yaw value that will be sent to the internal copter
    controller as a set point.

    \param fYaw The yaw value to send */
  void setYaw(float fYaw);
  
  /*! \brief Set the external position data
     */ 
  void setExtPosition(float X, float Y, float Z);

  /*! \brief Manages internal calculation operations

    Should be called during every 'cycle' of the main program using
    this class. Things like sensor reading processing, integral
    calculation and controller signal application are performed
    here. This function also triggers communication with the
    copter. Not calling it for too long will cause a disconnect from
    the copter's radio.

    \return Returns a boolean value denoting the current status of the
    radio dongle. If it returns 'false', the dongle was most likely
    removed or somehow else disconnected from the host machine. If it
    returns 'true', the dongle connection works fine. */
  bool cycle();
  /*! \brief Signals whether the copter is in range or not

    Returns whether the radio connection to the copter is currently
    active.

    \return Returns 'true' is the copter is in range and radio
    communication works, and 'false' if the copter is either out of
    range or is switched off. */
  bool copterInRange();

  /*! \brief Whether or not the copter was initialized successfully.

    \returns Boolean value denoting the initialization status of the
    copter communication. */
  bool isInitialized();

  /*! \brief Set whether setpoints are currently sent while cycle()

    While performing the cycle() function, the currently set setpoint
    is sent to the copter regularly. If this is not the case, dummy
    packets are sent. Using this mechanism, you can effectively switch
    off sending new commands to the copter.

    Default value: `false`

    \param bSendSetpoints When set to `true`, the current setpoint is
    sent while cycle(). Otherwise, not. */
  void setSendSetpoints(bool bSendSetpoints);

  /*! \brief Set whether external position data  is sent while cycle()

    While performing the cycle() function, choose if the external 
    position datais sent to the copter regularly.

    Default value: `false`

    \param bSendSetpoints When set to `true`, the current position 
    data is sent while cycle(). Otherwise, not. */
  void setSendExtPosition(bool bSendExtPosition);

  /*! \brief Whether or not setpoints are currently sent to the copter

    \return Boolean value denoting whether or not the current setpoint
    is sent to the copter while performing cycle(). */
  bool sendsSetpoints();

  /*! \brief Read back a sensor value you subscribed to

    Possible sensor values might be:
    * stabilizer.yaw
    * stabilizer.roll
    * stabilizer.pitch
    * pm.vbat

    The possible key names strongly depend on your firmware. If you
    don't know what to do with this, just use the convience functions
    like roll(), pitch(), yaw(), and batteryLevel().

    \return Double value denoting the current value of the requested
    log variable. */
  double sensorDoubleValue(std::string strName);

  uint32_t sensorTimestamp(std::string blockName);
  double sensorLocalTimestamp(std::string blockName);
  bool sensorNewData(std::string blockName);

  uint32_t accTimestamp();
  double accLocalTimestamp();
  bool accNewData();
  float accX();
  float accY();
  float accZ();
  float accZW();
  float accMag2();

  uint32_t baroTimestamp();
  double baroLocalTimestamp();
  bool baroNewData();
  float baroAsl();
  float baroAslLong();
  float baroAslRaw();
  float baroTemp();
  float baroPressure();

  uint32_t gyroTimestamp();
  double gyroLocalTimestamp();
  bool gyroNewData();
  float gyroX();
  float gyroY();
  float gyroZ();

  uint32_t batTimestamp();
  double batLocalTimestamp();
  bool batNewData();
  float batState();
  float batChargeCurrent();
  double batLevel();

  uint32_t magTimestamp();
  double magLocalTimestamp();
  bool magNewData();
  float magX();
  float magY();
  float magZ();

  uint32_t motorTimestamp();
  double motorLocalTimestamp();
  bool motorNewData();
  float motor1();
  float motor2();
  float motor3();
  float motor4();

  uint32_t stabTimestamp();
  double stabLocalTimestamp();
  bool stabNewData();
  int stabThrust();
  float stabRoll();
  float stabPitch();
  float stabYaw();

  uint32_t actuatorTimestamp();
  double actuatorLocalTimestamp();
  bool actuatorNewData();
  float actuatorThrust();
  float actuatorRoll();
  float actuatorPitch();
  float actuatorYaw();
  
  uint32_t SMRM_rollTimestamp();
  double SMRM_rollLocalTimestamp();
  bool SMRM_rollNewData();
  float SMRM_rollX_hat();
  float SMRM_rollV_hat();
  float SMRM_rollTh_hat();
  float SMRM_rollOm_hat();

  uint32_t extPosTimestamp();
  double extPosLocalTimestamp();
  bool extPosNewData();
  float extPosX();
  float extPosY();
  float extPosZ();
};


#endif /* __C_CRAZYFLIE_H__ */
