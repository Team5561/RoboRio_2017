/*
  RobotConstCal.hpp

 */
#ifndef SRC_ROBOTCONSTCAL_HPP_
#define SRC_ROBOTCONSTCAL_HPP_

// Set to ROBOT 1 for the competition bot and ROBOT 2 for the practice bot.  Either should work for the bench.
#define ROBOT1
// Set GYRO here:  GYRONO = no gyro GYRO1 = Analog gyro GYRO2 = navX
#define GYRO1
// Set vision on/off here.  To turn on, set VISION.  To turn off, set VISONNO
#define VISION

#include "WPILib.h"
#ifdef GYRO2
#include "AHRS.h"
#endif
#include "CANTalon.h"
#include "CameraServer.h"
#include "DriverStation.h"
#include <math.h>
#include <CameraServer.h>
#include <IterativeRobot.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#ifdef GYRO1
#include "ADXRS450_Gyro.h"
#endif


/* Enum definitions: */
 typedef enum
 {
   C_AutonPositionLeft,
   C_AutonPositionMiddle,
   C_AutonPositionRight,
   C_AutonPositionDefault,
   C_AutonPositionSz
 } AutonPosition;

 typedef enum
 {
   C_JoystickX_FwdOnly,
   C_JoystickX_RvrsOnly,
   C_JoystickY_FwdOnly,
   C_JoystickY_RvrsOnly,
   C_JoystickDefault
 }JoystickLock;

 typedef enum
 {
   C_Disabled,
   C_Auton,
   C_Teleop,
   C_Test,
   C_Null
 }RoboState;

 typedef enum
 {
   LED_Color_Red,
   LED_Color_Blue,
   LED_Color_Green,
   LED_Color_White,
   LED_Color_Purple,
   LED_Color_Yellow,
   LED_Color_Pink,
   LED_Color_Orange,
   LED_Color_Rainbow, // This is meant to indicate when a random mixture of colors are desired
   LED_Color_Multi,   // This is meant to indicate when it is desired to have the colors cycle through all of the avaiable colors above rainbow
   LED_Color_Black    // This is more of an "off mode", must remain at end of enum
 } LED_Color;

 typedef enum
 {
   LED_Mode0,
   LED_Mode1,
   LED_Mode2,
   LED_Mode3,
   LED_Mode4,
   LED_Mode5,
   LED_Mode6,
   LED_Mode7,
   LED_ModeSz
 } LED_Mode;

 typedef enum
 {
   C_Auton3_Off,
   C_Auton3_Forward1,
   C_Auton3_Rotate,
   C_Auton3_Forward2,
   C_Auton3_CenterSearch,
   C_Auton3_Forward3
 } Auton_Mode3;

 typedef enum
 {
   Auton_Mode3_CenterSearchOff,
   Auton_Mode3_CenterSearchWaitingForCamera,
   Auton_Mode3_CenterSearchMoving,
   Auton_Mode3_CenterSearchRotate,
   Auton_Mode3_CenterSearchComplete
 } Auton_Mode3_CenterSearch;

extern double        V_DriveX_Auton;
extern double        V_DriveY_Auton;
extern double        V_DriveZ_Auton;
extern bool          V_AutonCameraLED;
extern Auton_Mode3   V_AutonMode3;
extern double        V_Distance1;
extern double        V_Distance2;
extern double        V_Area1;
extern double        V_Area2;
extern double        V_Height1;
extern double        V_Height2;
extern double        V_BlobsDetected;
extern double        V_Center;
extern double        V_CenterRaw;
extern bool          V_AutonFieldOriented;
extern AutonPosition V_AutonPosition;
extern double        V_AutonMode3_CenterSearchError;
extern bool          V_AutonLightReq;
extern bool          V_CameraLED;
extern std::shared_ptr<NetworkTable> table;

void AutonStatesInit(void);

void AutonOption3(double      L_GyroAngle,
                  double      L_YawRate,
                  double      L_X_Velocity,
                  double      L_X_Accel,
                  double      L_Y_Velocity,
                  double      L_Y_Accel);

void AutonDefault(double      L_MatchTime);

double CenterPoint(double L_Distance1, double L_Distnce2, double L_BlobsDetected);

void CameraProcessing(void);

/* Constants */
  const double C_ControllerUpdateRate = 0.01;  // Execution rate of the Roborio controller

  const double C_ImageOutsideRange    = 1000;

  const int   C_USB_Cam = 0;
  const int   C_USB_Cam2 = 1;

  const int   C_XB_POV = 0;       //XBox POV
  const int   C_XB_A_Btn = 1;
  const int   C_XB_B_Btn = 2;
  const int   C_XB_X_Btn = 3;
  const int   C_XB_Y_Btn = 4;
  const int   C_XB_L_Bumper = 5;
  const int   C_XB_R_Bumper = 6;
  const int   C_XB_BackBtn = 7;
  const int   C_XB_StartBtn = 8;
  const int   C_XB_L_StickPush = 9;
  const int   C_XB_R_StickPush = 10;
  const int   C_XB_JoystickLX = 0;
  const int   C_XB_JoystickLY = 1;
  const int   C_XB_JoystickRX = 4;
  const int   C_XB_JoystickRY = 5;  //XBox Right Y button
  const int   C_XB_RTrigger = 3;    //XBox Right trigger

  const int   C_Lgt_JoystickX = 0;  //Logitech X axis
  const int   C_Lgt_JoystickY = 1;
  const int   C_Lgt_JoystickZ = 2;
  const int   C_Lgt_POV = 0;
  const int   C_Lgt_Trigger = 1;
  const int   C_Lgt_Btn2 = 2;
  const int   C_Lgt_Btn3 = 3;
  const int   C_Lgt_Btn4 = 4;
  const int   C_Lgt_Btn5 = 5;
  const int   C_Lgt_Btn6 = 6;
  const int   C_Lgt_Btn7 = 7;
  const int   C_Lgt_Btn8 = 8;
  const int   C_Lgt_Btn9 = 9;
  const int   C_Lgt_Btn10 = 10;
  const int   C_Lgt_Btn11 = 11;
  const int   C_Lgt_Btn12 = 12;

/* Calibrations, these can be tweaked */
  const double K_EndMatchWarningTime  =  30;    // This is the expected time remaining that the robot will warn the driver that the end of the match is near.  This will change the light output and signal to the driver station.
  const double K_JoystickDirectionDbX =   0.20; // Dead band applied to the X axis of the joystick.
  const double K_JoystickDirectionDbY =   0.20; // Dead band applied to the Y axis of the joystick.
  const double K_JoystickDirectionDbZ =   0.20; // Dead band applied to the Z axis of the joystick.
  const double K_MaxFwdSpeedPOV       =   1.0;
  const double K_MaxRvrsSpeedPOV      =  -1.0;
  const double K_MaxLeftSpeedPOV      =  -1.0;
  const double K_MaxRightSpeedPOV     =   1.0;
  const double K_IndexerFwdSpd        =   1.0;
  const double K_IndexerRvrsSpd       =  -1.0;
  const double K_DriveGainMin         =   0.25;
  const double K_DriveGainMax         =   1.0;
  const double K_DriveLagCoeffX       =   0.7; // First order lag coefficent for the X driven direction
  const double K_DriveLagCoeffY       =   0.7; // First order lag coefficent for the X driven direction
  const double K_DriveLagCoeffZ       =   0.8; // First order lag coefficent for the X driven direction
  const double K_LED_WinchOnTime      =   3.0;  // This is the amount of accumulated time that the winch needs to be commanded on at the end of the game in order to trigger the rainbow effect
  const double K_WinchOnThreshold     =   0.1; // Threshold above which the winch is considered to be on.

  /* Auton Calibrations: */
  const double K_AutonLagCoeffX       =   1.0; // First order lag coefficent for the X driven direction, for auton
  const double K_AutonLagCoeffY       =   1.0; // First order lag coefficent for the X driven direction, for auton
  const double K_AutonLagCoeffZ       =   1.0; // First order lag coefficent for the X driven direction, for auton

#ifdef ROBOT1
  const double K_RotationGain         =    0.05;
#else
  const double K_RotationGain         =    -0.05;
#endif
  const double K_RotationMinPos       =    0.3;
  const double K_RotationMinNeg       =   -0.3;
  const double K_MaxPosRotation       =    0.4;
  const double K_MaxNegRotation       =   -0.4;
  const double K_RotationErrorDb      =    2.0; // Deadband for angle, in degrees
  const double K_YawRateStopThresh    =    0.1;

  const double K_X_AccelStopThresh    =     0.2;

  const double K_Y_AccelStopThresh    =    0.2;

  const double K_StopWaitTime         =    0.5;
  const double K_CntrlDebounceTime    =    0.05;

  const double K_CameraCenterDB_High    =   5;
  const double K_CameraCenterDB_Low     =   5;
  const double K_CameraCenterDB_Time    =   0.3;
  const double K_CameraExpectedBlobCount = 2;

#ifdef ROBOT1
  const double K_Auton3_ForwardTime[C_AutonPositionSz] =
      {
        0.90, /* C_AutonPositionLeft    */
        0.9, /* C_AutonPositionMiddle  */
        0.90, /* C_AutonPositionRight   */
       10.00  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardSpd1[C_AutonPositionSz] =
      {
        0.3, /* C_AutonPositionLeft    */
        0.3, /* C_AutonPositionMiddle  */
        0.3, /* C_AutonPositionRight   */
        0.2  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardTime2[C_AutonPositionSz] =
      {
        0.79, /* C_AutonPositionLeft    */
        0.0, /* C_AutonPositionMiddle  */
        0.79, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardSpd2[C_AutonPositionSz] =
      {
        0.3, /* C_AutonPositionLeft    */
        0.3, /* C_AutonPositionMiddle  */
        0.3, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardTime3[C_AutonPositionSz] =
      {
        0.3, /* C_AutonPositionLeft    */
        0.3, /* C_AutonPositionMiddle  */
        0.3, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardSpd3[C_AutonPositionSz] =
      {
        0.3, /* C_AutonPositionLeft    */
        0.3, /* C_AutonPositionMiddle  */
        0.3, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };

  const double K_Auton3_SearchTimeGain[C_AutonPositionSz] =
      {
        0.004, /* C_AutonPositionLeft    */
        0.0035, /* C_AutonPositionMiddle  */
        0.004, /* C_AutonPositionRight   */
        0.0   /* C_AutonPositionDefault */
      };

  const double K_Auton3_SearchTime = 0.3;
  const double K_Auton3_SearchPos = 0.4;
  const double K_Auton3_SearchNeg = -0.4;

  const double K_Auton3_Angle[C_AutonPositionSz] =
      {
        57.0, /* C_AutonPositionLeft    */
        0.0, /* C_AutonPositionMiddle  */
       -57.0, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };
#else
  const double K_Auton3_ForwardTime[C_AutonPositionSz] =
      {
        2.5, /* C_AutonPositionLeft   2.34 */
        2.00, /* C_AutonPositionMiddle  */
        2.5, /* C_AutonPositionRight   */
       12.00  /* C_AutonPositionDefault */
      };
//  const double K_Auton3_ForwardTime[C_AutonPositionSz] =
//      {
//        2.00, /* C_AutonPositionLeft    */
//        1.00, /* C_AutonPositionMiddle  */
//        1.34, /* C_AutonPositionRight   */
//       2.00  /* C_AutonPositionDefault */
//      };

  const double K_Auton3_ForwardSpd[C_AutonPositionSz] =
      {
        0.3, /* C_AutonPositionLeft    */
        0.3, /* C_AutonPositionMiddle  */
        0.3, /* C_AutonPositionRight   */
        0.4  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardTime2[C_AutonPositionSz] =
      {
        1.3, /* C_AutonPositionLeft    */
        0.0, /* C_AutonPositionMiddle  */
        1.3, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };

  const double K_Auton3_ForwardTime3[C_AutonPositionSz] =
      {
        0.6, /* C_AutonPositionLeft    */
        0.4, /* C_AutonPositionMiddle  */
        0.6, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };


  const double K_Auton3_SearchTimeGain[C_AutonPositionSz] =
      {
        0.002, /* C_AutonPositionLeft    */
        0.002, /* C_AutonPositionMiddle  */
        0.002, /* C_AutonPositionRight   */
        0.0   /* C_AutonPositionDefault */
      };

  const double K_Auton3_SearchTime =   0.3;

  const double K_Auton3_SearchPos = 0.6;
  const double K_Auton3_SearchNeg = -0.6;
  const double K_Auton3_Angle[C_AutonPositionSz] =
      {
       57.0, /* C_AutonPositionLeft    */
        0.0, /* C_AutonPositionMiddle  */
      -57.0, /* C_AutonPositionRight   */
        0.0  /* C_AutonPositionDefault */
      };
#endif

const double K_Auton3_MinSearchTm[C_AutonPositionSz] =
    {
      0.1, /* C_AutonPositionLeft    */
      0.1, /* C_AutonPositionMiddle  */
      0.1, /* C_AutonPositionRight   */
      0.0  /* C_AutonPositionDefault */
    };

  const double K_Auton3_CenterDeadbandHigh = 346;
  const double K_Auton3_CenterDeadbandLow  = 336;
  const double K_Auton3_DsrdCenter         = 342;  // Center location, in pixels 267




#endif /* SRC_ROBOTCONSTCAL_HPP_ */
