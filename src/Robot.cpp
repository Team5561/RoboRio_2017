#include "RobotConstCal.hpp"


JoystickLock  V_JoystickLock;
double        V_DriveX_Prev;
double        V_DriveY_Prev;
double        V_DriveZ_Prev;
double        V_DriveX_Auton;
double        V_DriveY_Auton;
double        V_DriveZ_Auton;
bool          V_AutonCameraLED;
double        V_EndGameWinchTime;
bool          V_LED_RainbowLatch;
int           V_AutonState;
AutonPosition V_AutonPosition;
#ifdef VISION
std::shared_ptr<NetworkTable> table;
#endif



class MecanumDefaultCode : public IterativeRobot
{
#ifdef ROBOT1
  CANTalon V_LeftFrontMotor;  // Left front
  CANTalon V_LeftRearMotor;   // Left rear
  CANTalon V_RightFrontMotor; // Right front
  CANTalon V_RightRearMotor;  // Right rear
#else
  Talon V_LeftFrontMotor;
  Talon V_LeftRearMotor;
  Talon V_RightFrontMotor;
  Talon V_RightRearMotor;
#endif



public:
  RobotDrive *m_robotDrive;
  Joystick V_Logitech;
  Joystick V_XboxDrive;
  BuiltInAccelerometer V_Accel;
#ifdef GYRO2
  AHRS *ahrs;
#endif
  Talon V_Winch;
  Talon V_GearIndexer;
  DigitalOutput V_LED_State0;
  DigitalOutput V_LED_State1;
  DigitalOutput V_LED_State2;
  LiveWindow *V_Lw;
#ifdef GYRO1
  ADXRS450_Gyro V_GyroOne;
#endif
//  Relay *CameraLED;


  frc::SendableChooser<std::string> V_AutonOption;
  const std::string C_AutonDefault = "Default";
  const std::string C_AutonLeft   = "Left";
  const std::string C_AutonMiddle = "Middle";
  const std::string C_AutonRight  = "Right";
  std::string V_AutonSelected;


	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */
#ifdef ROBOT1
#ifdef GYRO1
	MecanumDefaultCode(void) : V_LeftFrontMotor(1), V_LeftRearMotor(2), V_RightFrontMotor(3), V_RightRearMotor(4), V_GearIndexer(0), V_Winch(1), V_XboxDrive(0), V_Logitech(1), V_LED_State0(0), V_LED_State1(1), V_LED_State2(2), V_Lw(NULL), V_GyroOne(SPI::Port::kOnboardCS0)
#endif
#ifdef GYRONO
  MecanumDefaultCode(void) : V_LeftFrontMotor(1), V_LeftRearMotor(2), V_RightFrontMotor(3), V_RightRearMotor(4), V_GearIndexer(0), V_Winch(1), V_XboxDrive(0), V_Logitech(1), V_LED_State0(0), V_LED_State1(1), V_LED_State2(2), V_Lw(NULL)
#endif
#else
  MecanumDefaultCode(void) : V_LeftFrontMotor(0), V_LeftRearMotor(1), V_RightFrontMotor(2), V_RightRearMotor(3), V_GearIndexer(4), V_Winch(5), V_XboxDrive(0), V_Logitech(1), V_LED_State0(0), V_LED_State1(1), V_LED_State2(2), V_Lw(NULL)
#endif
	{

#ifdef VISION
//    table = NetworkTable::Initialize();
		NetworkTable::SetPort(1735);

		NetworkTable::SetTeam(5561);
		NetworkTable::SetUpdateRate(0.1);
    table = NetworkTable::GetTable("GRIP/myContoursReport");
#endif




//    CameraLED->Set(Relay::Direction::kForwardOnly);
    /* Update every motor controller to reset the motor safety timeout. */
	  V_LeftFrontMotor.Set(0);
	  V_LeftRearMotor.Set(0);
	  V_RightFrontMotor.Set(0);
	  V_RightRearMotor.Set(0);
    V_GearIndexer.Set(0);
    V_Winch.Set(0);
    m_robotDrive = new RobotDrive(V_LeftFrontMotor, V_LeftRearMotor, V_RightFrontMotor, V_RightRearMotor);
    m_robotDrive->SetExpiration(0.5);
    m_robotDrive->SetSafetyEnabled(false);
//    CameraLED = new Relay(2, Relay::Direction::kForwardOnly);
//    CameraLED->SetExpiration(0.5);
//    CameraLED->SetSafetyEnabled(false);
    // Define joystick being used at USB port #0 on the Drivers Station
	}

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Run the functions that are expected during the robot init.
 ******************************************************************************/
 void RobotInit ()
 {
//   V_Lw = LiveWindow::GetInstance();
//   CameraServer::GetInstance()->AddAxisCamera("AxisCamera");
//   cs::AxisCamera camera = CameraServer::GetInstance()->AddAxisCamera("10.55.61.11");

//   CameraServer::GetInstance()->AddAxisCamera("10.55.61.11");
//   CameraServer::GetInstance()->PutVideo("AxisCamera", 640, 480);
#ifdef GYRO1
	 V_GyroOne.Calibrate();
#endif

#ifdef GYRO2
   ahrs = new AHRS(SerialPort::Port::kUSB);
#endif


   CameraServer::GetInstance()->AddAxisCamera("AxisCamera5561","10.55.61.14");
//   CameraServer::GetInstance()->AddAxisCamera("AxisCamera5561","10.55.61.92");
//   CameraServer::GetInstance()->AddAxisCamera("AxisCamera5561");
   CameraServer::GetInstance()->StartAutomaticCapture(C_USB_Cam);
   CameraServer::GetInstance()->StartAutomaticCapture(C_USB_Cam2);

   V_AutonOption.AddDefault(C_AutonDefault, C_AutonDefault);
   V_AutonOption.AddObject(C_AutonLeft, C_AutonLeft);
   V_AutonOption.AddObject(C_AutonMiddle, C_AutonMiddle);
   V_AutonOption.AddObject(C_AutonRight, C_AutonRight);
   frc::SmartDashboard::PutData("Auto Modes", &V_AutonOption);

   AutonStatesInit();

   V_JoystickLock = C_JoystickDefault;
   V_DriveX_Prev = 0;
   V_DriveY_Prev = 0;
   V_DriveZ_Prev = 0;
   V_DriveX_Auton = 0.0;
   V_DriveY_Auton = 0.0;
   V_DriveZ_Auton = 0.0;
   V_AutonCameraLED = false;
   V_LED_RainbowLatch = false;
   V_EndGameWinchTime = 0.0;
   V_Center = 0.0;
 }



/******************************************************************************
 * Function:     DisabledPeriodic
 *
 * Description:  This code is run when the robot is disabled.
 ******************************************************************************/
  void DisabledPeriodic()
    {
      bool     L_LgtT0;
      LED_Mode L_LED_Mode;
      double   L_GyroAngle;

//      Scheduler::GetInstance()->Run();

      while(IsDisabled())
      {
        L_LgtT0  = V_Logitech.GetRawButton(C_Lgt_Trigger);

#ifdef GYRO2
        L_GyroAngle = ahrs->GetAngle();
#endif
#ifdef GYRO1
        L_GyroAngle = V_GyroOne.GetAngle();
#endif
#ifdef GYRONO
        L_GyroAngle = 0.0;
#endif

        CameraProcessing();

        UpdateActuatorCmnds(0,
                            false,
                            false,
                            0,
                            0,
                            0,
                            0,
                            0,
                            0);

        L_LED_Mode = UpdateLED_Output(C_Disabled,
                                      L_LgtT0,
                                      0.0,
                                      false);

        UpdateSmartDashboad(L_GyroAngle,
                            L_LED_Mode,
                            C_Disabled);

      Wait(C_ControllerUpdateRate);
    }
  }




/******************************************************************************
 * Function:     AutonomousInit
 *
 * Description:  Initialize the autonomous logic.
 ******************************************************************************/
void AutonomousInit()
  {
    LED_Mode L_LED_Mode;
    bool L_NavigationSensorConnected;
    V_JoystickLock = C_JoystickDefault;
    V_DriveX_Prev = 0;
    V_DriveY_Prev = 0;
    V_DriveZ_Prev = 0;
    V_LED_RainbowLatch = false;
    V_EndGameWinchTime = 0.0;

#ifdef GYRO2
    L_NavigationSensorConnected = ahrs->IsConnected();
#else
    L_NavigationSensorConnected = true;
#endif

    V_AutonSelected = V_AutonOption.GetSelected();
//    std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
    std::cout << "Auto selected: " << V_AutonSelected << std::endl;

    if (L_NavigationSensorConnected == true &&
        V_AutonSelected != C_AutonDefault)
      {
      if (V_AutonSelected == C_AutonLeft)
        {
        // Robot is on the left of the field:
        V_AutonPosition = C_AutonPositionLeft;
        }
      else if (V_AutonSelected == C_AutonMiddle)
        {
        // Robot is on the middle of the field:
        V_AutonPosition = C_AutonPositionMiddle;
        }
      else if (V_AutonSelected == C_AutonRight)
        {
        // Robot is on the right of the field:
        V_AutonPosition = C_AutonPositionRight;
        }
      }
    else
      {
      V_AutonPosition = C_AutonPositionDefault;
      }

    AutonStatesInit();

    CameraProcessing();

    UpdateActuatorCmnds(0,
                        false,
                        false,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0);

    L_LED_Mode = UpdateLED_Output(C_Auton,
                                  false,
                                  0.0,
                                  false);

    UpdateSmartDashboad(0,
                        L_LED_Mode,
                        C_Auton);
  }

/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Run the autonomous logic.
 ******************************************************************************/
void AutonomousPeriodic()
  {
    LED_Mode L_LED_Mode;
    int      L_Index;
    double   L_MatchTime;
    double   L_Distance1;
    double   L_Distance2;
    double   L_Center;
    double   L_Drive;
    double   L_GyroAngle;
    double   L_YawRate;
    double   L_X_Displacement;
    double   L_X_Velocity;
    double   L_X_Accel;
    double   L_Y_Displacement;
    double   L_Y_Velocity;
    double   L_Y_Accel;
    double   L_Error;
    double   L_RotationCmnd;
    bool     L_NavigationSensorConnected;
    bool     L_CameraLED = false;
    bool     L_FieldOriented = true;


	  while((IsAutonomous()) && (IsEnabled()))
      {
      L_GyroAngle = 0.0;
      L_MatchTime = DriverStation::GetInstance().GetMatchTime();
#ifdef GYRO2
	    L_GyroAngle = ahrs->GetAngle();
	    L_YawRate = ahrs->GetRate();
	    L_X_Displacement = ahrs->GetVelocityY();
	    L_X_Velocity = ahrs->GetVelocityX();
	    L_X_Accel = ahrs->GetWorldLinearAccelX();
	    L_Y_Displacement = ahrs->GetDisplacementY();
	    L_Y_Velocity = ahrs->GetVelocityY();
	    L_Y_Accel = ahrs->GetWorldLinearAccelY();
	    L_NavigationSensorConnected = ahrs->IsConnected();
#endif
#ifdef GYRO1
	    L_GyroAngle = V_GyroOne.GetAngle();
	    L_YawRate = 0.0;
	    L_X_Velocity = 0.0;
	    L_X_Accel = V_Accel.GetX();
	    L_Y_Velocity = 0.0;
	    L_Y_Accel = V_Accel.GetY();
	    L_NavigationSensorConnected = true;
#endif
#ifdef GYRONO
	    L_GyroAngle = 0.0;
	    L_YawRate = 0.0;
	    L_X_Velocity = 0.0;
	    L_X_Accel = 0.0;
	    L_Y_Velocity = 0.0;
	    L_Y_Accel = 0.0;
	    L_NavigationSensorConnected = false;
#endif


	    CameraProcessing();

	    if (V_AutonPosition != C_AutonPositionDefault)
	      {
	      AutonOption3(L_GyroAngle,
	                   L_YawRate,
	                   L_X_Velocity,
	                   L_X_Accel,
	                   L_Y_Velocity,
	                   L_Y_Accel);
	      }
	    else
	      {
	      AutonDefault(L_MatchTime);
	      }

      UpdateActuatorCmnds(L_GyroAngle,
                          V_AutonFieldOriented,
                          true,
                          V_DriveX_Auton,
                          V_DriveY_Auton,
                          V_DriveZ_Auton,
                          0,
                          0,
                          K_DriveGainMin);

	    L_LED_Mode = UpdateLED_Output(C_Auton,
	                                  V_AutonLightReq,
	                                  0.0,
	                                  false);

	    UpdateSmartDashboad(L_GyroAngle,
	                        L_LED_Mode,
	                        C_Auton);

	    Wait(C_ControllerUpdateRate);
      }
  }

/******************************************************************************
 * Function:     ApplyLagFilt
 *
 * Description:  Apply a first order lag filter to the desired signal.
 ******************************************************************************/
double ApplyLagFilt(double L_RawValue,
                    double L_PrevValue,
                    double L_LagCoef)
  {
    double L_Output;

    if(L_RawValue >= L_PrevValue)
      {
        L_Output = L_PrevValue + ((L_RawValue - L_PrevValue)*L_LagCoef);
      }
    else
      {
        L_Output = L_PrevValue - ((L_PrevValue - L_RawValue)*L_LagCoef);
      }

    return (L_Output);
  }

/******************************************************************************
 * Function:     ApplyDeadband
 *
 * Description:  Apply a dead band to the given signal.
 ******************************************************************************/
double ApplyDeadband(double L_RawValue,
                     double L_DeadbandThresh)
  {
    double L_Output;

		if((L_RawValue < (-L_DeadbandThresh)) ||
		   (L_RawValue > L_DeadbandThresh))
		  {
		    L_Output = L_RawValue;
		  }
		else
		  {
		    L_Output = 0;
		  }

		return (L_Output);
  }

/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  Initialize the Teleop logic.
 ******************************************************************************/
void TeleopInit()
  {
    LED_Mode L_LED_Mode;

    V_JoystickLock = C_JoystickDefault;
    V_DriveX_Prev = 0;
    V_DriveY_Prev = 0;
    V_DriveZ_Prev = 0;
    V_LED_RainbowLatch = false;
    V_EndGameWinchTime = 0.0;

    UpdateActuatorCmnds(0,
                        false,
                        false,
                        0,
                        0,
                        0,
                        0,
                        0,
                        0);

    L_LED_Mode = UpdateLED_Output(C_Teleop,
                                  false,
                                  0.0,
                                  false);

    UpdateSmartDashboad(0,
                        L_LED_Mode,
                        C_Teleop);

  }

/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Update the teleop periodic logic.  Gets called once for each
 *               new packet from the DS.
 ******************************************************************************/
	void TeleopPeriodic(void)
	{
    float  L_XB_DrvRY, L_XB_DrvLY, L_XB_DrvLX, L_XB_DrvRX, L_XB_RTrigger;
    int    L_LgtP1;
    double L_LgtX, L_LgtY, L_LgtZ;
    bool   L_LgtT0;
    bool   L_LgtB1,L_LgtB3,L_LgtB4,L_LgtB5,L_LgtB6,L_LgtB7,L_LgtB8,L_LgtB9,L_LgtB11,L_LgtB10,L_LgtB12;
    bool   L_XbxA, L_XbxB, L_XbxX, L_XbxY, L_Xbx_LB, L_Xbx_RB;
    int    L_XbxPOV;
    double L_DirectionX = 0;
    double L_DirectionY = 0;
    double L_DirectionZ = 0;
    double L_Winch;
    double L_GearIndexer;
    LED_Mode L_LED_Mode;
    double L_GyroAngle = 0;
    double L_DriveGain;

    while (IsOperatorControl() && IsEnabled())
      {
      /* Read Logitech controller commands here: */
      L_LgtX =         V_Logitech.GetRawAxis(C_Lgt_JoystickX);
      L_LgtY =         V_Logitech.GetRawAxis(C_Lgt_JoystickY);
      L_LgtZ =        -V_Logitech.GetRawAxis(C_Lgt_JoystickZ); // Axis was backwards
      L_LgtP1  =       V_Logitech.GetPOV(C_Lgt_POV);
      L_LgtT0  =       V_Logitech.GetRawButton(C_Lgt_Trigger);
      L_LgtB1  =       V_Logitech.GetRawButton(C_Lgt_Btn2);
      L_LgtB3  =       V_Logitech.GetRawButton(C_Lgt_Btn3);
      L_LgtB4  =       V_Logitech.GetRawButton(C_Lgt_Btn4);
      L_LgtB5  =       V_Logitech.GetRawButton(C_Lgt_Btn5);
      L_LgtB6  =       V_Logitech.GetRawButton(C_Lgt_Btn6);
      L_LgtB7  =       V_Logitech.GetRawButton(C_Lgt_Btn7);
      L_LgtB8  =       V_Logitech.GetRawButton(C_Lgt_Btn8);
      L_LgtB9  =       V_Logitech.GetRawButton(C_Lgt_Btn9);
      L_LgtB10 =       V_Logitech.GetRawButton(C_Lgt_Btn10);
      L_LgtB11 =       V_Logitech.GetRawButton(C_Lgt_Btn11);
      L_LgtB12 =       V_Logitech.GetRawButton(C_Lgt_Btn12);

      /* Read Xbox controller commands here: */
      L_XB_DrvLY =     -V_XboxDrive.GetRawAxis(C_XB_JoystickLY);
      L_XB_DrvLX =     -V_XboxDrive.GetRawAxis(C_XB_JoystickLX);
      L_XB_DrvRY =     V_XboxDrive.GetRawAxis(C_XB_JoystickRY);
      L_XB_DrvRX =     V_XboxDrive.GetRawAxis(C_XB_JoystickRX);
      L_XB_RTrigger =  V_XboxDrive.GetRawAxis(C_XB_RTrigger);
      L_XbxA =         V_XboxDrive.GetRawButton(C_XB_A_Btn);
      L_XbxB =         V_XboxDrive.GetRawButton(C_XB_B_Btn);
      L_XbxX =         V_XboxDrive.GetRawButton(C_XB_X_Btn);
      L_XbxY =         V_XboxDrive.GetRawButton(C_XB_Y_Btn);
      L_Xbx_LB =       V_XboxDrive.GetRawButton(C_XB_L_Bumper);
      L_Xbx_RB =       V_XboxDrive.GetRawButton(C_XB_R_Bumper);
      L_XbxPOV =       V_XboxDrive.GetPOV(C_XB_POV);

//      L_GyroAngle = ahrs->GetAngle();
      L_GyroAngle = 0;

  /* Determine the drive motor overrides and control: */
      if (L_XbxPOV == 90)
        {
          V_JoystickLock = C_JoystickX_FwdOnly;
        }
      else if (L_XbxPOV == 270)
        {
          V_JoystickLock = C_JoystickX_RvrsOnly;
        }
      else if (L_XbxPOV == 0)
        {
          V_JoystickLock = C_JoystickY_FwdOnly;
        }
      else if (L_XbxPOV == 180)
        {
          V_JoystickLock = C_JoystickY_RvrsOnly;
        }
      else
        {
          V_JoystickLock = C_JoystickDefault;
        }

      switch (V_JoystickLock)
        {
          case C_JoystickX_FwdOnly:
            L_DirectionX = K_MaxRightSpeedPOV;
            L_DirectionY = 0.0;
            L_DirectionZ = 0.0;
            break;

          case C_JoystickX_RvrsOnly:
            L_DirectionX = K_MaxLeftSpeedPOV;
            L_DirectionY = 0.0;
            L_DirectionZ = 0.0;
            break;

          case C_JoystickY_FwdOnly:
            L_DirectionX = 0.0;
            L_DirectionY = K_MaxFwdSpeedPOV;
            L_DirectionZ = 0.0;
            break;

          case C_JoystickY_RvrsOnly:
            L_DirectionX = 0.0;
            L_DirectionY = K_MaxRvrsSpeedPOV;
            L_DirectionZ = 0.0;
            break;

          case C_JoystickDefault:
          default:
            L_DirectionX = L_XB_DrvLX;
            L_DirectionY = L_XB_DrvLY;
            L_DirectionZ = L_XB_DrvRX;
            break;
        }

      /* Determine the winch control */
      if (L_LgtY < 0)
        {
        L_Winch = L_LgtY;
        }
      else if (L_LgtY > 0 &&
               L_LgtT0 == true)
        {
        L_Winch = L_LgtY;
        }
      else
        {
        L_Winch = 0.0;
        }

      /* Determine the indexer control */
      if(L_LgtB5)
        {
          L_GearIndexer = K_IndexerFwdSpd;
        }
      else if (L_LgtB6)
        {
          L_GearIndexer = K_IndexerRvrsSpd;
        }
      else
        {
          L_GearIndexer = 0.0;
        }

      L_DriveGain = fabs(L_XB_RTrigger);

      UpdateActuatorCmnds(L_GyroAngle,
                          false,
                          false,
                          L_DirectionX,
                          L_DirectionY,
                          L_DirectionZ,
                          L_Winch,
                          L_GearIndexer,
                          L_DriveGain);

      L_LED_Mode = UpdateLED_Output(C_Teleop,
                                    L_LgtT0,
                                    L_Winch,
                                    false);

      UpdateSmartDashboad(L_GyroAngle,
                          L_LED_Mode,
                          C_Teleop);

      Wait(C_ControllerUpdateRate);
      }
	}

/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  This is the function that is called during the periodic period
 *               for the test period.
 ******************************************************************************/
  void TestPeriodic()
  {
    double L_GyroAngle;
    bool L_LgtT0;
    LED_Mode L_LED_Mode;

      while (IsOperatorControl() && IsEnabled())
         {
//           L_GyroAngle = ahrs->GetAngle();
           L_GyroAngle = 0;

           L_LgtT0  = V_Logitech.GetRawButton(C_Lgt_Trigger);

           UpdateActuatorCmnds(0,
                               false,
                               false,
                               0,
                               0,
                               0,
                               0,
                               0,
                               0);

           L_LED_Mode = UpdateLED_Output(C_Test,
                                         L_LgtT0,
                                         0.0,
                                         true);

           UpdateSmartDashboad(L_GyroAngle,
                               L_LED_Mode,
                               C_Test);

           Wait(C_ControllerUpdateRate);
         }
  }

/******************************************************************************
 * Function:     UpdateLED_Output
 *
 * Description:  Update the commands sent out to the Arduino controlling the LEDs.
 *               The 8 modes are as follows:
 *
 *                * Chan0 * Chan1 * Chan2 *
 *                    0       0       0      * Mode0 * - Default no comm
 *                    0       0       1      * Mode1 * - Com, but disabled
 *                    0       1       0      * Mode2 * - Blue alliance, main
 *                    0       1       1      * Mode3 * - Blue alliance, final part
 *                    1       0       0      * Mode4 * - Red alliance, main
 *                    1       0       1      * Mode5 * - Red alliance, final part
 *                    1       1       0      * Mode6 * - User light request
 *                    1       1       1      * Mode7 * - End game
 ******************************************************************************/
LED_Mode UpdateLED_Output(RoboState L_RobotState,
                          bool      L_DriverOverride,
                          double    L_Winch,
                          bool      L_CameraLED)
  {
  bool L_Pin0 = false;
  bool L_Pin1 = false;
  bool L_Pin2 = false;
  LED_Mode L_LED_Mode = LED_Mode0;
  double L_MatchTime = DriverStation::GetInstance().GetMatchTime();

  DriverStation::Alliance L_AllianceColor;

  L_AllianceColor = DriverStation::GetInstance().GetAlliance();

  if (L_MatchTime < K_EndMatchWarningTime &&
      L_RobotState == C_Teleop &&
      V_LED_RainbowLatch == false &&
      fabs(L_Winch) > K_WinchOnThreshold)
    {
    /* Accumulate time that the winch has been on while in the end game time.  Once the winch has been on long enough,
     * trigger the final LED effect. */
    V_EndGameWinchTime += C_ControllerUpdateRate;
    if (V_EndGameWinchTime >= K_LED_WinchOnTime)
      {
      V_LED_RainbowLatch = true;
      }
    }

  if (L_DriverOverride == true)
    {
    /* Allow the driver to always override the current LED mode. */
      L_LED_Mode = LED_Mode6;
    }
  else if ((L_RobotState == C_Auton || L_RobotState == C_Teleop) &&
           (L_AllianceColor != DriverStation::Alliance::kInvalid))
    {
      if (V_LED_RainbowLatch == true)
        {
          L_LED_Mode = LED_Mode7;
        }
      else if (L_AllianceColor == DriverStation::Alliance::kBlue)
        {
          if (L_MatchTime > K_EndMatchWarningTime || L_RobotState == C_Auton)
            {
              L_LED_Mode = LED_Mode2;
            }
          else
            {
              L_LED_Mode = LED_Mode3;
            }
        }
      else if (L_AllianceColor == DriverStation::Alliance::kRed)
        {
          if (L_MatchTime > K_EndMatchWarningTime || L_RobotState == C_Auton)
            {
              L_LED_Mode = LED_Mode4;
            }
          else
            {
              L_LED_Mode = LED_Mode5;
            }
        }
    }
  else if (L_RobotState == C_Disabled)
    {
      L_LED_Mode = LED_Mode1;
    }
  else if (L_RobotState == C_Test)
    {
      L_LED_Mode = LED_Mode7;
    }

  switch (L_LED_Mode)
    {
      case LED_Mode1:
        L_Pin0 = false; L_Pin1 = false; L_Pin2 = true;
        break;

      case LED_Mode2:
        L_Pin0 = false; L_Pin1 = true; L_Pin2 = false;
        break;

      case LED_Mode3:
        L_Pin0 = false; L_Pin1 = true; L_Pin2 = true;
        break;

      case LED_Mode4:
        L_Pin0 = true; L_Pin1 = false; L_Pin2 = false;
        break;

      case LED_Mode5:
        L_Pin0 = true; L_Pin1 = false; L_Pin2 = true;
        break;

      case LED_Mode6:
        L_Pin0 = true; L_Pin1 = true; L_Pin2 = false;
        break;

      case LED_Mode7:
        L_Pin0 = true; L_Pin1 = true; L_Pin2 = true;
        break;

      case LED_Mode0:
      default:
        L_Pin0 = false; L_Pin1 = false; L_Pin2 = false;
        break;
  }

// Output to the relay control pins:
  L_CameraLED = false; //TEST!

//  if (L_CameraLED == true)
//    {
////    CameraLED->Set(Relay::Value::kOn);
////    CameraLED->Set(Relay::Value::kForward);
//    }
//  else
//    {
////    CameraLED->Set(Relay::Value::kForward);
////    CameraLED->Set(Relay::Value::kOff);
//    }




  // Output to the DIO pins:
  V_LED_State0.Set(L_Pin0);
  V_LED_State1.Set(L_Pin1);
  V_LED_State2.Set(L_Pin2);

  return (L_LED_Mode);
  }

/******************************************************************************
 * Function:     UpdateActuatorCmnds
 *
 * Description:  Update the commands sent out to the RoboRIO.
 ******************************************************************************/
void UpdateActuatorCmnds(double L_GyroAngle,
                         bool   L_FieldOriented,
                         bool   L_Auton,
                         double L_DriveX,
                         double L_DriveY,
                         double L_DriveRotate,
                         float  L_Winch,
                         float  L_GearIndex,
                         double L_DriveGain)
  {
  double L_GyroInput;

  if (L_FieldOriented == true)
    {
      L_GyroInput = L_GyroAngle;
    }
  else
    {
      L_GyroInput = 0.0;
    }

  if (L_DriveGain < K_DriveGainMin)
    {
      L_DriveGain = K_DriveGainMin;
    }
  else if (L_DriveGain > K_DriveGainMax)
    {
      L_DriveGain = K_DriveGainMax;
    }

#ifdef ROBOT2
  // The practice bot has the drive motors inverted from the final bot
//  L_DriveX = L_DriveX * (-1);
//  L_DriveY = L_DriveY * (-1);
//  L_DriveRotate = L_DriveRotate * (-1);
#endif

  if (L_Auton == false)
    {
      L_DriveX = ApplyDeadband(L_DriveX, K_JoystickDirectionDbX);
      L_DriveX = L_DriveGain * L_DriveX;
      L_DriveX = ApplyLagFilt(L_DriveX, V_DriveX_Prev, K_DriveLagCoeffX);
      V_DriveX_Prev = L_DriveX;

      L_DriveY = ApplyDeadband(L_DriveY, K_JoystickDirectionDbY);
      L_DriveY = L_DriveGain * L_DriveY;
      L_DriveY = ApplyLagFilt(L_DriveY, V_DriveY_Prev, K_DriveLagCoeffY);
      V_DriveY_Prev = L_DriveY;

      L_DriveRotate = ApplyDeadband(L_DriveRotate, K_JoystickDirectionDbZ);
      L_DriveRotate = L_DriveGain * L_DriveRotate;
      L_DriveRotate = ApplyLagFilt(L_DriveRotate, V_DriveZ_Prev, K_DriveLagCoeffZ);
      V_DriveZ_Prev = L_DriveRotate;
    }
  else
    {
      L_DriveX = ApplyLagFilt(L_DriveX, V_DriveX_Prev, K_DriveLagCoeffX);
      V_DriveX_Prev = L_DriveX;

      L_DriveY = ApplyLagFilt(L_DriveY, V_DriveY_Prev, K_DriveLagCoeffY);
      V_DriveY_Prev = L_DriveY;

      L_DriveRotate = ApplyLagFilt(L_DriveRotate, V_DriveZ_Prev, K_DriveLagCoeffZ);
      V_DriveZ_Prev = L_DriveRotate;
    }

  V_GearIndexer.Set(L_GearIndex);

  V_Winch.Set(L_Winch);

  m_robotDrive->MecanumDrive_Cartesian(L_DriveX,
                                       L_DriveY,
                                       L_DriveRotate,
                                       L_GyroInput);

/* my right side motors need to drive negative to move robot forward */
  m_robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,true);
  m_robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor,true);
  }

/******************************************************************************
 * Function:     UpdateSmartDashboad
 *
 * Description:  Report data back to the smart dashboard.
 ******************************************************************************/
void UpdateSmartDashboad(double    L_GyroAngle,
                         LED_Mode  L_LED_Mode,
                         RoboState L_RobotState)
  {
    double L_MatchTime = DriverStation::GetInstance().GetMatchTime();
//    int    L_Location = DriverStation::GetInstance().GetLocation();
    bool   L_MatchEndGameFlag;

    if (L_MatchTime <= K_EndMatchWarningTime &&
        L_RobotState == C_Teleop)
      {
        L_MatchEndGameFlag = true;
      }
    else
      {
        L_MatchEndGameFlag = false;
      }

    /* Update the sensors on the robot: */
    SmartDashboard::PutNumber("XValue", V_Accel.GetX());
    SmartDashboard::PutNumber("YValue", V_Accel.GetY());
    SmartDashboard::PutNumber("ZValue", V_Accel.GetZ());
    SmartDashboard::PutNumber("GyroAngle", L_GyroAngle);

    /* Update the state variables: */
    SmartDashboard::PutBoolean("30 Second Warning", L_MatchEndGameFlag);
    SmartDashboard::PutNumber("Match Time",L_MatchTime);
    SmartDashboard::PutNumber("LED Mode", double(L_LED_Mode));

//    SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
#ifdef GYRO2
    SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
#endif

    SmartDashboard::PutNumber( "Distance1",       V_Distance1);
    SmartDashboard::PutNumber( "Distance2",       V_Distance2);
    SmartDashboard::PutNumber( "Center",          V_Center);
    SmartDashboard::PutNumber( "Blobs",           V_BlobsDetected);

    SmartDashboard::PutNumber( "XB LY", V_XboxDrive.GetRawAxis(C_XB_JoystickLY));
    SmartDashboard::PutNumber( "XB LX", V_XboxDrive.GetRawAxis(C_XB_JoystickLX));
    SmartDashboard::PutNumber( "XB RY", V_XboxDrive.GetRawAxis(C_XB_JoystickRY));
    SmartDashboard::PutNumber( "XB RX", V_XboxDrive.GetRawAxis(C_XB_JoystickRX));

    SmartDashboard::PutNumber( "XB Gain", V_XboxDrive.GetRawAxis(C_XB_RTrigger));




    SmartDashboard::PutNumber( "Auton Mode2",      (double)V_AutonPosition);
    SmartDashboard::PutNumber( "Auton Mode3",      (double)V_AutonMode3);

  }
};
START_ROBOT_CLASS(MecanumDefaultCode);
