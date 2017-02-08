#include "WPILib.h"
#include "CANTalon.h"

typedef enum {
	C_JoystickX_Only,
	C_JoystickY_Only,
	C_JoystickZ_Only,
	C_JoystickDefault
}JoystickLock;

JoystickLock V_JoystickLock;

class MecanumDefaultCode : public IterativeRobot
{
	CANTalon lf; /*left front */
	CANTalon lr;/*left rear */
	CANTalon rf; /*right front */
	CANTalon rr; /*right rear */
public:
	RobotDrive *m_robotDrive;		// RobotDrive object using PWM 1-4 for drive motors
	Joystick V_Logitech;			// Joystick object on USB port 1 (mecanum drive)public:
      BuiltInAccelerometer V_Accel;
    ADXRS450_Gyro V_GyroOne;
	/**
	 * Constructor for this "MecanumDefaultCode" Class.
	 */
	MecanumDefaultCode(void) : lf(1), lr(2), rf(3), rr(4), V_GyroOne(SPI::Port::kOnboardCS0), V_Logitech(0)
	{
		/* Set every Talon to reset the motor safety timeout. */
		lf.Set(0);
		lr.Set(0);
		rf.Set(0);
		rr.Set(0);
		// Create a RobotDrive object using PWMS 1, 2, 3, and 4
		m_robotDrive = new RobotDrive(lf, lr, rf, rr);
		m_robotDrive->SetExpiration(0.5);
		m_robotDrive->SetSafetyEnabled(false);
		// Define joystick being used at USB port #0 on the Drivers Station
	}
	void TeleopInit()
	{
		V_JoystickLock = C_JoystickDefault;
		V_GyroOne.Calibrate();
	}
	/** @return 10% deadband */
	double Db(double axisVal)
	{
		if(axisVal < -0.25)
			return (axisVal*0.5);
		if(axisVal > +0.25)
			return (axisVal*0.5);
		return 0;
	}
	/**
	 * Gets called once for each new packet from the DS.
	 */
	void TeleopPeriodic(void)
	{
		int L_LgtP1;
		double L_LgtX;
		double L_LgtY;
		double L_LgtZ;
        bool L_LgtT0;
        bool L_LgtB1,L_LgtB3,L_LgtB4,L_LgtB5,L_LgtB6,L_LgtB7,L_LgtB8,L_LgtB9,L_LgtB11,L_LgtB10,L_LgtB12;
        double L_DirectionX = 0;
		double L_DirectionY = 0;
		double L_DirectionZ = 0;
		double angle = V_GyroOne.GetAngle();



		L_LgtX = V_Logitech.GetRawAxis(0);
		L_LgtY = V_Logitech.GetRawAxis(1);
		L_LgtZ = V_Logitech.GetRawAxis(2);
		L_LgtP1  = V_Logitech.GetPOV(0);
		L_LgtT0  = V_Logitech.GetRawButton(1);
		L_LgtB1  = V_Logitech.GetRawButton(2);
		L_LgtB3  = V_Logitech.GetRawButton(3);
		L_LgtB4  = V_Logitech.GetRawButton(4);
		L_LgtB5  = V_Logitech.GetRawButton(5);
		L_LgtB6  = V_Logitech.GetRawButton(6);
		L_LgtB7  = V_Logitech.GetRawButton(7);
		L_LgtB8  = V_Logitech.GetRawButton(8);
		L_LgtB9  = V_Logitech.GetRawButton(9);
		L_LgtB10 = V_Logitech.GetRawButton(10);
		L_LgtB11 = V_Logitech.GetRawButton(11);
		L_LgtB12 = V_Logitech.GetRawButton(12);

		if (L_LgtB5 == true)
		{
			V_JoystickLock = C_JoystickX_Only;
		}
		else if (L_LgtB6 == true)
		{
			V_JoystickLock = C_JoystickY_Only;
		}
		else if (L_LgtB4 == true)
		{
			V_JoystickLock = C_JoystickZ_Only;
		}
		else if (L_LgtB1 == true)
		{
			V_JoystickLock = C_JoystickDefault;
		}

		if (V_JoystickLock == C_JoystickDefault)
		{
		    L_DirectionX = L_LgtX;
		    L_DirectionY = L_LgtY;
		    L_DirectionZ = L_LgtZ;
		}
		else if (V_JoystickLock ==C_JoystickX_Only)
		{
			L_DirectionX = L_LgtX;
		    L_DirectionY = 0;
		    L_DirectionZ = 0;
		}
		else if (V_JoystickLock ==C_JoystickY_Only)
		{
		    L_DirectionX = 0;
			L_DirectionY = L_LgtY;
			L_DirectionZ = 0;
		}
		else if (V_JoystickLock ==C_JoystickZ_Only)
				{
				    L_DirectionX = 0;
					L_DirectionY = 0;
					L_DirectionZ = L_LgtZ;
				}


		//std::cout << "Angle : " << angle << std::endl;
		m_robotDrive->MecanumDrive_Cartesian(	Db(L_DirectionX),
												Db(L_DirectionY),
												Db(L_DirectionZ),
												angle);
		/* my right side motors need to drive negative to move robot forward */
		m_robotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor,true);
		m_robotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor,true);
		/* on button 5, reset gyro angle to zero */
		if(L_LgtT0)
			V_GyroOne.Calibrate();

		  SmartDashboard::PutNumber("XValue", V_Accel.GetX());
		    SmartDashboard::PutNumber("YValue", V_Accel.GetY());
		    SmartDashboard::PutNumber("ZValue", V_Accel.GetZ());
		    SmartDashboard::PutNumber("GyroAngle",V_GyroOne.GetAngle());
		    SmartDashboard::PutNumber("Lockout: 0-X 1-Y 2-Z 3-Default", double(V_JoystickLock));
	}
};
START_ROBOT_CLASS(MecanumDefaultCode);
