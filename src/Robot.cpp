#include "WPILib.h"
#include "Shooter.h"
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <sys/time.h>
#include <vector>
#include <cmath>
#include <math.h>

#ifndef BUTTON_LAYOUT
#define BUTTON_LAYOUT

#define TRIGGER 	1  	// Trigger button number
#define THUMB 		2	// Thumb button number
#define RAMP_RAISE  5   // Button 3 for Raising Ramp
#define RAMP_LOWER  3	// Button 4 to lower ramp.
#define UNJAM		11
#define GIM_UP		7
#define GIM_DOWN	9
#define GIM_ZERO	10
#define GIM_SHOOT	8

#define GIMB_DEFAULT 768 //Default position of the gimble (*1000)
#define GIMB_DELTA 2 //Amount to increment by

#define ARMS_SCALE  0.75

#define DEADZONE_RADIUS 0.05 	// Deadzone Radius prevents tiny twitches in the joystick's value from
								// affecting the robot.  Use this for cleaning up drive train and shooter.
								// Also used for detecting changes in an axis' value.

#define TURN_FACTOR 1.5			// Left(x,y)  = y*(1 + TF*x)  :  x < 0
								// 			  = y 			  :  x >= 0
								// Right(x,y) = y			  :  x < 0
								// 			  = y*(1 - TF*x)  :  x >= 0

#endif  // BUTTON_LAYOUT

#ifndef LOG
#define LOG(X) logA << X; logB << X; logC << X //for writing data to the csv file
#endif

class Robot: public IterativeRobot
{
private:
	TalonSRX left_drive, right_drive;
	CANTalon shooter1, shooter2,
			ramp,
			arms;
	RobotDrive drive;
	Shooter shooter;
	Joystick driver_stick, operator_stick;
	Timer auto_clock;
	Servo gimbs;
	int gimba; // angle of the gimble as a value of from 0 to 1000

	// instance variables
	bool pickupRunning;  // don't want to spam the Talon with set messages.  Toggle the pickup when a button is pressed or released.
	bool inverting;
	bool ramping;
	bool shooting;
	bool unjamming;
	bool arming;
	bool arcade;
	float shooter_power;

	enum auto_states
	{
		START,
		DRIVING_FORWARD,
		STOP
	};

	auto_states auto_status;

	LiveWindow *lw = LiveWindow::GetInstance();
	SendableChooser *chooser;
	const std::string autoNameDefault = "1.7 Seconds (Slightly Longer)";
	const std::string autoNameShort = "1.5 Seconds (Short, good on the moat)";
	const std::string autoNameOne = "1 Seconds";
	const std::string autoNameDisable = "Disable Autonomous (set time to 0)";
	std::string autoSelected;

	void LogCSVData()
	{
		static PowerDistributionPanel pdp;	// preparing to read from the pdp
		static std::vector<CANTalon*> motors;

		static std::ofstream logA, logB, logC;
		timeval tm;

		SmartDashboard::PutBoolean("log A", logA.is_open());
		SmartDashboard::PutBoolean("log B", logB.is_open());
		SmartDashboard::PutBoolean("log C", logC.is_open());

		SmartDashboard::PutBoolean("Ramp Limit F", !ramp.IsFwdLimitSwitchClosed());
		SmartDashboard::PutBoolean("Ramp Limit R", !ramp.IsRevLimitSwitchClosed());
		SmartDashboard::PutBoolean("Arms Limit F", !arms.IsFwdLimitSwitchClosed());
		SmartDashboard::PutBoolean("Arms Limit R", !arms.IsRevLimitSwitchClosed());

		SmartDashboard::PutNumber("angle of camera", gimba);

		if (!logA.is_open() && !logB.is_open() && !logC.is_open())
		{
			std::fstream logNumFile;
			int logNum;
			logNumFile.open("/home/lvuser/logNum");
			logNumFile >> logNum;
			logNum++;
			logNumFile.seekp(0);
			logNumFile << logNum;
			// writing to /home/lvuser/logs/[unixtime].log
			logA.open("/media/sda1/logs/log" + std::to_string(logNum) + ".csv");
			std::cerr << (logA.is_open() ? "Opened" : "Failed to open") << "log A." << std::endl;
			logB.open("/media/sdb1/logs/log" + std::to_string(logNum) + ".csv");
			std::cerr << (logB.is_open() ? "Opened" : "Failed to open") << "log B." << std::endl;
			logC.open("/home/lvuser/logs/log" + std::to_string(logNum) + ".csv");
			std::cerr << (logC.is_open() ? "Opened" : "Failed to open") << "log C." << std::endl;

			LOG("Time\tpdpInput voltage\tpdpTemperature\tpdpTotal Current\t");
			for (int ii = 0; ii < 16; ii++)
			{
				LOG("pdpChannel " << ii << " current\t");
			}

			LOG("tlaunch_spinny Bus Voltage\ttlaunch_spinny Output Current\ttlaunch_spinny Output Voltage\ttlaunch_spinny Temperature");
			motors.push_back(&ramp);
			LOG("\tShooter1 Bus Voltage\tShooter1 Output Current\tShooter1 Output Voltage\tShooter1 Temperature");
			motors.push_back(&shooter1);
			LOG("\tShooter2 Bus Voltage\tShooter2 Output Current\tShooter2 Output Voltage\tShooter2 Temperature");
			motors.push_back(&shooter2);

			LOG("\tJoystick X\tJoystick Y\tJoystick Twist");
			LOG("\tAlliance\tLocation\tMatch Time\tFMS Attached\tBrowned Out");
			LOG("\tTestStage");
			LOG(std::endl);
		}
		gettimeofday(&tm, NULL);
		LOG(time(0) << '.' << std::setfill('0') << std::setw(3) << tm.tv_usec/1000);
		// Some general information
		LOG("\t" << pdp.GetVoltage());
		LOG("\t" << pdp.GetTemperature());
		LOG("\t" << pdp.GetTotalCurrent());
		// current on each channel
		for (int ii = 0; ii < 16; ii++)
		{
			LOG("\t" << pdp.GetCurrent(ii));
		}

		//Talon Data
		for(int ii = 0; ii < motors.size(); ii++)
		{
			LOG("\t" << motors[ii]->GetBusVoltage());
			LOG("\t" << motors[ii]->GetOutputVoltage());
			LOG("\t" << motors[ii]->GetOutputCurrent());
			LOG("\t" << motors[ii]->GetTemperature());
		}
		//control data
		LOG("\t" << driver_stick.GetX());
		LOG("\t" << driver_stick.GetY());
		LOG("\t" << driver_stick.GetTwist());

		//DriverStation Data
		LOG("\t" << DriverStation::GetInstance().GetAlliance());
		LOG("\t" << DriverStation::GetInstance().GetLocation());
		LOG("\t" << DriverStation::GetInstance().GetMatchTime());
		LOG("\t" << DriverStation::GetInstance().IsFMSAttached());
		LOG("\t" << DriverStation::GetInstance().IsSysBrownedOut());
		LOG(std::endl);
	}

	/**
	 *	Takes the gross raw throttle input from joystick and returns a
	 *	value between 0.0-1.0 (no negative values)
	 */
	float SaneThrottle(float rawThrottle)
	{
		return ((1.0 - rawThrottle) / 2.0);
	}

	void UpdateDrive()
	{
		float x = -driver_stick.GetX();
		float y = -driver_stick.GetY();
		if (x > 0)
		{
			float right = y * SaneThrottle(driver_stick.GetThrottle());
			float left = (1-x)*y * SaneThrottle(driver_stick.GetThrottle());
			drive.TankDrive(left, right);
		}
		else
		{
			float left = y * SaneThrottle(driver_stick.GetThrottle());
			float right = (1+x)*y * SaneThrottle(driver_stick.GetThrottle());
			drive.TankDrive(left, right);
		}
	}

public:
	Robot():
		left_drive(0),			// Left DriveTrain Talons plug into PWM channel 1 with a Y-splitter
		right_drive(1),			// Right DriveTrain Talons plug  	// left wheel 2
		shooter1(11),  			// shooter drive 1
		shooter2(10),   		// shooter drive 2
		ramp(12),
		arms(13),
		drive(&left_drive, &right_drive),
		shooter(				// initialize Shooter object.
				&shooter1, &shooter2, &ramp),
		driver_stick(0), 				// right stick (operator)
		operator_stick(1),			// left stick (driver)
		gimbs(3),
		gimba(GIMB_DEFAULT)
	{

	}

	void RobotInit()
	{
		chooser = new SendableChooser();

		chooser->AddDefault(autoNameDefault, '(void*)&autoNameDefault');
		chooser->AddObject(autoNameShort, (void*)&autoNameShort);
		chooser->AddObject(autoNameOne, (void*)&autoNameOne);
		chooser->AddObject(autoNameDisable, (void*)&autoNameDisable);
		SmartDashboard::PutData("Auto Modes", chooser);
		shooter1.Enable();
		shooter2.Enable();
		left_drive.SetInverted(true);
		right_drive.SetInverted(true);
		//ramp.SetInverted(true);
		inverting = false;
		pickupRunning = false;
		ramping = false;
		shooting = false;
		unjamming = false;
		arming = false;
		shooter_power = 0;
		arcade = false;

		// Initialize the number so we can get it later
		SmartDashboard::PutNumber("shooting angle", GIMB_DEFAULT);

	}

	void AutonomousInit()
	{
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		auto_status = START;
	}

	void AutonomousPeriodic()
	{
		const float drivePower = 1;
		float driveTime = 1.7;
		LogCSVData();
		if (autoSelected == autoNameDisable) driveTime = 0;
		else if (autoSelected == autoNameShort) driveTime = 1.5;
		else if (autoSelected == autoNameOne) driveTime = 1;
		else driveTime = 1.7;

		//Default Auto goes here
		switch (auto_status)
		{
		case START:
		{
			auto_clock.Start();
			auto_status = DRIVING_FORWARD;
			break;
		}
		case DRIVING_FORWARD:
		{
			if (auto_clock.Get() > driveTime)
			{
				drive.TankDrive(0.0, 0.0);
				auto_status = STOP;
			}
			else
			{
				drive.TankDrive(drivePower, drivePower);
			}
			break;
		}
		case STOP:
		{
			std::cout << "All done!\n" ;
			break;
		}
		}
		gimbs.Set(gimba/1000.0);
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		LogCSVData();
		std::cout << "arm encoder position: " << arms.GetEncPosition() << std::endl;
		std::cout << "arm encoder velocity: " << arms.GetEncVel() << std::endl;

		if(driver_stick.GetRawButton(7))
		{
			arcade = true;
		}
		if(driver_stick.GetRawButton(8))
		{
			arcade = false;
		}
		if (arcade)
		{
			drive.ArcadeDrive(driver_stick);
		}
		else
		{
			if (driver_stick.GetRawButton(THUMB))
			{
				float left = driver_stick.GetTwist();
				float right = -driver_stick.GetTwist();
				drive.TankDrive(left, right);
			}
			else
			{
				UpdateDrive();
			}
		}
		//bool rampDoing = false;
		//  This is shit code for testing.  Replace it with real code.
		if(!ramping && operator_stick.GetRawButton(RAMP_RAISE))
		{
			std::cout << "Raising Ramp.";
			//ramp.Set(1);
			shooter.RaiseRamp();
			ramping =true;
		}
		else if(!ramping && operator_stick.GetRawButton(RAMP_LOWER))
		{
			std::cout << "Lowering Ramp.";
			//ramp.Set(-1);
			shooter.LowerRamp();
			ramping = true;
		}
		/*else if(!ramping && operator_stick.GetRawButton(TRIGGER))
		{
			shooter.BoostRamp();
			ramping = true;
		}*/
		else if(ramping && !operator_stick.GetRawButton(RAMP_RAISE)
				&& !operator_stick.GetRawButton(RAMP_LOWER))
		{
			shooter.StopRamp();
			ramping = false;
		}


		if(!unjamming && operator_stick.GetRawButton(UNJAM))
		{
			unjamming = true;
			shooter.Unjam();
		}
		else if(!unjamming && operator_stick.GetRawButton(TRIGGER))
		{
			shooter.ShootLow();
			unjamming = true;
		}
		else if(unjamming && !operator_stick.GetRawButton(UNJAM) && !operator_stick.GetRawButton(TRIGGER))
		{
			shooter.PickUp(false);
			unjamming = false;
		}

		/*
		 * This is for controlling the gimbal.
		 */
		if(operator_stick.GetRawButton(GIM_UP))
		{
	  		gimba += GIMB_DELTA;
		}
		else if(operator_stick.GetRawButton(GIM_DOWN)){
			gimba -= GIMB_DELTA;
		}
		else if(operator_stick.GetRawButton(GIM_SHOOT))
		{
			gimba = SmartDashboard::GetNumber("shooting angle", GIMB_DEFAULT);
		}
		gimbs.Set(gimba/1000.0);

		/*
		 * 	Run the Shooter only while the THUMB button is held down on the operator stick.
		 *	the 'pickupRunning' boolean is there to prevent the shooter from calling PickUp
		 *	every iteration of the TeleopPeriodic method (once every 10ms!)
		 *	The pickup is disabled when the thumb button is released, but the code still
		 *  has 'pickupRunning' as true.
		 */
		if(operator_stick.GetRawButton(THUMB) && !pickupRunning)
		{
			shooter.PickUp();
			pickupRunning = true;
		}
		else if(!operator_stick.GetRawButton(THUMB) && pickupRunning)
		{
			shooter.PickUp(false);
			pickupRunning = false;
		}
		/*
		 *	The 'inverting' variable is used to make sure that the drive train isn't getting
		 *	inverted every iteration of the TeleopPeriodic method while the button is held down.
		 *	This is important because the TeleopPeriodic method executes something like once every 10ms.
		 *	Thus, this if-else if pair make the button a toggle.
		 */
		if(driver_stick.GetRawButton(TRIGGER) && !inverting)
		{
			std::cout << "Inverting Drive Train.";
			left_drive.SetInverted(!left_drive.GetInverted());
			right_drive.SetInverted(!right_drive.GetInverted());
			inverting = true;
		}
		else if(!driver_stick.GetRawButton(TRIGGER))
		{
			inverting = false;
		}

		if(operator_stick.GetRawButton(4))
		{
			arms.Set(-operator_stick.GetY() * ARMS_SCALE);
		}
		else
		{
			arms.Set(0);
		}

        // This code will become obsolete after the Shooter logic is complete.
		float opThrottle = SaneThrottle(operator_stick.GetThrottle());

		if(!pickupRunning && ( opThrottle > shooter_power + DEADZONE_RADIUS
                               ||      opThrottle < shooter_power - DEADZONE_RADIUS))
		{
			shooter.SetPower(opThrottle);
		}
	}

	void TestPeriodic()
	{
		const int driveTime = 5; //Drive for 5 seconds
		const float drivePower = 0.5, armPower=.35, rampPower=.4; //Arbitrary powers to test at
		static Timer t;
		enum test_stages {
			INIT,
			ARMS_UP,
			ARMS_DOWN,
			RAMP_UP,
			RAMP_DOWN,
			DRIVE_FORWARD,
			DRIVE_BACKWARD,
			TURN_CW,
			TURN_CCW,
			END};

		static enum test_stages test_stage, old_test_stage;

		if (old_test_stage != test_stage)
		{
			//Reset timer between stages
			t.Stop();
			t.Reset();

			//Wait for a button press between stages
			if (operator_stick.GetRawButton(TRIGGER))
				old_test_stage = test_stage;
		}
		else
		{
			if(!t.Get()) t.Start();

			switch (test_stage)
			{
			case INIT:
			{
				break;
			}

			case ARMS_UP:
			{
				if (arms.GetForwardLimitOK())
					arms.Set(1);
				else
				{
					arms.Set(0);
					test_stage = ARMS_DOWN;
				}
				break;
			}

			case ARMS_DOWN:
			{
				if (arms.GetReverseLimitOK())
					arms.Set(-1);
				else
				{
					arms.Set(0);
					test_stage = RAMP_UP;
				}
				break;
			}

			case RAMP_UP:
			{
				if (arms.GetForwardLimitOK())
					ramp.Set(1);
				else
				{
					ramp.Set(0);
					test_stage = RAMP_DOWN;
				}
				break;
			}

			case RAMP_DOWN:
			{
				if (arms.GetReverseLimitOK())
					ramp.Set(-1);
				else
				{
					ramp.Set(0);
					test_stage = DRIVE_FORWARD;
				}
				break;
			}

			case DRIVE_FORWARD:
			{
				if (t.Get() < driveTime)
					drive.TankDrive(drivePower, drivePower);
				else
				{
					drive.TankDrive(0.0, 0.0);
					test_stage = DRIVE_BACKWARD;
				}
				break;
			}

			case DRIVE_BACKWARD:
			{
				if (t.Get() < driveTime)
					drive.TankDrive(-drivePower, -drivePower);
				else
				{
					drive.TankDrive(0.0, 0.0);
					test_stage = TURN_CW;
				}
				break;
			}

			case TURN_CW:
			{
				if (t.Get() < driveTime)
					drive.TankDrive(drivePower, -drivePower);
				else
				{
					drive.TankDrive(0.0, 0.0);
					test_stage = TURN_CCW;
				}
				break;
			}

			case TURN_CCW:
			{
				if (t.Get() < driveTime)
					drive.TankDrive(-drivePower, drivePower);
				else
				{
					drive.TankDrive(0.0, 0.0);
					test_stage = END;
				}
				break;
			}

			case END:
			{
				break;
			}
			}
		}

		LogCSVData();
	}
};

START_ROBOT_CLASS(Robot)
