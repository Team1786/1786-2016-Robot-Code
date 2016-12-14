/*
 * Shooter.h
 *
 *  Created on: Feb 2, 2016
 *      Author: Jason
 */

#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_

#define PICKUP_POWER 0.75
#define LAUNCH_POWER 1
#define SPINUP_TIME 1.5 // seconds.
#define LAUNCH_TIME 0.5
#define RAMP_POWER 0.3

class Shooter 
{
public:

	/**
	 * Shooter talons and launch-spinny talon.
	 * s2 is also for the pickup-mechanism and can be controlled independently.
	 *
	 */
	enum ShooterState
	{
		READY,
		ON_FIRE,
		SPINNINGUP,
		LAUNCH,
		LAUNCHING,
		RESETTING
	};

	Shooter(CANTalon *s1, CANTalon *s2, CANTalon *r)
	{
	//	shooterDrive = new RobotDrive(s1, s2);
		launcher = s1;
		pickup = s2;
		ramp = r;
		ready = true;
		state = READY;
	}

	/**
	 * Call this method on TeleopInit so that the ramp is properly
	 * set at the beginning of the match.
	 */

	virtual ~Shooter() 
	{
		delete launcher;
		delete pickup;
		delete ramp;
	}
	
	void StopShooter()
	{
		ready = true;
		ramp->Set(0);
		launcher->Set(0);
		pickup->Set(0);
	}

	void LowerRamp()
	{
		ramp->Set(-RAMP_POWER);
		if(ramp->Limits::kReverseLimit){
			SmartDashboard::PutNumber("ramp", 2); //going to put a circlar dial to show where the ramp could be
		} else {
			SmartDashboard::PutNumber("ramp", 1);
		}
	}

	void RaiseRamp()
	{
		ramp->Set(RAMP_POWER);
		if(ramp->Limits::kForwardLimit){
			SmartDashboard::PutNumber("ramp", 0); //going to put a circlar dial to show where the ramp could be
		} else {
			SmartDashboard::PutNumber("ramp", 1);
		}
	}

	void StopRamp()
	{
		ramp->Set(0);
	}
	void Shoot()
	{
		if (shotClock.Get() > (SPINUP_TIME + 0.1))
		{
			state = RESETTING;
		}
		switch (state)
		{
			case READY:
			{
				state = SPINNINGUP;
				ramp->Set(-1);
				launcher->Set(LAUNCH_POWER);
				pickup->Set(LAUNCH_POWER);
				shotClock.Reset();
				shotClock.Start();
				break;
			}
			case SPINNINGUP:
			{
				if (shotClock.Get() > SPINUP_TIME)
				{
					state = LAUNCH;
					shotClock.Reset();
					shotClock.Start();
				} else
				{
					//std::cout << "*Goku noises*\n";
				}
				break;
			}
			case LAUNCH:
			{
				ramp->Set(LAUNCH_POWER);
				state = LAUNCHING;
				break;
			}
			case LAUNCHING:
			{
				if (shotClock.Get() > LAUNCH_TIME)
				{

					state = RESETTING;
				}
				break;
			}
			case RESETTING:
			{
				ramp->Set(0);
				launcher->Set(0);
				pickup->Set(0);
				state = READY;
				break;
			}
			case ON_FIRE:
			{
				std::cout << "Something is wrong with the launch sequence.\n";
				break;
			}
		}
		
	}
	
	void PickUp(bool state = true) 
	{
		pickup->Set((float) (state * PICKUP_POWER));
		launcher->Set((float) (state * PICKUP_POWER * -1));
		//std::cout << "picking up!\n";
	}

	/**
	 * Call this to run the pickup backwards if the ball gets jammed somehow...
	 */
	void Unjam()
	{
		pickup->Set(-1 * PICKUP_POWER);
	}

	void ShootLow()
	{
		pickup->Set(-1);
	}

	void SetPower(float power) 
	{
		pickup->Set(power);
		launcher->Set(power);
		//std::cout << "setting shooter power" << std::endl;
	}

	int GetState()
	{
		return state;
	}
private:

	//RobotDrive *shooterDrive;
	CANTalon *launcher;
	CANTalon *pickup;
	CANTalon *ramp;
	ShooterState state;
	
	Timer shotClock;  
	bool ready;
	int fake_position;
};

#endif /* SRC_SHOOTER_H_ */
