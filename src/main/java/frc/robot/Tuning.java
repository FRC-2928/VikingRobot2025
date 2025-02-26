package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/*
 * This class instantiates LoggedNetworkNumbers for any tuning parameters we want to tune/change during runtime.
 * The values can be used in a class by calling number.get() in a subsystem periodic() or command execute()
 * and can be accessed/modified via Shuffleboard
 */
public final class Tuning {
	private Tuning() { throw new IllegalCallerException("Cannot instantiate `Tuning`"); }

	public static final LoggedNetworkNumber intakeSpeed = new LoggedNetworkNumber
		("Tuning/SpeedIntakePercent",
		.8);
}
