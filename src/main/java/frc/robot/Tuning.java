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
	public static final LoggedNetworkNumber lFourHeightMeter = new LoggedNetworkNumber(
		"/Tuning/LFourHeight",
		1
	);
	public static final LoggedNetworkNumber coralHeight = new LoggedNetworkNumber(
		"/Tuning/CoralHeight",
		1
	);
	public static final LoggedNetworkNumber coralPivot = new LoggedNetworkNumber(
		"/Tuning/coralPivot",
		0
	);
	public static final LoggedNetworkNumber algielHeight = new LoggedNetworkNumber(
		"/Tuning/algielHeight",
		0.5
	);
	public static final LoggedNetworkNumber algiePivot = new LoggedNetworkNumber(
		"/Tuning/algiePivot",
		0
	);
	public static final LoggedNetworkNumber climbHeight = new LoggedNetworkNumber(
		"/Tuning/climbHeight",
		2
	);
	public static final LoggedNetworkNumber climbPivot = new LoggedNetworkNumber(
		"/Tuning/climbPivot",
		0
	);
}
