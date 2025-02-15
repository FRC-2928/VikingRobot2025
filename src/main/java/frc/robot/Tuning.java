package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public final class Tuning {
	private Tuning() { throw new IllegalCallerException("Cannot instantiate `Tuning`"); }

	public static final LoggedDashboardNumber drivetrainP = new LoggedDashboardNumber(
		"Tuning/Drivetrain P",
		0.15
	);
	public static final LoggedDashboardNumber shootSpeakerPivotThreshold = new LoggedDashboardNumber(
		"Tuning/ShootSpeakerPivotThreshold",
		1.25
	);
	public static final LoggedDashboardNumber shootSpeakerExponent = new LoggedDashboardNumber(
		"Tuning/ShootSpeakerExponent",
		1
	);
}
