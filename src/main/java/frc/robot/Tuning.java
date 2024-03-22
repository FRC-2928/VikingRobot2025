package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.units.Units;

public final class Tuning {
	private Tuning() { throw new IllegalCallerException("Cannot instantiate `Tuning`"); }

	public static final LoggedDashboardNumber flywheelVelocity = new LoggedDashboardNumber(
		"Tuning/FlywheelSpeed",
		Constants.Shooter.flywheels.speakerVelocity.in(Units.RotationsPerSecond)
	);
	public static final LoggedDashboardNumber flywheelVelocityThreshold = new LoggedDashboardNumber(
		"Tuning/FlywheelSpeedThreshold",
		Constants.Shooter.flywheels.speakerVelocityThreshold.in(Units.RotationsPerSecond)
	);

	public static final LoggedDashboardNumber ampAngle = new LoggedDashboardNumber(
		"Tuning/AmpAngle",
		Constants.Shooter.shootAmp.in(Units.Degrees)
	);
	public static final LoggedDashboardNumber ampPower = new LoggedDashboardNumber(
		"Tuning/AmpPower",
		Constants.Shooter.flywheels.ampPower
	);
}
