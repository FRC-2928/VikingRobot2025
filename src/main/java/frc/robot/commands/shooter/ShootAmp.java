package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class ShootAmp extends Command {
	public ShootAmp(final boolean correction) {
		this.correction = correction;

		this.addRequirements(Robot.cont.shooter);
		if(correction) this.addRequirements(Robot.cont.drivetrain);
	}

	private final boolean correction;

	private boolean fired;

	@Override
	public void initialize() { this.fired = false; }

	@Override
	public void execute() {
		Robot.cont.shooter.io.rotate(Constants.Shooter.shootAmp);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.of(15));

		Robot.cont.drivetrain
			.control(Robot.cont.drivetrain.joystickSpeeds.plus(Robot.cont.drivetrain.rod(new ChassisSpeeds(0, 0, 0))));

		if(
			(Math
				.abs(
					Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Constants.Shooter.shootAmp.in(Units.Degrees)
				) <= 20.5
				&& Robot.cont.shooter.inputs.flywheelSpeed.in(Units.RotationsPerSecond) >= 10
				&& Robot.cont.driverOI.intakeShoot.getAsBoolean()) || this.fired
		) {
			Robot.cont.shooter.io.runFeeder(Demand.Forward);
			this.fired = true;
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote
					? Constants.Shooter.startingConfiguration
					: Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.zero());
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}

// TODO: implement aiming
// fwd/back based on pose
