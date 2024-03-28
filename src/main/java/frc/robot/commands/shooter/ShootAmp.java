package frc.robot.commands.shooter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;
import frc.robot.utils.STalonFX;

public class ShootAmp extends Command {
	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);

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
		Robot.cont.shooter.io.rotate(Units.Degrees.of(Tuning.ampAngle.get()));
		Robot.cont.shooter.io.runFlywheels(Tuning.ampPower.get());
		if(this.correction) {
			Robot.cont.drivetrain
				.control(
					Robot.cont.drivetrain.joystickSpeeds
						.plus(Robot.cont.drivetrain.rod(new ChassisSpeeds(0, 0, 0)))
						.div(2)
				);
		}

		if(
			(Math.abs(Robot.cont.shooter.inputs.angle.in(Units.Degrees) - Tuning.ampAngle.get()) <= 20.5
				&& Robot.cont.driverOI.intakeShoot.getAsBoolean()) || this.fired
		) {
			if(Math.abs(this.pivot.getVelocity().getValue()) < Constants.Shooter.pivotMaxVelocityShoot) {
				Robot.cont.shooter.io.runFeeder(Demand.Forward);
				this.fired = true;
			}
		}

		// todo: auto align
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote
					? Constants.Shooter.startingConfiguration
					: Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}
