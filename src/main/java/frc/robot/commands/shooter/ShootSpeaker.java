package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIO.Demand;

public class ShootSpeaker extends Command {
	public ShootSpeaker() { this.addRequirements(Robot.cont.shooter); }

	protected boolean fired;
	private final SimpleMotorFeedforward rffw = new SimpleMotorFeedforward(0, 10);

	@Override
	public void initialize() { this.fired = false; }

	@Override
	public void execute() {
		final boolean forward = Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getCos() < 0;
		final boolean current = Robot.cont.shooter.inputs.angle.in(Units.Degrees) - 90 < 0;

		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.of(90));

		Robot.cont.drivetrain.limelightShooter.setPipeline(forward ? 0 : 1);
		if(forward == current) {
			if(Robot.cont.drivetrain.limelightShooter.hasValidTargets()) {
				final Measure<Angle> po = Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset();
				final Measure<Angle> yo = Robot.cont.drivetrain.limelightShooter.getTargetVerticalOffset();

				Robot.cont.drivetrain
					.control(
						Robot.cont.drivetrain.joystickSpeeds
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											this.rffw.calculate(yo.in(Units.Rotations) * (forward ? 1 : -1))
										)
									)
							)
					);

				if(Math.abs(po.in(Units.Degrees)) >= 2.5 && !this.fired) {
					Robot.cont.shooter.io.rotate(Robot.cont.shooter.inputs.angle.minus(po));
				} else {
					if(
						(Robot.cont.shooter.inputs.flywheelSpeed
							.in(Units.RotationsPerSecond) >= Constants.Shooter.flywheelSpeedThreshold
								.in(Units.RotationsPerSecond)
							&& Robot.cont.driverOI.intakeShoot.getAsBoolean()) || this.fired
					) {
						Robot.cont.shooter.io.runFeeder(Demand.Forward);
						this.fired = true;
					}
				}
			} else Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
		} else {
			Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
			Robot.cont.shooter.io
				.rotate(forward ? Constants.Shooter.readyShootFront : Constants.Shooter.readyShootRear);
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(Units.RotationsPerSecond.zero());
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}

// TODO: implement aiming
// fwd/back based on pose
