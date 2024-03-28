package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Tuning;
import frc.robot.subsystems.ShooterIO.Demand;
import frc.robot.utils.STalonFX;

public class ShootSpeaker extends Command {
	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public ShootSpeaker(final boolean triggerFire) { this(triggerFire, Constants.Shooter.readyShootRear); }

	public ShootSpeaker(final boolean triggerFire, final Measure<Angle> startAngle) {
		this.addRequirements(Robot.cont.shooter);
		this.triggerFire = triggerFire;
		this.rearAngle = startAngle;
	}

	public final boolean triggerFire;
	public final Measure<Angle> rearAngle;

	private double fired;
	private final SimpleMotorFeedforward targetRotationFeedforward = new SimpleMotorFeedforward(0, 10);

	@Override
	public void initialize() { this.fired = -1; }

	@Override
	public void execute() {
		final boolean forward = Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getCos() < 0;
		final boolean current = Robot.cont.shooter.inputs.angle.in(Units.Degrees) - 90 < 0;
		if(this.pivot.getVelocity().getValue() > Constants.Shooter.pivotMaxVelocityShoot) {
			Robot.cont.shooter.io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
		}
		Robot.cont.drivetrain.limelightShooter.setPipeline(forward ? 0 : 1);
		if(forward == current) {
			if(Robot.cont.drivetrain.limelightShooter.hasValidTargets()) {
				final Measure<Angle> po = Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset();
				final Measure<Angle> yo = Robot.cont.drivetrain.limelightShooter.getTargetVerticalOffset();

				Logger.recordOutput("Shooter/ShootSpeaker/tx", po);
				Logger.recordOutput("Shooter/ShootSpeaker/ty", yo);

				Robot.cont.drivetrain
					.control(
						Robot.cont.drivetrain.joystickSpeeds
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											this.targetRotationFeedforward
												.calculate(yo.in(Units.Rotations) * (forward ? 1 : -1))
										)
									)
							)
					);

				if(Math.abs(po.in(Units.Degrees)) >= 1.25 && this.fired == -1) {
					Robot.cont.shooter.io.rotate(Robot.cont.shooter.inputs.angle.minus(po));
				} else {
					if(
						((Robot.cont.shooter.inputs.flywheelSpeedA
							.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get()
							|| Robot.cont.operatorOI.overrideShoot.getAsBoolean())
							&& (Robot.cont.driverOI.intakeShoot.getAsBoolean() || !this.triggerFire))
							|| this.fired != -1
					) {
						Robot.cont.shooter.io.runFeeder(Demand.Forward);
						if(this.fired == -1) this.fired = Timer.getFPGATimestamp();
					}
				}
			} else if(!forward) {
				Robot.cont.drivetrain
					.control(
						Robot.cont.drivetrain.joystickSpeeds
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											this.targetRotationFeedforward
												.calculate(
													Robot.cont.drivetrain.limelightBack
														.getTargetHorizontalOffset()
														.in(Units.Rotations)
												)
										)
									)
							)
					);
			} else {
				Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
			}
		} else {
			Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
			Robot.cont.shooter.io.rotate(forward ? Constants.Shooter.readyShootFront : this.rearAngle);
		}
	}

	@Override
	public void end(final boolean interrupted) {
		Robot.cont.shooter.io
			.rotate(
				Robot.cont.shooter.inputs.holdingNote ? Constants.Shooter.readyDrive : Constants.Shooter.readyIntake
			);
		Robot.cont.shooter.io.runFlywheels(0);
		Robot.cont.shooter.io.runFeeder(Demand.Halt);
	}

	@Override
	public boolean isFinished() {
		return this.fired != -1 && Timer.getFPGATimestamp() - this.fired >= Constants.Shooter.fireTimeout;
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() { return InterruptionBehavior.kCancelIncoming; }
}
