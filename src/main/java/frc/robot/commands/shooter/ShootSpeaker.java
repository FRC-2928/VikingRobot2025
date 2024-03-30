package frc.robot.commands.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

public class ShootSpeaker extends Command {
	public ShootSpeaker(final boolean triggerFire) { this(triggerFire, Constants.Shooter.readyShootRear); }

	public ShootSpeaker(final boolean triggerFire, final Measure<Angle> startAngle) {
		this.addRequirements(Robot.cont.shooter);
		this.triggerFire = triggerFire;
		this.rearAngle = startAngle;
	}

	public final boolean triggerFire;
	public final Measure<Angle> rearAngle;

	private double fired;
	private final SimpleMotorFeedforward targetRotationFeedforward = new SimpleMotorFeedforward(0, 5);

	private final PIDController pitch = new PIDController(4, 0, 0.01);

	@Override
	public void initialize() { this.fired = -1; }

	@Override
	public void execute() {
		final boolean forward = Robot.cont.drivetrain.est.getEstimatedPosition().getRotation().getCos() < 0;
		final boolean current = Robot.cont.shooter.inputs.angle.in(Units.Degrees) - 90 < 0;
		Robot.cont.shooter.io.runFlywheelsVelocity(Tuning.flywheelVelocity.get());
		Robot.cont.drivetrain.limelightShooter.setPipeline(forward ? 0 : 1);
		if(forward == current) {
			final boolean shooterLLSees = Robot.cont.drivetrain.limelightShooter.hasValidTargets();
			final boolean rearLLSees = Robot.cont.drivetrain.limelightRear.hasValidTargets();
			final boolean flywheelSpeed = Robot.cont.shooter.inputs.flywheelSpeedA
				.in(Units.RotationsPerSecond) >= Tuning.flywheelVelocityThreshold.get();
			final boolean pivotVelocity = Math
				.abs(
					Robot.cont.shooter.inputs.angleSpeed.in(Units.RotationsPerSecond)
				) < Constants.Shooter.pivotMaxVelocityShoot.in(Units.RotationsPerSecond);
			final boolean demandFire = Robot.cont.driverOI.intakeShoot.getAsBoolean() || !this.triggerFire;
			final boolean overrideShoot = Robot.cont.operatorOI.overrideShoot.getAsBoolean();

			Logger.recordOutput("Shooter/ShootSpeaker/ShooterLLSees", shooterLLSees);
			Logger.recordOutput("Shooter/ShootSpeaker/RearLLSees", rearLLSees);
			Logger
				.recordOutput(
					"Shooter/ShootSpeaker/PivotAngle",
					Math
						.abs(
							Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset().in(Units.Degrees)
						) < 1.25
				);
			Logger.recordOutput("Shooter/ShootSpeaker/FlywheelSpeed", flywheelSpeed);
			Logger.recordOutput("Shooter/ShootSpeaker/PivotVelocity", pivotVelocity);
			Logger.recordOutput("Shooter/ShootSpeaker/DemandFire", demandFire);
			Logger.recordOutput("Shooter/ShootSpeaker/OverrideShoot", overrideShoot);
			Logger.recordOutput("Shooter/ShootSpeaker/Fired", this.fired != -1);

			if(shooterLLSees) {
				final Measure<Angle> po = Robot.cont.drivetrain.limelightShooter.getTargetHorizontalOffset();
				final Measure<Angle> yo = Robot.cont.drivetrain.limelightShooter
					.getTargetVerticalOffset()
					.times(forward ? 1 : -1);

				Logger.recordOutput("Shooter/ShootSpeaker/tx", po.in(Units.Degrees));
				Logger.recordOutput("Shooter/ShootSpeaker/ty", yo.in(Units.Degrees));

				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/ShooterAlign",
						this.targetRotationFeedforward.calculate(yo.in(Units.Rotations))
					);

				Robot.cont.drivetrain
					.control(
						this
							.norot(Robot.cont.drivetrain.joystickSpeeds)
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											MathUtil
												.clamp(
													this.targetRotationFeedforward
														//.calculate(Math.copySign(Math.pow(yo.in(Units.Rotations), 0.8), yo.in(Units.Rotations)))
														.calculate(yo.in(Units.Rotations)),
													-0.125,
													0.125
												)
										)
									)
							)
					);

				if(Math.abs(po.in(Units.Degrees)) >= 1.25 && this.fired == -1) {
					Robot.cont.shooter.io
						.rotate(
							Units.Rotations
								.of(
									Robot.cont.shooter.inputs.angle.in(Units.Rotations)
										+ this.pitch
											.calculate(
												Math
													.copySign(
														Math.pow(Math.abs(po.in(Units.Rotations)), 1.5),
														po.in(Units.Rotations)
													)
											)
								)
						);
				} else {
					if((((flywheelSpeed && pivotVelocity) || overrideShoot) && demandFire) || this.fired != -1) {
						Robot.cont.shooter.io.runFeeder(Demand.Forward);
						if(this.fired == -1) this.fired = Timer.getFPGATimestamp();
					}

					Robot.cont.shooter.io
						.rotate(Units.Rotations.of(Robot.cont.shooter.inputs.angle.in(Units.Rotations)));
				}
			} else if(!forward) {
				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/RearAlign",
						this.targetRotationFeedforward
							.calculate(
								Robot.cont.drivetrain.limelightRear.getTargetHorizontalOffset().in(Units.Rotations)
							)
					);

				Logger
					.recordOutput(
						"Shooter/ShootSpeaker/txr",
						Robot.cont.drivetrain.limelightRear.getTargetHorizontalOffset().in(Units.Degrees)
					);

				Robot.cont.drivetrain
					.control(
						this
							.norot(Robot.cont.drivetrain.joystickSpeeds)
							.plus(
								Robot.cont.drivetrain
									.rod(
										new ChassisSpeeds(
											0,
											0,
											this.targetRotationFeedforward
												.calculate(
													Robot.cont.drivetrain.limelightRear
														.getTargetHorizontalOffset()
														.in(Units.Rotations)
												)
										)
									)
							)
					);
				Robot.cont.shooter.io.rotate(forward ? Constants.Shooter.readyShootFront : this.rearAngle);
			} else {
				Robot.cont.drivetrain.control(Robot.cont.drivetrain.joystickSpeeds);
				Robot.cont.shooter.io.rotate(forward ? Constants.Shooter.readyShootFront : this.rearAngle);
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

	private ChassisSpeeds norot(final ChassisSpeeds speeds) {
		return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
	}
}
