package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainModifier;
import frc.robot.subsystems.ShooterIO;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;
import frc.robot.vision.Limelight;

/*
 * Adjusts the robot angle and shooter angle until the shooter is lined up to shoot at the speaker
 */
public class AimAtSpeaker extends DrivetrainModifier.Modification {
	public final Drivetrain drivetrain;
	public final ShooterIO shooter;
	public final Limelight shooterLimelight;
	// public final Limelight backLimelight; TODO use back limelight to find the tag faster (in case bot is facing away from speaker)

	private final PIDController absoluteController = Constants.Drivetrain.visionAbsoluteRotationErrorPID
		.createController();
	private final SimpleMotorFeedforward ffw = new SimpleMotorFeedforward(0, 0, 0);

	private final Measure<Angle> horizontalAngleTolerance = Units.Degrees.of(2);
	private final Measure<Angle> verticalAngleTolerance = Units.Degrees.of(1);
	private final Measure<Angle> standardForwardAngle = Units.Degrees.of(45);
	private final Measure<Angle> standardBackwardsAngle = Units.Degrees.of(-45);

	public AimAtSpeaker(final Drivetrain drivetrain, final ShooterIO shooter, final Limelight shooterLimelight) {
		super();

		this.drivetrain = drivetrain;
		this.shooter = shooter;
		this.shooterLimelight = shooterLimelight;

		this.absoluteController.enableContinuousInput(-0.5, 0.5);

		this.addRequirements(Robot.cont.shooter, Robot.cont.shooter);
	}

	@Override
	public ChassisSpeeds modify(final ChassisSpeeds control) {
		if(this.shooterLimelight.hasValidTargets()) { // rotate the robot to center the apriltag in view
			if(
				Math
					.abs(
						this.shooterLimelight.getTargetHorizontalOffset().in(Units.Degrees)
					) > this.horizontalAngleTolerance.in(Units.Degrees)
			) {
				return new ChassisSpeeds(
					control.vxMetersPerSecond,
					control.vyMetersPerSecond,
					this.ffw
						.calculate(
							this.absoluteController
								.calculate(this.shooterLimelight.getTargetHorizontalOffset().in(Units.Rotations), 0)
						)
				);
			} else {
				return new ChassisSpeeds(control.vxMetersPerSecond, control.vyMetersPerSecond, 0);
			}
		} else {
			return new ChassisSpeeds(
				control.vxMetersPerSecond,
				control.vyMetersPerSecond,
				MathUtil
					.clamp(
						this.absoluteController
							.calculate(
								this.drivetrain.est.getEstimatedPosition().getRotation().getRotations(),
								this.optimizedRobotFieldAngle().getRotations()
							),
						-1,
						1
					)
			);
		}
	}

	private void aimShooter() {
		if(this.shooterLimelight.hasValidTargets()) { // rotate the shooter to center the apriltag in view
			final Measure<Angle> verticalMeasurement = this.shooterLimelight.getTargetVerticalOffset();

			if(Math.abs(verticalMeasurement.in(Units.Degrees)) > this.verticalAngleTolerance.in(Units.Degrees)) {
				final ShooterIOInputs inputs = new ShooterIOInputs();
				this.shooter.updateInputs(inputs);

				this.shooter.rotate(inputs.angle.plus(verticalMeasurement));
			}
		} else {
			// move shooter up to angle where tag can be seen
			if(this.shouldAimBackwards()) {
				this.shooter.rotate(this.standardBackwardsAngle);
			} else {
				this.shooter.rotate(this.standardForwardAngle);
			}
		}
	}

	/*
	 * Returns the translation of the current alliance's speaker relative to the alliance origin
	 */
	private Translation2d getAllianceSpeakerCoordinate() {
		if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return new Translation2d(0, Constants.fieldDepth.minus(Units.Meters.of(5.547868)).in(Units.Meters)); // Red speaker coordinate
		} else {
			return new Translation2d(0, 5.547868); // Blue speaker coordinate
		}
	}

	/*
	 * Whether the robot should aim forwards or backwards
	 */
	private boolean shouldAimBackwards() {
		return Math
			.abs(
				this.drivetrain.est.getEstimatedPosition().getRotation().minus(this.robotToSpeakerAngle()).getDegrees()
			) > 90;
	}

	private Rotation2d optimizedRobotFieldAngle() {
		if(this.shouldAimBackwards()) {
			return this.robotToSpeakerAngle().plus(Rotation2d.fromRotations(0.5));
		}
		return this.robotToSpeakerAngle();
	}

	private Rotation2d robotToSpeakerAngle() {
		return this
			.getAllianceSpeakerCoordinate()
			.minus(this.drivetrain.est.getEstimatedPosition().getTranslation())
			.getAngle();
	}
}
