package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterIO;
import frc.robot.subsystems.ShooterIO.ShooterIOInputs;
import frc.robot.vision.Limelight;

/*
 * Adjusts the robot angle and shooter angle until the shooter is lined up to shoot at the speaker
 */
public class AimAtSpeaker {
	public final Drivetrain drivetrain;
	public final ShooterIO shooter;
	public final Limelight shooterLimelight;

	private final PIDController absoluteController = Constants.Drivetrain.visionAbsoluteRotationErrorPID
		.createController();
	private final SimpleMotorFeedforward ffw = new SimpleMotorFeedforward(0, 0, 0);

	private final Measure<Angle> horizontalAngleTolerance = Units.Degrees.of(2);
	private final Measure<Angle> verticalAngleTolerance = Units.Degrees.of(1);
	private final Measure<Angle> standardForwardAngle = Units.Degrees.of(45);

	private boolean aimBackwards; // the shooter will aim backwards

	public AimAtSpeaker(final Drivetrain drivetrain, final ShooterIO shooter, final Limelight shooterLimelight) {
		super();

		this.drivetrain = drivetrain;
		this.shooter = shooter;
		this.shooterLimelight = shooterLimelight;

		this.absoluteController.enableContinuousInput(-0.5, 0.5);

		this.addRequirements(Robot.cont.shooter);
	}

	@Override
	public ChassisSpeeds modify(final ChassisSpeeds control) {
		if(this.drivetrain.limelightShooter.hasValidTargets()) { // rotate the robot to center the apriltag in view
			if(
				Math
					.abs(
						this.drivetrain.limelightShooter.getTargetHorizontalOffset().in(Units.Degrees)
					) > this.horizontalAngleTolerance.in(Units.Degrees)
			) {
				return new ChassisSpeeds(
					control.vxMetersPerSecond,
					control.vyMetersPerSecond,
					this.ffw
						.calculate(
							this.absoluteController
								.calculate(
									this.drivetrain.limelightShooter.getTargetHorizontalOffset().in(Units.Rotations),
									0
								)
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
								this
									.getAllianceSpeakerCoordinate()
									.minus(this.drivetrain.est.getEstimatedPosition().getTranslation())
									.getAngle()
									.getRotations()
							),
						-1,
						1
					)
			);
		}
	}

	private void aimShooter() {
		if(this.drivetrain.limelightShooter.hasValidTargets()) { // rotate the shooter to center the apriltag in view
			final Measure<Angle> verticalMeasurement = this.drivetrain.limelightShooter.getTargetVerticalOffset();

			if(Math.abs(verticalMeasurement.in(Units.Degrees)) > this.verticalAngleTolerance.in(Units.Degrees)) {
				final ShooterIOInputs inputs = new ShooterIOInputs();
				this.shooter.updateInputs(inputs);

				final Measure<Angle> newAngle = inputs.angle.plus(verticalMeasurement);
				// this.shooter.rotate(newAngle);
			}
		} else {
			// move shooter up to angle where tag can be seen
			// this.shooter.rotate(standardForwardAngle);
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
}
