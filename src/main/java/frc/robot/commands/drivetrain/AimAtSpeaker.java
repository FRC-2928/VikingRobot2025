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

/*
 * Adjusts the robot angle and shooter angle until the shooter is lined up to shoot at the speaker
 */
public class AimAtSpeaker extends DrivetrainModifier.Modification {
	public final Drivetrain drivetrain;

	private final PIDController absoluteController = Constants.Drivetrain.visionAbsoluteRotationErrorPID
		.createController();
	private final SimpleMotorFeedforward ffw = new SimpleMotorFeedforward(0, 0, 0);

	private final Measure<Angle> horizontalAngleTolerance = Units.Degrees.of(2);
	private final Measure<Angle> verticalAngleTolerance = Units.Degrees.of(1);

	public AimAtSpeaker(final Drivetrain drivetrain) {
		super();

		this.drivetrain = drivetrain;

		this.absoluteController.enableContinuousInput(-0.5, 0.5);

		this.addRequirements(Robot.cont.shooter);
	}

	@Override
	public ChassisSpeeds modify(final ChassisSpeeds control) {
		if(this.drivetrain.limelight.hasValidTargets()) { // rotate the robot to center the apriltag in view
			if(
				Math
					.abs(
						this.drivetrain.limelight.getTargetHorizontalOffset().in(Units.Degrees)
					) > this.horizontalAngleTolerance.in(Units.Degrees)
			) {
				return new ChassisSpeeds(
					control.vxMetersPerSecond,
					control.vyMetersPerSecond,
					this.ffw
						.calculate(
							this.absoluteController
								.calculate(this.drivetrain.limelight.getTargetHorizontalOffset().in(Units.Rotations), 0)
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
		if(this.drivetrain.limelight.hasValidTargets()) { // rotate the robot to center the apriltag in view
			final Measure<Angle> verticalMeasurement = this.drivetrain.limelight.getTargetVerticalOffset();
			//TODO: output voltage to the shooter angle subsystem
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
