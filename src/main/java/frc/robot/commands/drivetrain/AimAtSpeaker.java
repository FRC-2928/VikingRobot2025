package frc.robot.commands.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/*
 * Adjusts the robot angle and shooter angle until the shooter is lined up to shoot at the speaker
 */
public class AimAtSpeaker extends Command {

    public final Drivetrain drivetrain;

	private final PIDController absoluteController = Constants.Drivetrain.visionAbsoluteRotationErrorPID.createController();
	// private final PIDController shooterAngleController = Constants.Drivetrain.targetVerticalControllerPID.createController();

	private final double horizontalAngleToleranceDegrees = 1;
	// private final double verticalAngleToleranceDegrees = 1;
	private final double omegaRadiansPerSecondStaticFeedforward = 0.1;

    public AimAtSpeaker(final Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

		this.absoluteController.enableContinuousInput(-0.5, 0.5);

		this.addRequirements(drivetrain);
    }

	@Override
	public void execute() {
		aimRobotBase();
		aimShooter();
	}

	/**
	 * Uses LL and odometry to find the ideal robot angle
	 */
	private void aimRobotBase() {
		if (this.drivetrain.limelight.hasValidTargets()) { // rotate the robot to center the apriltag in view
			if (Math.abs(this.drivetrain.limelight.getTargetHorizontalOffset()) > horizontalAngleToleranceDegrees) {
				final Rotation2d horizontalMeasurement = Rotation2d.fromDegrees(this.drivetrain.limelight.getTargetHorizontalOffset());
				final double absoluteAngleSpeed = this.absoluteController.calculate(horizontalMeasurement.getRotations(), 0);
				final double feedForward = Math.signum(absoluteAngleSpeed) * omegaRadiansPerSecondStaticFeedforward;
				final double omegaRadiansPerSecond = MathUtil.clamp(absoluteAngleSpeed, -1d, 1d) * Constants.Drivetrain.maxAngularVelocityRadPerSec + feedForward;
				Logger.recordOutput("AimAtSpeaker/OmegaRadPerSec", omegaRadiansPerSecond);
				this.drivetrain.drive(0, 0, omegaRadiansPerSecond, false);
			} else {
				this.drivetrain.drive(0, 0, 0, false);
			}
		} else { // rotate the bot toward where it thinks the speaker is based on odometry
			Translation2d speakerLocation = getAllianceSpeakerCoordinate();
			// subtract robot coordinate from speaker coordinate to get vector pointing at speaker
			Translation2d odometryTranslationRelativeToSpeaker = speakerLocation.minus(this.drivetrain.getPose().getTranslation());
			// get angle from this vector using arctan
			double angleSetpoint = Rotation2d.fromRadians(
				Math.atan2(odometryTranslationRelativeToSpeaker.getY(), odometryTranslationRelativeToSpeaker.getX())
				).getRotations();
			double measurement = this.drivetrain.getPose().getRotation().getRotations();

			final double absoluteAngleSpeed = this.absoluteController.calculate(measurement, angleSetpoint);
			final double omegaRadiansPerSecond = MathUtil.clamp(absoluteAngleSpeed, -1d, 1d) * Constants.Drivetrain.maxAngularVelocityRadPerSec;
			Logger.recordOutput("AimAtSpeaker/OmegaRadPerSec", omegaRadiansPerSecond);
			this.drivetrain.drive(0, 0, omegaRadiansPerSecond, false);
		}
	}
    
	/**
	 * Uses LL to find the ideal shooter angle
	 */
    private void aimShooter() {
		if (this.drivetrain.limelight.hasValidTargets()) { // rotate the robot to center the apriltag in view
			final double verticalMeasurement = this.drivetrain.limelight.getTargetVerticalOffset();
			// final double shooterAngleSpeed = this.shooterAngleController.calculate(verticalMeasurement, 0);
			//TODO: output voltage to the shooter angle subsystem
		}
	}

	/*
	 * Returns the translation of the current alliance's speaker relative to the alliance origin
	 */
	private Translation2d getAllianceSpeakerCoordinate() {
		if (Robot.instance.alliance.isPresent() && Robot.instance.alliance.get() == Alliance.Red) {
			return new Translation2d(0, Constants.FIELD_DEPTH_METERS - 5.547868); // Red speaker coordinate
		}
		return new Translation2d(0, 5.547868); // Blue speaker coordinate
	}
}
