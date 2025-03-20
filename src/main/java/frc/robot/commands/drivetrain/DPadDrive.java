package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.oi.DriverOI;
import frc.robot.subsystems.Drivetrain;

public class DPadDrive extends Command{
	public final Drivetrain drivetrain;
	public final DriverOI oi;

	private final ProfiledPIDController absoluteController = Constants.Drivetrain.absoluteRotationPID
			.createProfiledController(Constants.Drivetrain.absoluteRotationConstraints);

	public DPadDrive(final Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
		this.oi = RobotContainer.getInstance().driverOI;
		this.absoluteController.enableContinuousInput(-1, 1);

	}

	@Override
	public void execute() {
		final Translation2d translate = translation();
		final ChassisSpeeds robotOrientedSpeeds = new ChassisSpeeds(translate.getX(), translate.getY(), Rotation2d.kZero.getDegrees());
		drivetrain.control(robotOrientedSpeeds);
	}

	private Translation2d translation() {
		boolean right = this.oi.reefMovementRight.getAsBoolean();
		boolean left = this.oi.reefMovementLeft.getAsBoolean();

		if (left) {
			return new Translation2d(0d, 0.15d);
		}
		if (right) {
			return new Translation2d(0d, -0.15d);
		}
		return new Translation2d(0d, 0d);
	}
}