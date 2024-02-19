package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

public final class AutonomousRoutines {

	public static SendableChooser<Command> createAutonomousChooser(final Drivetrain drivetrain) {

		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.addOption("Drive test trajectory", new SequentialCommandGroup(runTrajectory("test", drivetrain)));

		// Set up SysId routines
		chooser.addOption(
			"Drive SysId (Quasistatic Forward)",
			drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		chooser.addOption(
			"Drive SysId (Quasistatic Reverse)",
			drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		chooser.addOption(
			"Drive SysId (Dynamic Forward)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
		chooser.addOption(
			"Drive SysId (Dynamic Reverse)", drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
   
		return chooser;
	}

	public static Command runTrajectory(String trajectoryName, Drivetrain drivetrain) {

		ChoreoTrajectory traj = Choreo.getTrajectory(trajectoryName);
		SmartDashboard.putString("Choreo/Trajectory Name", trajectoryName);
		SmartDashboard.putNumber("Choreo/Initial X", traj.getInitialPose().getX());
		SmartDashboard.putNumber("Choreo/Initial Y", traj.getInitialPose().getY());
		SmartDashboard.putNumber("Choreo/Initial Rotation", traj.getInitialPose().getRotation().getDegrees());

		var thetaController = new PIDController(0.1, 0, 0);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		Command swerveCommand = Choreo
			.choreoSwerveCommand(
				traj, // Choreo trajectory from above
				drivetrain::getBlueOriginPose, // A function that returns the current field-relative pose of the robot: 
				// your wheel or vision odometry
				new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative X
				// translation (input: X error in meters, output: m/s).
				new PIDController(0.1, 0.0, 0.0), // PIDController for field-relative Y
				// translation (input: Y error in meters, output: m/s).
				thetaController, // PID constants to correct for rotation error
				(ChassisSpeeds speeds) -> drivetrain
					.drive(
						// needs to be robot-relative
						speeds.vxMetersPerSecond,
						speeds.vyMetersPerSecond,
						speeds.omegaRadiansPerSecond,
						false
					),
				() -> shouldMirror(), // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
				drivetrain // The subsystem(s) to require, typically your drive subsystem only
			);

			// Construct initial auto pose based on current pose angle and choreo starting translation
			Pose2d initialPose = new Pose2d(getPoseForAlliance(traj.getInitialPose()).getTranslation(), drivetrain.getPose().getRotation());

		return Commands
			.sequence(
				Commands.runOnce(() -> drivetrain.setModuleStates(SwerveModule.State.forward())),
				// new WaitCommand(0.5),
				Commands.runOnce(() -> drivetrain.resetOdometry(initialPose)),
				swerveCommand,
				drivetrain.run(() -> drivetrain.drive(0, 0, 0, false))
			);

	}

	/*
	 * Checks if the choreo trajectory should be mirrored based on alliance color
	 */
	private static boolean shouldMirror() {
		return Robot.instance.alliance.isPresent() && Robot.instance.alliance.get() == Alliance.Red;
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(Pose2d initialPose) {
		if (shouldMirror()) {
			return new Pose2d(
				initialPose.getX(), 
				Constants.FIELD_HEIGHT_METERS - initialPose.getY(), 
				initialPose.getRotation().unaryMinus()
			);
		}
		return initialPose;
	}
}
