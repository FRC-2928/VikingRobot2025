package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.ReverseIntakeTranslation;
import frc.robot.commands.shooter.IntakeGround;
import frc.robot.commands.shooter.ShootSpeaker;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;

public final class AutonomousRoutines {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"Drive test trajectory",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("ManSysId")),
					AutonomousRoutines.choreo(Choreo.getTrajectory("ManSysId"))
				)
			);

		chooser
			.addOption(
				"Tri",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("Test")),
					AutonomousRoutines.choreo(Choreo.getTrajectory("Test"))
				)
			);

		chooser
			.addOption(
				"Spinny",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("Spinny")),
					AutonomousRoutines.choreo(Choreo.getTrajectory("Spinny"))
				)
			);

		chooser.addOption("TestShoot", new SequentialCommandGroup(new IntakeGround(true), new ShootSpeaker(false)));

		chooser
			.addOption(
				"[comp] Four Note Middle",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("4Note.1")),
					new ShootSpeaker(false).withTimeout(4),
					AutonomousRoutines.choreo(Choreo.getTrajectory("4Note.1")),
					new IntakeGround(true).withTimeout(2),
					new ReverseIntakeTranslation(),
					new ShootSpeaker(false).withTimeout(4),
					AutonomousRoutines.choreo(Choreo.getTrajectory("4Note.3")),
					new IntakeGround(true).withTimeout(2),
					new ReverseIntakeTranslation(),
					new ShootSpeaker(false).withTimeout(4),
					AutonomousRoutines.choreo(Choreo.getTrajectory("4Note.5")),
					new IntakeGround(true).withTimeout(2),
					new ReverseIntakeTranslation(),
					new ShootSpeaker(false).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[comp] Four Note RTL",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("4Note.1")),
					new ShootSpeaker(false).withTimeout(4),
					AutonomousRoutines.choreo(Choreo.getTrajectory("4Note.1")),
					new IntakeGround(true).withTimeout(2),
					new ReverseIntakeTranslation(),
					new ShootSpeaker(false).withTimeout(4),
					AutonomousRoutines.choreo(Choreo.getTrajectory("4Note.2")),
					new IntakeGround(true).withTimeout(2),
					new ReverseIntakeTranslation(),
					new ShootSpeaker(false).withTimeout(4),
					AutonomousRoutines.choreo(Choreo.getTrajectory("4Note.3")),
					new IntakeGround(true).withTimeout(2),
					new ReverseIntakeTranslation(),
					new ShootSpeaker(false).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[comp] Two Note Center",
				new SequentialCommandGroup(
					new ShootSpeaker(false).withTimeout(4),
					new IntakeGround(true).withTimeout(2),
					new ShootSpeaker(false).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"Four Note Amp side",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("fourNoteClose.1")),
					new ShootSpeaker(false),
					AutonomousRoutines.choreo(Choreo.getTrajectory("fourNoteClose.1")),
					new IntakeGround(true),
					new ParallelCommandGroup(
						AutonomousRoutines.choreo(Choreo.getTrajectory("fourNoteClose.2")),
						new ShootSpeaker(false)
					),
					new IntakeGround(true),
					new ShootSpeaker(false),
					AutonomousRoutines.choreo(Choreo.getTrajectory("fourNoteClose.3")),
					new IntakeGround(true),
					new ShootSpeaker(false)
				)
			);

		chooser
			.addOption(
				"Two Note Center D",
				new SequentialCommandGroup(
					AutonomousRoutines.setInitialPose(Choreo.getTrajectory("TwoNoteCenterD.1")),
					new ShootSpeaker(false),
					AutonomousRoutines.choreo(Choreo.getTrajectory("TwoNoteCenterD.1")),
					new IntakeGround(true),
					AutonomousRoutines.choreo(Choreo.getTrajectory("TwoNoteCenterD.2")),
					new ShootSpeaker(false)
				)
			);

		return chooser;
	}

	public static Command setInitialPose(final ChoreoTrajectory trajectory) {
		return Commands.runOnce(() -> {
			Robot.cont.drivetrain
				.reset(
					new Pose2d(
						AutonomousRoutines.getPoseForAlliance(trajectory.getInitialPose()).getTranslation(),
						Robot.cont.drivetrain.est.getEstimatedPosition().getRotation()
					)
				);

			Logger.recordOutput("Drivetrain/Choreo/x0", trajectory.getInitialPose().getX());
			Logger.recordOutput("Drivetrain/Choreo/y0", trajectory.getInitialPose().getY());
			Logger.recordOutput("Drivetrain/Choreo/r0", trajectory.getInitialPose().getRotation().getDegrees());
		});
	}

	public static Command choreo(final ChoreoTrajectory trajectory) {
		final ChoreoControlFunction controller = Choreo
			.choreoSwerveController(
				Constants.Drivetrain.Choreo.x.createController(), // PID to correct for field-relative X error
				Constants.Drivetrain.Choreo.y.createController(), // PID to correct for field-relative Y error
				Constants.Drivetrain.Choreo.theta.createController()
			);

		final Timer timer = new Timer();
		return new FunctionalCommand(timer::restart, () -> {
			final ChoreoTrajectoryState poseDemand = trajectory
				.sample(timer.get(), DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);

			Logger.recordOutput("Choreo/DesiredPose", poseDemand.getPose());

			Robot.cont.drivetrain
				.controlRobotOriented(controller.apply(Robot.cont.drivetrain.blueOriginPose(), poseDemand));
		}, interrupted -> {
			timer.stop();
			if(interrupted) {
				Robot.cont.drivetrain.controlRobotOriented(new ChassisSpeeds());
			}
		}, () -> timer.hasElapsed(trajectory.getTotalTime()), Robot.cont.drivetrain);
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
		if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return new Pose2d(
				initialPose.getX(),
				Constants.fieldDepth.in(Units.Meters) - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}
}
