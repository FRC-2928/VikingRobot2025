package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.choreo.lib.*;
import com.pathplanner.lib.auto.*;
import com.pathplanner.lib.path.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.DriveTime;
import frc.robot.commands.shooter.*;

public final class Autonomous {
	public static SendableChooser<Command> createAutonomousChooser() {
		final SendableChooser<Command> chooser = new SendableChooser<>();

		chooser
			.addOption(
				"[comp] Five Note",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear),
					Autonomous.setInitialPose(Choreo.getTrajectory("MiddleFiveNote.1")),
					new ShootSpeaker(false, Units.Degrees.of(107)).withTimeout(4),
					Autonomous.choreo(Choreo.getTrajectory("MiddleFiveNote.1")),
					new IntakeGround(true).withTimeout(2),
					Autonomous.onTheFlyPath("MiddleFiveNote.3"),
					new ShootSpeaker(false, Units.Degrees.of(119)).withTimeout(4),
					new ReadyShooter(Constants.Shooter.readyIntake),
					new IntakeGround(true).withTimeout(2),
					Autonomous.onTheFlyPath("MiddleFiveNote.4"),
					new ShootSpeaker(false, Units.Degrees.of(117)).withTimeout(4),
					new ReadyShooter(Constants.Shooter.readyIntake),
					new IntakeGround(true).withTimeout(2),
					Autonomous.onTheFlyPath("MiddleFiveNote.4"),
					new ShootSpeaker(false, Units.Degrees.of(118)).withTimeout(4),
					Autonomous.choreo(Choreo.getTrajectory("MiddleFiveNote.4")),
					new IntakeGround(true).withTimeout(2),
					Autonomous.onTheFlyPath("MiddleFiveNote.5"),
					Autonomous.choreo(Choreo.getTrajectory("MiddleFiveNote.5")),
					new ShootSpeaker(false, Units.Degrees.of(118)).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[testing] Five Note Middle",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear),
					Autonomous.setInitialPose(Choreo.getTrajectory("5Note.1")),
					new ShootSpeaker(false, Units.Degrees.of(107)).withTimeout(4),
					Autonomous.choreo(Choreo.getTrajectory("5Note.1")),
					new IntakeGround(true).withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.onTheFlyPath("5Note.4")
						// AutonomousRoutines.choreo(Choreo.getTrajectory("5Note.3"))
					),
					new ShootSpeaker(false, Units.Degrees.of(119)).withTimeout(4),
					new IntakeGround(true).withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.onTheFlyPath("5Note.6")
						// AutonomousRoutines.choreo(Choreo.getTrajectory("5Note.5"))
					),
					new ShootSpeaker(false, Units.Degrees.of(117)).withTimeout(4),
					new IntakeGround(true).withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.choreo(Choreo.getTrajectory("5Note.7"))
					),
					new ShootSpeaker(false, Units.Degrees.of(117)).withTimeout(4),
					Autonomous.choreo(Choreo.getTrajectory("5Note.8")),
					new IntakeGround(true).withTimeout(2),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.choreo(Choreo.getTrajectory("5Note.9"))
					),
					new ShootSpeaker(false, Units.Degrees.of(118)).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[testing] Source Side Center Note",
				new SequentialCommandGroup(
					Autonomous.setInitialPose(Choreo.getTrajectory("SourceSideCenterNote.1")),
					new ReadyShooter(Constants.Shooter.readyShootRear),
					new ShootSpeaker(false, Units.Degrees.of(107)).withTimeout(4),
					Autonomous.choreo(Choreo.getTrajectory("SourceSideCenterNote.1")),
					new IntakeGround(true).withTimeout(2),
					Autonomous.onTheFlyPath("SourceSideCenterNote.2"),
					new ParallelCommandGroup(
						new ReadyShooter(Constants.Shooter.readyShootRear),
						Autonomous.choreo(Choreo.getTrajectory("SourceSideCenterNote.2"))
					),
					new ShootSpeaker(false, Units.Degrees.of(119)).withTimeout(4)
				)
			);

		chooser
			.addOption(
				"[comp] Drive",
				new SequentialCommandGroup(new ReadyShooter(Constants.Shooter.readyShootRear), new DriveTime(1))
			);

		chooser
			.addOption(
				"[comp] Shoot/Drive",
				new SequentialCommandGroup(
					new ReadyShooter(Constants.Shooter.readyShootRear),
					new ShootSpeaker(false),
					new DriveTime(1)
				)
			);

		chooser
			.addOption(
				"[comp] Amp Side Delay Strafe",
				new SequentialCommandGroup(
					Autonomous.setInitialPose(Choreo.getTrajectory("AmpSideCenterNote.1")),
					new WaitCommand(10),
					new ReadyShooter(Constants.Shooter.readyShootRear)
						.alongWith(Autonomous.choreo(Choreo.getTrajectory("AmpSideCenterNote.1"))),
					new ShootSpeaker(false).withTimeout(4),
					Autonomous.onTheFlyPath("AmpSideCenterNote.2"),

					Autonomous.choreo(Choreo.getTrajectory("AmpSideCenterNote.2")),
					new IntakeGround(true)
				)
			);

		chooser
			.addOption(
				"[comp] Two Note",
				new SequentialCommandGroup(
					new ShootSpeaker(false).withTimeout(4),
					new IntakeGround(true).withTimeout(2),
					new ShootSpeaker(false).withTimeout(4)
				)
			);

		return chooser;
	}

	public static Command setInitialPose(final ChoreoTrajectory trajectory) {
		return Commands.runOnce(() -> {
			Robot.cont.drivetrain
				.reset(
					new Pose2d(
						Autonomous.getPoseForAlliance(trajectory.getInitialPose()).getTranslation(),
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
				Constants.Drivetrain.Auto.x.createController(), // PID to correct for field-relative X error
				Constants.Drivetrain.Auto.y.createController(), // PID to correct for field-relative Y error
				Constants.Drivetrain.Auto.theta.createController()
			);

		final Timer timer = new Timer();
		return new FunctionalCommand(timer::restart, () -> {
			final ChoreoTrajectoryState poseDemand = trajectory
				.sample(
					timer.get(),
					DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
				);

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

	public Command pathPlannerLib(final String trajectory) {
		final PathPlannerPath choreoPath = PathPlannerPath.fromChoreoTrajectory(trajectory);
		return AutoBuilder.followPath(choreoPath);
	}

	public static Command onTheFlyPath(final String nextChoreoPath) {
		final ChoreoTrajectory nextTrajectory = Choreo.getTrajectory(nextChoreoPath);

		return AutoBuilder
			.pathfindToPoseFlipped(
				nextTrajectory.getInitialPose(),
				new PathConstraints(
					Constants.Drivetrain.maxVelocity.in(Units.MetersPerSecond),
					3,
					Constants.Drivetrain.maxAngularVelocity.in(Units.RadiansPerSecond),
					2
				)
			)
			.alongWith(
				new InstantCommand(
					() -> Logger
						.recordOutput(
							"Drivetrain/OnTheFlyTarget",
							Autonomous.getPathPlannerPoseForAlliance(nextTrajectory).getInitialPose()
						)
				)
			);
	}

	/*
	 * Returns the original or mirrored pose depending on alliance color (since the field is flipped)
	 */
	private static Pose2d getPoseForAlliance(final Pose2d initialPose) {
		if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
			return new Pose2d(
				initialPose.getX(),
				Constants.fieldDepth.in(Units.Meters) - initialPose.getY(),
				initialPose.getRotation().unaryMinus()
			);
		} else return initialPose;
	}

	private static ChoreoTrajectory getPathPlannerPoseForAlliance(final ChoreoTrajectory initialPose) {
		if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
			return initialPose.flipped();
		} else return initialPose;
	}
}
