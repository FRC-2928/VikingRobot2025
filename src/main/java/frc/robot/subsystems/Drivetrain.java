package frc.robot.subsystems;

import edu.wpi.first.units.*;
import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.SwerveModule.Place;
import frc.robot.vision.Limelight;

public class Drivetrain extends SubsystemBase {
	public static class State {
		public final SwerveModuleState[] states;

		public State() { this.states = new SwerveModuleState[4]; }

		public State(final SwerveModuleState[] states) { this.states = states; }

		public static State forward() {
			return new State()
				.set(Place.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
				.set(Place.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
				.set(Place.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(0)))
				.set(Place.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		}

		public static State locked() {
			return new State()
				.set(Place.FrontLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
				.set(Place.FrontRight, new SwerveModuleState(0, Rotation2d.fromDegrees(45)))
				.set(Place.BackLeft, new SwerveModuleState(0, Rotation2d.fromDegrees(-135)))
				.set(Place.BackRight, new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
		}

		public SwerveModuleState get(final Place place) { return this.states[place.index]; }

		public State set(final Place place, final SwerveModuleState state) {
			this.states[place.index] = state;
			return this;
		}
	}

	public final GyroIO gyro;
	public final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	public final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR

	public final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;
	public final SwerveDrivePoseEstimator est;
	public final Limelight limelightNote = new Limelight("limelight-note");
	public final Limelight limelightShooter = new Limelight("limelight-shooter");
	public final Limelight limelightBack = new Limelight("limelight-back");

	private final JoystickDrive joystickDrive = new JoystickDrive(this);
	public ChassisSpeeds joystickSpeeds = new ChassisSpeeds();

	public Drivetrain() {
		this.gyro = switch(Constants.mode) {
		case REAL -> new GyroIOReal();
		case SIM -> new GyroIOSim(this);
		case REPLAY -> new GyroIO() {
		};
		default -> throw new Error();
		};

		this.modules[0] = new SwerveModule(Place.FrontLeft);
		this.modules[1] = new SwerveModule(Place.FrontRight);
		this.modules[2] = new SwerveModule(Place.BackLeft);
		this.modules[3] = new SwerveModule(Place.BackRight);

		this.est = new SwerveDrivePoseEstimator(
			this.kinematics,
			new Rotation2d(this.gyroInputs.yawPosition),
			this.modulePositions(),
			new Pose2d()
		);

		this.setDefaultCommand(this.joystickDrive);
	}

	public void control(ChassisSpeeds speeds) {
		Logger.recordOutput("Drivetrain/dx", speeds.vxMetersPerSecond);
		Logger.recordOutput("Drivetrain/dy", speeds.vyMetersPerSecond);
		Logger.recordOutput("Drivetrain/dtheta", speeds.omegaRadiansPerSecond);

		speeds = this.fod(speeds);
		speeds = this.compensate(speeds);
		speeds = ChassisSpeeds.discretize(speeds, 0.02);

		this.control(this.kinematics.toSwerveModuleStates(speeds));
	}

	public void controlRobotOriented(final ChassisSpeeds speeds) {
		Logger.recordOutput("Drivetrain/dx", speeds.vxMetersPerSecond);
		Logger.recordOutput("Drivetrain/dy", speeds.vyMetersPerSecond);
		Logger.recordOutput("Drivetrain/dtheta", speeds.omegaRadiansPerSecond);

		this
			.control(
				this.kinematics
					.toSwerveModuleStates(
						new ChassisSpeeds(
							-speeds.vxMetersPerSecond,
							-speeds.vyMetersPerSecond,
							speeds.omegaRadiansPerSecond
						)
					)
			);
	}

	public void control(final Drivetrain.State state) { this.control(state.states); }

	public void control(final SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxVelocity);

		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].control(states[i]);
	}

	public void halt() { this.control(State.locked()); }

	public ChassisSpeeds fod(final ChassisSpeeds speeds) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.est.getEstimatedPosition().getRotation());
	}

	public ChassisSpeeds rod(final ChassisSpeeds speeds) {
		return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, this.est.getEstimatedPosition().getRotation());
	}

	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor)
			);
	}

	public void resetAngle() {
		this.reset(new Pose2d(this.est.getEstimatedPosition().getTranslation(), Rotation2d.fromRadians(0)));
		((JoystickDrive) this.getDefaultCommand()).forTarget = Units.Radians.zero();
	}

	public void reset(final Pose2d newPose) {
		this.est.resetPosition(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions(), newPose);
	}

	@AutoLogOutput(key = "Drivetrain/CurrentPositions")
	public SwerveModulePosition[] modulePositions() {
		return Arrays.stream(this.modules).map(module -> module.position).toArray(SwerveModulePosition[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/States/Desired")
	public SwerveModuleState[] desiredModuleStates() {
		return Arrays.stream(this.modules).map(module -> module.current).toArray(SwerveModuleState[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/States/Current")
	public SwerveModuleState[] currerntModuleStates() {
		return Arrays.stream(this.modules).map(module -> module.desired).toArray(SwerveModuleState[]::new);
	}

	// Returns the current odometry pose, transformed to blue origin coordinates.
	@AutoLogOutput(key = "Odometry/BlueOriginPose")
	public Pose2d blueOriginPose() {
		if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return this.est
				.getEstimatedPosition()
				.relativeTo(new Pose2d(Constants.fieldWidth, Constants.fieldDepth, Rotation2d.fromRadians(Math.PI)));
		} else {
			return this.est.getEstimatedPosition();
		}
	}

	@Override
	public void periodic() {
		this.gyro.updateInputs(this.gyroInputs);
		Logger.processInputs("Drivetrain/Gyro", this.gyroInputs);

		this.joystickSpeeds = this.joystickDrive.speeds();
		if(this.getCurrentCommand() == this.joystickDrive) this.control(this.joystickSpeeds);

		for(final SwerveModule module : this.modules)
			module.periodic();

		// Update the odometry pose
		this.est.update(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions());

		/*
		// Fuse odometry pose with vision data if we have it.
		if(this.limelightShooter.hasValidTargets() && this.limelightShooter.getNumberOfAprilTags() >= 2) {
			// distance from current pose to vision estimated pose
			// double poseDifference = this.poseEstimator
			// 	.getEstimatedPosition()
			//	.getTranslation()
			// 	.getDistance(this.limelight.getPose2d().getTranslation());
		
			// if (poseDifference < 0.5) {
			this.est.addVisionMeasurement(this.limelightShooter.getPose2d(), Timer.getFPGATimestamp() - 0.3);
			// }
		}
		*/

		Logger.recordOutput("Drivetrain/Pose", this.est.getEstimatedPosition());
	}
}
