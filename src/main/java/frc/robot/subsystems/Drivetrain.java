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
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	// Used to track odometry
	public final SwerveDriveOdometry pose;
	public final SwerveDrivePoseEstimator poseEstimator;
	public final Limelight limelight = new Limelight("limelight");

	private final SysIdRoutine sysId;

	public Drivetrain(GyroIO gyroIO, final ModuleIO fl, final ModuleIO fr, final ModuleIO bl, final ModuleIO br) {
		if(gyroIO == null) gyroIO = new GyroIOSim(this);

		this.gyro = gyroIO;
		this.modules[0] = new SwerveModule(fl, Place.FrontLeft);
		this.modules[1] = new SwerveModule(fr, Place.FrontRight);
		this.modules[2] = new SwerveModule(bl, Place.BackLeft);
		this.modules[3] = new SwerveModule(br, Place.BackRight);

		this.poseEstimator = new SwerveDrivePoseEstimator(
			this.kinematics,
			new Rotation2d(this.gyroInputs.yawPosition),
			this.modulePositions(),
			new Pose2d()
		);

		this.pose = new SwerveDriveOdometry(
			this.kinematics,
			new Rotation2d(this.gyroInputs.yawPosition),
			this.modulePositions(),
			new Pose2d()
		);

		// Configure SysId
		this.sysId = new SysIdRoutine(
			new SysIdRoutine.Config(
				null,
				null,
				null,
				state -> Logger.recordOutput("Drive/SysIdState", state.toString())
			),
			new SysIdRoutine.Mechanism(voltage -> {
				for(int i = 0; i < 4; i++) {
					this.modules[i].runCharacterization(voltage.in(Units.Volts));
				}
			}, null, this)
		);
	}

	public void control(ChassisSpeeds speeds) {
		speeds = ChassisSpeeds.discretize(speeds, 0.02);

		Logger.recordOutput("Drivetrain/dx", speeds.vxMetersPerSecond);
		Logger.recordOutput("Drivetrain/dy", speeds.vyMetersPerSecond);
		Logger.recordOutput("Drivetrain/dtheta", speeds.omegaRadiansPerSecond);

		this.control(this.kinematics.toSwerveModuleStates(speeds));
	}

	public void control(final SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxVelocity);

		Logger.recordOutput("Drivetrain/States/desired", states);

		for(int i = 0; i < this.modules.length; i++)
			this.modules[i].control(states[i]);
	}

	public void control(final Drivetrain.State state) { this.control(state.states); }

	public void halt() { this.control(State.locked()); }

	// Convert field-relative ChassisSpeeds to robot-relative ChassisSpeeds.
	public ChassisSpeeds fieldOrientedDrive(final ChassisSpeeds field) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(field, this.pose.getPoseMeters().getRotation());
	}

	// Compensate for wheel rotation.  This prevents the robot
	// from drifting to one side while driving and rotating
	public ChassisSpeeds compensate(final ChassisSpeeds original) {
		// using this function because its just an easy way to rotate the axial/lateral speeds
		return ChassisSpeeds
			.fromFieldRelativeSpeeds(
				original,
				Rotation2d.fromRadians(original.omegaRadiansPerSecond * Constants.Drivetrain.thetaCompensationFactor)
			);
	}

	public void resetOdometryEstimator(final Pose2d pose) {
		this.poseEstimator.resetPosition(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions(), pose);
	}

	public void resetOdometry(final Pose2d newPose) {
		this.pose.resetPosition(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions(), newPose);
	}

	public void reset() {
		this.gyro.reset();
		((JoystickDrive) this.getDefaultCommand()).absoluteTarget = this.pose.getPoseMeters().getRotation();
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

	/**
	 * Updates the pose with vision if close to current position.
	 */
	public void updatePoseEstimatorWithVision() {
		if(this.limelight.hasValidTargets()) {
			// distance from current pose to vision estimated pose
			final double poseDifference = this.poseEstimator
				.getEstimatedPosition()
				.getTranslation()
				.getDistance(this.limelight.getPose2d().getTranslation());
			SmartDashboard.putNumber("pose difference", poseDifference);

			if(poseDifference < 0.5) {
				this.poseEstimator.addVisionMeasurement(this.limelight.getPose2d(), Timer.getFPGATimestamp() - 0.3);
			}
		}
	}

	@Override
	public void periodic() {
		this.gyro.updateInputs(this.gyroInputs);
		Logger.processInputs("Drivetrain/Gyro", this.gyroInputs);

		for(final SwerveModule module : this.modules)
			module.periodic();

		// Update the odometry pose
		this.pose.update(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions());
		this.poseEstimator.update(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions());

		// Fuse odometry pose with vision data if we have it.
		this.updatePoseEstimatorWithVision();

		Logger.recordOutput("Drivetrain/Pose", this.poseEstimator.getEstimatedPosition());
	}

	/** Returns a command to run a quasistatic test in the specified direction. */
	public Command sysIdQuasistatic(final SysIdRoutine.Direction direction) {
		return this.sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	public Command sysIdDynamic(final SysIdRoutine.Direction direction) {
		return this.sysId.dynamic(direction);
	}

	public boolean getHasValidTargetsSim() {
		final double heading = this.poseEstimator.getEstimatedPosition().getRotation().getDegrees();

		return heading > 135 || heading < -135;
	}
}
