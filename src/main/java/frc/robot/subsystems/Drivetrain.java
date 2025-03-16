package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaePosition;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.subsystems.SwerveModule.Place;
import frc.robot.vision.Limelight;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

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

	private final GyroIO gyro;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

	public final SwerveModule[] modules = new SwerveModule[4]; // FL, FR, BL, BR
	public final Field2d field = new Field2d();
	private final SwerveDriveKinematics kinematics = Constants.Drivetrain.kinematics;
	public final SwerveDrivePoseEstimator est;
	public final Limelight limelightForward = new Limelight("limelight-forward");
	public final Limelight limelightSide = new Limelight("limelight-side");
	public final Limelight limelightReverse = new Limelight("limelight-reverse");
	public final Limelight[] limelights = {limelightForward, limelightSide, limelightReverse};

	private final JoystickDrive joystickDrive = new JoystickDrive(this, 1d);
	private Rotation2d joystickFOROffset;

	public AutoFactory autoFactory;
	// Choreo PID controllers have to be created in our code
	private final PIDController xController = new PIDController(5, 0.0, 0);
    private final PIDController yController = new PIDController(5, 0.0, 0);
    private final PIDController headingController = new PIDController(5, 0.0, 0.2);

	public Drivetrain() {
		this.gyro = switch(Constants.mode) {
		case REAL -> new GyroIOReal();
		case REPLAY -> new GyroIO() {};
		case SIM -> new GyroIOReal();
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

		this.joystickFOROffset = new Rotation2d(this.gyroInputs.yawPosition);

		autoFactory = new AutoFactory(
            this.est::getEstimatedPosition, // A function that returns the current robot pose
            this::reset, // A function that resets the current robot pose to the provided Pose2d
            this::controlSwerveSample, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            this // The drive subsystem
        );

		headingController.enableContinuousInput(-Math.PI, Math.PI);
	}

	public void control(ChassisSpeeds speeds) {
		// NOTE: The speeds provided must be robot-relative, else the robot will go the wrong way
		Logger.recordOutput("Drivetrain/DemandedChassisSpeedsROD", speeds);

		// Discretize is used to compensate for swerve mechanics.
		// When the robot is translating while rotating, the motion is actually along an arc, but ChassisSpeeds is representing linear movement.
		// So, it figures out what arc movement gets the robot to the correct spot after 1 program loop based on the provided ChassisSpeeds.
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		this.control(this.kinematics.toSwerveModuleStates(speeds));
	}

	public void controlSwerveSample(final SwerveSample sample) {
		// Get the current pose of the robot
        Pose2d pose = getEstimatedPosition();

		Logger.recordOutput("Drivetrain/Auto/SwerveSample", sample);
        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Convert the speeds to robot-oriented and control
		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getEstimatedPosition().getRotation());
		control(speeds);
	}

	public void control(final Drivetrain.State state) { this.control(state.states); }

	public void control(final SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.maxVelocity);

		for(int i = 0; i < this.modules.length; i++) {
			if (this.modules[i] != null) {
				this.modules[i].control(states[i]);
			}
		}
	}

	public void halt() { this.control(State.locked()); }

	public Command haltCommand() {
		return new RunCommand(() -> halt(), this).withTimeout(0.1);
	}

	public void resetAngle() {
		this.joystickFOROffset = new Rotation2d(this.gyroInputs.yawPosition);
	}

	public Angle getFieldOrientedAngle() {
		return this.gyroInputs.yawPosition.minus(this.joystickFOROffset.getMeasure());
	}

	public void reset(final Pose2d newPose) {
		this.est.resetPosition(new Rotation2d(this.gyroInputs.yawPosition), this.modulePositions(), newPose);
		for(Limelight lime : limelights){
			lime.setIMUMode(1);
			lime.setRobotOrientation(newPose.getRotation().getMeasure());
		}
	}

	public void setAngle(final Angle angle){
		this.reset(new Pose2d(this.est.getEstimatedPosition().getTranslation(), new Rotation2d(angle)));
	}

	@AutoLogOutput
	public void runCharacterization(final double volts) {
		for(int i = 0; i < this.modules.length; i++) {
			this.modules[i].runCharacterization(volts);
		}
		Logger.recordOutput("Drivetrain/InputVoltage", volts);
	}

	@AutoLogOutput(key = "Drivetrain/CurrentPositions")
	public SwerveModulePosition[] modulePositions() {
		return Arrays.stream(this.modules).map(module -> (module != null) ? module.position : new SwerveModulePosition()).toArray(SwerveModulePosition[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/States/Desired")
	public SwerveModuleState[] desiredModuleStates() {
		return Arrays.stream(this.modules).map(module -> (module != null) ? module.desired : new SwerveModuleState()).toArray(SwerveModuleState[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/States/Current")
	public SwerveModuleState[] currentModuleStates() {
		return Arrays.stream(this.modules).map(module -> (module != null) ? module.current : new SwerveModuleState()).toArray(SwerveModuleState[]::new);
	}

	@AutoLogOutput(key = "Drivetrain/CurrentChassisSpeeds")
	public ChassisSpeeds getCurrentChassisSpeeds() {
		return this.kinematics.toChassisSpeeds(this.currentModuleStates());
	}

	public Pose2d getEstimatedPosition(){
		return this.est.getEstimatedPosition();
	}
	public AlgaePosition getAlgaeHeight(){
		double smallst = Double.MAX_VALUE;
      	int tagPose = 17;
		for(int tag:CenterLimelight.reefTags) {
			Pose2d distance = Constants.FIELD_LAYOUT.getTagPose(tag).get().toPose2d().relativeTo(RobotContainer.getInstance().drivetrain.getEstimatedPosition());
			if(Math.hypot(distance.getX(), distance.getY()) < smallst){
				tagPose = tag;
				smallst = Math.hypot(distance.getX(), distance.getY());
			}
		}
		if(Constants.algaeL2.contains(tagPose)){
			return AlgaePosition.L2;
		}
		return AlgaePosition.L3;

	}

	public boolean isUpdateable(PoseEstimate posEst) {
		if (Math.abs(this.gyroInputs.yawVelocityRadPerSec.in(Units.DegreesPerSecond)) > 720) { 
			return false;
		}
		if (posEst.tagCount == 0) {
			return false;
		}
		if (Double.isNaN(posEst.pose.getX()) || Double.isNaN(posEst.pose.getY()) || Double.isInfinite(posEst.pose.getX()) || Double.isInfinite(posEst.pose.getY())) {
			return false;
		}
		if (Double.isNaN(posEst.pose.getRotation().getDegrees())|| Double.isInfinite(posEst.pose.getRotation().getDegrees())) {
			return false;
		}
		if (posEst.pose.getX() > 100d || posEst.pose.getY() > 100d) {
			return false;
		}
		
		return true;
	}

	@Override
	public void periodic() {
		this.gyro.updateInputs(this.gyroInputs);
		Logger.processInputs("Drivetrain/Gyro", this.gyroInputs);

		for(final SwerveModule module : this.modules) {
			if (module != null) {
				module.periodic();
			}
		}

		// Update the odometry pose
		Angle yawPosition = this.gyroInputs.yawPosition;
		if (!this.gyroInputs.connected) {
			yawPosition = Units.Degrees.of(limelightSide.getIMUData().robotYaw);
			Logger.recordOutput("Drivetrain/UsingLLYaw", true);
		} else {
			Logger.recordOutput("Drivetrain/UsingLLYaw", false);
		}

		this.est.update(new Rotation2d(yawPosition), this.modulePositions());

		// Add vision measurements to pos est with megatag 2
		for(Limelight limelight : limelights){
			PoseEstimate mt2 = limelight.getPoseMegatag2();
			if (mt2 != null) {
				Logger.recordOutput("Drivetrain/poseMegatag"+limelight.getLimelightName(), mt2.pose);

				// if our angular velocity is greater than 720 degrees per second, ignore vision updates or if it doesnt see any tags		
				Logger.recordOutput("Drivetrain/doRejectUpdate", doRejectUpdate);
				if(isUpdateable(mt2)) {
					est.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,9999999));
					est.addVisionMeasurement(
						mt2.pose,
						mt2.timestampSeconds);
				}
			}
		}
		field.setRobotPose(this.est.getEstimatedPosition());
		Logger.recordOutput("Drivetrain/Pose", this.est.getEstimatedPosition());
		Logger.recordOutput("Drivetrain/Imumode", limelightSide.getImuMode());
		Logger.recordOutput("Drivetrain/limelightHasTargets",limelightSide.hasValidTargets());
		Logger.recordOutput("Drivetrain/matchTimer", Timer.getMatchTime());
		PoseEstimate mt1 = this.limelightSide.getPoseMegatag1();
		if (mt1 != null) {
			Logger.recordOutput("Drivetrain/Mt1", mt1.pose);
		}
	}

	public void disabledPeriodic() {
		//TODO: maybe add some more filtering
		Angle rotaion = Units.Radians.of(0);
		int numberOfValidTargets = 0;
		PoseEstimate mostTrusted = null;
		int highNumAprilTags = 0;
		for(Limelight lime : limelights){
			PoseEstimate mt1 = lime.getPoseMegatag1();
			if(lime.hasValidTargets() && mt1 != null && lime.getNumberOfAprilTags() > highNumAprilTags){
				mostTrusted = mt1;
				highNumAprilTags = lime.getNumberOfAprilTags();
			}
		}
		if(mostTrusted!=null){
			setAngle(mostTrusted.pose.getRotation().getMeasure());
		}
	}

	public void seedLimelightImu(){
		disabledPeriodic();
	}

	public void setImuMode2(){
		for(Limelight lime : limelights){
			lime.setIMUMode(2);
		}
	}

	public void setDefaultCommand() {
		this.setDefaultCommand(this.joystickDrive);
	}

	@Override
	public void simulationPeriodic() {
		for (SwerveModule s: this.modules) {
			s.simulationPeriodic();
		}
		final ChassisSpeeds simulatedTwist = this.kinematics.toChassisSpeeds(this.currentModuleStates());
		gyro.simulationPeriodic(Units.Radians.of(simulatedTwist.omegaRadiansPerSecond * 0.02));
	}

	public JoystickDrive slowMode() {
		return new JoystickDrive(this, .15); // Conversion from 1 meter to 6 inches
	}
}
