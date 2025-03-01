package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
public class Constants {
	private static Mode currentMode() {
		if(Robot.isReal()) return Mode.REAL;
		if(!Logger.hasReplaySource()) return Mode.SIM;
		else return Mode.REPLAY;
	}

	private Constants() { throw new IllegalCallerException("Cannot instantiate `Constants`"); }

	public static final Mode mode = Constants.currentMode();
	public static final boolean real = Constants.mode == Constants.Mode.REAL;

	public static final AudioConfigs talonFXAudio = new AudioConfigs()
		.withAllowMusicDurDisable(true)
		.withBeepOnBoot(true)
		.withBeepOnConfig(true);

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public static enum ReefPosition {
		A(-1, List.of(18, 7)),
		B(1, List.of(18, 7)),
		C(-1, List.of(17, 8)),
		D(1, List.of(17, 8)),
		E(-1, List.of(22, 9)),
		F(1, List.of(22, 9)),
		G(-1, List.of(21, 10)),
		H(1, List.of(21, 10)),
		I(-1, List.of(20, 11)),
		J(1, List.of(20, 11)),
		K(-1, List.of(19, 6)),
		L(1, List.of(19, 6));

		private int direction;
		private List<Integer> tags;

		private ReefPosition(int direction, List<Integer> tags) {
			this.direction = direction;
			this.tags = tags;
		}

		public int getDirection() {
			return direction;
		}

		public List<Integer> getTagID() {
			return tags;
		}
	}

	public static enum GamePieceType {
		NONE(0),   // Game Piece Type for None
		ALGAE(1),  // Game Piece Type for Algae
		CORAL(2);  // Game Piece Type for Coral
		// other pieces here...

		private final int value;

		GamePieceType(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}

		public static GamePieceType fromInt(int value) {
			for (GamePieceType gamePieceType : GamePieceType.values()) {
				if (gamePieceType.getValue() == value) {
					return gamePieceType;
				}
			}
			return GamePieceType.NONE;
		}
	}

	public static enum CoralPosition {
		NONE(0),  // Coral Position for None
		L1(1),    // Coral Position for L1
		L2(2),    // Coral Position for L2
		L3(3),    // Coral Position for L3
		L4(4);    // Coral Position for L4
		// other positions here...

		private final int value;

		CoralPosition(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}

		public static CoralPosition fromInt(int value) {
			for (CoralPosition coralPosition : CoralPosition.values()) {
				if (coralPosition.getValue() == value) {
					return coralPosition;
				}
			}
			return CoralPosition.NONE;
		}
	}

	public static enum AlgaePosition {
		NONE(0),  // Algae Position for None
		L1(1),    // Algae Position for L1 (unused)
		L2(2),    // Algae Position for L2
		L3(3),    // Algae Position for L3
		L4(4);    // Algae Position for L4 (unused)
		// other positions here...

		private final int value;

		AlgaePosition(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}

		public static AlgaePosition fromInt(int value) {
			for (AlgaePosition algaePosition : AlgaePosition.values()) {
				if (algaePosition.getValue() == value) {
					return algaePosition;
				}
			}
			return AlgaePosition.NONE;
		}
	}

	public static enum SubsystemKey {
		NONE(0), 	    // Subsystem for None
		ELEVATOR(1),    // Subsystem for Elevator
		BANANA(2),      // Subsystem for Banana
		DRIVETRAIN(3);  // Subsystem for Drivetrain
		// other subsystems here...

		private final int value;

		SubsystemKey(int value) {
			this.value = value;
		}

		public int getValue() {
			return this.value;
		}

		public static SubsystemKey fromInt(int value) {
			for (SubsystemKey subsystemKey : SubsystemKey.values()) {
				if (subsystemKey.getValue() == value) {
					return subsystemKey;
				}
			}
			return SubsystemKey.NONE;
		}
	}

	public static final double mod(final double lhs, final double rhs) { return (lhs % rhs + rhs) % rhs; }

	public static final double angleNorm(final double angle) { return Constants.mod(angle + 180, 360) - 180; }

	public static final double angleDistance(final double a, final double b) {
		return Math
			.abs(Constants.angleNorm(180 - Math.abs(Math.abs(Constants.angleNorm(a) - Constants.angleNorm(b)) - 180)));
	}

	public static record PIDValues(double p, double i, double d, double f) {
		public final PIDController createController() { return new PIDController(this.p, this.i, this.d); }

		public final ProfiledPIDController createProfiledController(final TrapezoidProfile.Constraints profile) {
			return new ProfiledPIDController(this.p, this.i, this.d, profile);
		}
	}

	public static PIDConstants fromPIDValues(final PIDValues pid) { return new PIDConstants(pid.p, pid.d, pid.d); }

	public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

	public static final class LimelightFX {
		private LimelightFX() { throw new IllegalCallerException("Cannot instantiate `Constants.LimelightFX`"); }

		public static final boolean enabled = false;
	}

	public static final class CAN {
		private CAN() { throw new IllegalCallerException("Cannot instantiate `Constants.CAN`"); }

		public static final class Misc {
			public static final int pdh = 0;

			public static final int feederLauncher = 0;
			public static final int intakeRoller = 3;
		}

		public static final class CTRE {
			public static final String bus = "canivore";

			public static final int pigeon = 0;

			public static final int swerveFrontLeftAzimuth = 13;
			public static final int swerveFrontLeftDrive = 14;

			public static final int swerveFrontRightAzimuth = 19;
			public static final int swerveFrontRightDrive = 18;

			public static final int swerveBackLeftAzimuth = 10;
			public static final int swerveBackLeftDrive = 11;

			public static final int swerveBackRightAzimuth = 15;
			public static final int swerveBackRightDrive = 16;

			public static final int bananaPivot = 12;
			public static final int elevatorMotorA = 1;
			public static final int elevatorMotorB = 4;
			public static final int pivotLimitSwitch = 8;
			public static final int elevatorLimitSwitch = 9;

			public static final int bananaWheels = 20;
			public static final int intakeWheels = 99; //TODO: get right number
			public static final int intakeBelt = 98;
			public static final int intakePivot = 97;
			public static final int troughWheels = 96;

			public static final int climber = 17;
		}
	}

	public static final class PWM {
		private PWM() { throw new IllegalCallerException("Cannot instantiate `Constants.PWM`"); }

		public static final int climberRatchet = 9;

		public static final int ampBarServoA = 0;
		public static final int ampBarServoB = 1;
	}

	public static final class DIO {
		private DIO() { throw new IllegalCallerException("Cannot instantiate `Constants.DIO`"); }

		public static final int release = 0;
		public static final int lockout = 1;
	}

	public static final class Drivetrain {
		private Drivetrain() { throw new IllegalCallerException("Cannot instantiate `Constants.Drivetrain`"); }

		// Gear ratios for SDS MK4i L2, adjust as necessary
		public static final double driveGearRatio = (50.0 / 14) * (16.0 / 28) * (45.0 / 15); // ~= 6.746
		public static final double azimuthGearRatio = 150.0 / 7.0;

		public static final Distance wheelRadius = Units.Inches.of(1.85);
		public static final Distance wheelCircumference = Drivetrain.wheelRadius.times(2 * Math.PI);

		public static final LinearVelocity maxVelocity = Units.FeetPerSecond.of(15.5);  // MK4i max speed L2

		public static final Distance wheelBase = Units.Inches.of(27 - 2.5 * 2);
		public static final Distance trackWidth = Drivetrain.wheelBase; // For a square drivetrain
		public static final Distance halfRobotWidth = Units.Inches.of(27.0/2);
		public static final Distance halfRobotWidthBumpersOn = Units.Inches.of(27.0/2 + 3);

		// max angular velocity computes to 6.41 radians per second
		public static final AngularVelocity maxAngularVelocity = Units.RotationsPerSecond
			.of(
				Drivetrain.maxVelocity.in(Units.MetersPerSecond)
					/ (2
						* Math.PI
						* Math
							.hypot(
								Drivetrain.trackWidth.div(2).in(Units.Meters),
								Drivetrain.wheelBase.div(2).in(Units.Meters)
							))
			);

		public static final class Auto {
			public static final PIDValues translationDynamic = new PIDValues(10, 0, 0, 0);
			public static final PIDValues thetaDynamic = new PIDValues(10, 0, 0, 0);
			public static final PIDValues centerLimelight = new PIDValues(2, 0, 0, 0);
			public static final PIDValues centerTheta = new PIDValues(4, 0, 0.2, 0);
		}

		public static final SlotConfigs azimuth = new SlotConfigs()
			.withKP(-50)
			.withKI(0)
			.withKD(-0.5)
			.withKS(-2);

		public static final SlotConfigs drive = new SlotConfigs()
			.withKP(0)
			.withKI(0)
			.withKD(0)
			.withKS(0)
			.withKV(12.0/maxVelocity.in(Units.MetersPerSecond))
			.withKA(0);

		public static final PIDValues absoluteRotationPID = new PIDValues(2.3, 0, 0.15, 0);
		public static final TrapezoidProfile.Constraints absoluteRotationConstraints = new TrapezoidProfile.Constraints(
			1,
			17
		);

		public static final Angle swerveFrontLeftOffset = Units.Rotations.of(-0.420654296875);
		public static final Translation2d swerveFrontLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase,
			Constants.Drivetrain.trackWidth
		);
		public static final Angle swerveFrontRightOffset = Units.Rotations.of(0.299072265625);
		public static final Translation2d swerveFrontRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase,
			Constants.Drivetrain.trackWidth.unaryMinus()
		);
		public static final Angle swerveBackLeftOffset = Units.Rotations.of(0.033203125);
		public static final Translation2d swerveBackLeftTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase.unaryMinus(),
			Constants.Drivetrain.trackWidth
		);
		public static final Angle swerveBackRightOffset = Units.Rotations.of(-0.4169921875);
		public static final Translation2d swerveBackRightTranslation = new Translation2d(
			Constants.Drivetrain.wheelBase.unaryMinus(),
			Constants.Drivetrain.trackWidth.unaryMinus()
		);

		public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			Constants.Drivetrain.swerveFrontLeftTranslation,
			Constants.Drivetrain.swerveFrontRightTranslation,
			Constants.Drivetrain.swerveBackLeftTranslation,
			Constants.Drivetrain.swerveBackRightTranslation
		);
	}

	public static class Elevator {
		public static final class FlywheelConfiguration {
			public static final FlywheelConfiguration yellowFairlane = new FlywheelConfiguration(
				Units.RotationsPerSecond.of(30),
				Units.RotationsPerSecond.of(27),
				0.3
			);
			public static final FlywheelConfiguration greenBane = new FlywheelConfiguration(
				Units.RotationsPerSecond.of(40),
				Units.RotationsPerSecond.of(37),
				0.60
			);

			public FlywheelConfiguration(
				final AngularVelocity flywheelVelocity,
				final AngularVelocity flywheelVelocityThreshold,
				final double ampPower
			) {
				this.speakerVelocity = flywheelVelocity;
				this.speakerVelocityThreshold = flywheelVelocityThreshold;
				this.ampPower = ampPower;
			}

			public final AngularVelocity speakerVelocity;
			public final AngularVelocity speakerVelocityThreshold;

			public final double ampPower;
		}

		private Elevator() { throw new IllegalCallerException("Cannot instantiate `Constants.Elevator`"); }

		public static final double ELEVATOR_GEARING = 29.76190476d;
		public static final int NUMBER_OF_STAGES = 3;
		public static final Distance DRUM_RADIUS = Units.Inches.of(1.128);
		public static final double DISTANCE_CONVERSION_RATIO = ELEVATOR_GEARING / NUMBER_OF_STAGES / (2 * Math.PI * DRUM_RADIUS.in(Units.Meters));

		public static final Distance MAX_ELEVATOR_DISTANCE = Units.Inches.of(90);
		public static final Distance MIN_ELEVATOR_DISTANCE = Units.Inches.of(0);

		public static final Angle MAX_PIVOT_ANGLE = Units.Degrees.of(40);
		public static final Angle MIN_PIVOT_ANGLE = Units.Degrees.of(0);

		public static final SlotConfigs elevatorConfig = new SlotConfigs()
		.withGravityType(GravityTypeValue.Elevator_Static)
		.withKS(0)
		//.withKG(0.1)
		.withKV(6.81655937847)
		.withKA(0.53)
		.withKP(20)
		.withKD(0);

		public static final double pivotCurrentLimit = 40;
		public static final AngularVelocity pivotMaxVelocityShoot = Units.DegreesPerSecond.of(2);
		public static final Slot0Configs pivotConfig = new Slot0Configs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withKP(0.05)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0)
			.withKV(0.015)
			.withKA(0);
	}

	public static class Intake {
		// The minium value for the intake
		public static final Angle pivotMin = Units.Rotations.of(-0.1085);
		// max angle before exiting allowed extension range
		public static final Angle max = Units.Rotations.of(0.39);

		public static final Slot0Configs pivotConfig = new Slot0Configs()
			.withGravityType(GravityTypeValue.Arm_Cosine)
			.withKP(0.05)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0)
			.withKV(0.015)
			.withKA(0);

			public static final Slot0Configs flywheelGainsSlot0 = new Slot0Configs()
			.withKP(0.05)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0)
			.withKV(0.015)
			.withKA(0);
}
	
	public static class Banana {
		private Banana() { throw new IllegalCallerException("Cannot instantiate `Constants.Banana`"); }

		public static final Slot0Configs flywheelGainsSlot0 = new Slot0Configs()
			.withKS(0)
			.withKV(0.015)
			.withKA(0);
	}

	public static class Climber {
		private Climber() { throw new IllegalCallerException("Cannot instantiate `Constants.Climber`"); }

		public static final boolean ratchetEnabled = false;

		public static final SlotConfigs configFast = new SlotConfigs().withKP(0.25);
		public static final SlotConfigs configSlow = new SlotConfigs().withKP(0.01);

		public static final double ratchetLocked = 84;
		public static final double ratchetFree = 93;

		public static final double max = 129;
		public static final double disengageDistance = 0.5;
		public static final double initializeRaiseDistance = 2;
	}
}
