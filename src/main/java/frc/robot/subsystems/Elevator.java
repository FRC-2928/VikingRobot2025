package frc.robot.subsystems;

import java.util.Map;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Tuning;
import frc.robot.Constants.AlgaePosition;
import frc.robot.Constants.CagePosition;
import frc.robot.Constants.CoralPosition;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.SubsystemKey;

public class Elevator extends SubsystemBase {
	@AutoLog
	public static class ElevatorInputs {
		public Distance height;
		public boolean isPivotHomed;
		public boolean isElevatorHomed;
		public LinearVelocity speed;
		public Angle pivotAngle;
		public AngularVelocity pivotAngularVelocity;
	}

	public final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

	// --------------------- Internal Higher-Order States ---------------------
	// Target state variables
	private int targetCoralLevel = CoralPosition.L1.getValue();    // L1 by default
	private int targetAlgaeLevel = AlgaePosition.NONE.getValue();  // NONE by default
	private int targetCageLevel  = CagePosition.NONE.getValue();   // NONE by default
	private GamePieceType currentGamePieceType  = GamePieceType.NONE;  // NONE by default
	private GamePieceType previousGamePieceType = GamePieceType.NONE;  // NONE by default

	// ------------------- Elevator Position Maps -------------------
	// Map of Elevator Positions for Coral
	private final Map<Integer, Distance> elevatorPositionsCoral = Map.of(
		CoralPosition.NONE.getValue(), Units.Feet.of(0),
		CoralPosition.L1.getValue(),   Units.Feet.of(2.5),
		CoralPosition.L2.getValue(),   Units.Feet.of(3.5),
		CoralPosition.L3.getValue(),   Units.Feet.of(4.5),
		CoralPosition.L4.getValue(),   Units.Feet.of(6.4));

	// Map of Elevator Positions for Algae
	private final Map<Integer, Distance> elevatorPositionsAlgae = Map.of(
		AlgaePosition.NONE.getValue(), Units.Feet.of(0),
		AlgaePosition.L2.getValue(),   Units.Feet.of(3.5),
		AlgaePosition.L3.getValue(),   Units.Feet.of(4.5));

	// Map of Elevator Positions for Cage
	private final Map<Integer, Distance> elevatorPositionsCage = Map.of(
		CagePosition.NONE.getValue(),    Units.Feet.of(0),
		CagePosition.DEEP.getValue(),    Units.Feet.of(3.5),
		CagePosition.SHALLOW.getValue(), Units.Feet.of(4.5));

	// ------------------- Banana Angle Maps -------------------
	// Map of Banana Angles for Coral
	private final Map<Integer, Angle> bananaAnglesCoral = Map.of(
		CoralPosition.NONE.getValue(), Units.Degrees.of(0),
		CoralPosition.L1.getValue(),   Units.Degrees.of(0),
		CoralPosition.L2.getValue(),   Units.Degrees.of(0),
		CoralPosition.L3.getValue(),   Units.Degrees.of(0),
		CoralPosition.L4.getValue(),   Units.Degrees.of(45));

	// Map of Banana Angles for Algae
	private final Map<Integer, Angle> bananaAnglesAlgae = Map.of(
		AlgaePosition.NONE.getValue(), Units.Degrees.of(0),
		AlgaePosition.L2.getValue(),   Units.Degrees.of(0),
		AlgaePosition.L3.getValue(),   Units.Degrees.of(0));

	// Map of Banana Angles for Cage
	private final Map<Integer, Angle> bananaAnglesCage = Map.of(
		CagePosition.NONE.getValue(),    Units.Degrees.of(0),
		CagePosition.DEEP.getValue(),    Units.Degrees.of(0),
		CagePosition.SHALLOW.getValue(), Units.Degrees.of(0));

	// --------------------- Hardware Interfaces ---------------------
	private final TalonFX liftMotorA;
	private final TalonFX liftMotorB;
	private final TalonFX pivot;

	private final StatusSignal<Angle> elevatorMotorPosition;
	private final StatusSignal<AngularVelocity> elevatorMotorVelocity;
	private final StatusSignal<AngularVelocity> pivotMotorVelocity;
	private final StatusSignal<Angle> pivotMotorPosition;

	private Distance elevatorTargetPosition; // Represents final pos system is trying to reach
	private Angle pivotTargetAngle; // Represents final angle system is trying to reach

	private Distance elevatorCommandedPosition; // The pos that the motor is currently told to go to
	private Angle pivotCommmandedAngle; // The angle that the motor is currently told to go to

	private final Distance elevatorThresholdForPivot = Units.Inches.of(8); // The minimum distance that the elevator is allowed to be with a non-zero pivot angle
	private final Distance toleranceForFinishedMovement = Units.Millimeters.of(7);
	private final Angle toleranceForFinishedPivot = Units.Degrees.of(2);

	// Simulation objects
	private final ElevatorSim elevatorSim = new ElevatorSim(
		LinearSystemId.createElevatorSystem(
			DCMotor.getKrakenX60Foc(2), Units.Pounds.of(36).in(Units.Kilogram), 
			Constants.Elevator.DRUM_RADIUS.in(Units.Meters), 
			Constants.Elevator.ELEVATOR_GEARING), 
		DCMotor.getKrakenX60Foc(2), 
		0, 
		Units.Inches.of(30).in(Units.Meters),
		false, 
		0);

	/**
	 * Default Constructor
	 */
	public Elevator() {
		this.elevatorTargetPosition = Units.Feet.of(0);
		this.elevatorCommandedPosition = Units.Feet.of(0);
		this.pivotTargetAngle = Units.Degrees.of(0);
		this.pivotCommmandedAngle = Units.Degrees.of(0);

		liftMotorA = new TalonFX(Constants.CAN.CTRE.elevatorMotorA);
		liftMotorB = new TalonFX(Constants.CAN.CTRE.elevatorMotorB);

		pivot = new TalonFX(Constants.CAN.CTRE.bananaPivot);

		this.elevatorMotorPosition = this.liftMotorA.getPosition();
		this.elevatorMotorVelocity = this.liftMotorA.getVelocity();

		this.pivotMotorPosition = this.pivot.getRotorPosition();
		this.pivotMotorVelocity = this.pivot.getRotorVelocity();

		liftMotorB.setControl(new Follower(liftMotorA.getDeviceID(), false));

		final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

		elevatorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.CAN.CTRE.elevatorLimitSwitch;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
		elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Gearing, clockwise moves elevator up
		elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		
		// Peak output amps
		elevatorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
		
		// Supply current limites
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		// PID values
		elevatorConfig.Slot0 = Slot0Configs.from(Constants.Elevator.elevatorConfig);
		elevatorConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.DISTANCE_CONVERSION_RATIO;

		// Motion Magic Params
		// elevatorConfig.MotionMagic.MotionMagicAcceleration = 10;
		// elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 3.833 * Constants.Elevator.DISTANCE_CONVERSION_RATIO;
		elevatorConfig.MotionMagic.MotionMagicExpo_kV = 6.81655937847;
		elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.55;

		this.liftMotorA.getConfigurator().apply(elevatorConfig);


		final TalonFXConfiguration bananaConfig = new TalonFXConfiguration();

		bananaConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
		bananaConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
		bananaConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
		bananaConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.CAN.CTRE.pivotLimitSwitch;
		bananaConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
		bananaConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
		elevatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

		// Peak output amps
		bananaConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		bananaConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		bananaConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		bananaConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		// Supply current limits
		bananaConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		bananaConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		bananaConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		bananaConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		// PID values
		bananaConfig.Slot0 = Constants.Elevator.pivotConfig;

		this.pivot.getConfigurator().apply(bananaConfig);

		StatusSignal.setUpdateFrequencyForAll(50, elevatorMotorPosition, elevatorMotorVelocity, pivotMotorPosition, pivotMotorVelocity);

		this.setDefaultCommand(toDefaultPosition());
		this.targetCageLevel = CagePosition.SHALLOW.getValue();  // TODO: change this to accept values from SmartDashboard
	}

	private void moveToPosition(final Distance position) {
		this.elevatorTargetPosition = Units.Meters.of(
			MathUtil.clamp(
				position.in(Units.Meters),
				Constants.Elevator.MIN_ELEVATOR_DISTANCE.in(Units.Meters),
				Constants.Elevator.MAX_ELEVATOR_DISTANCE.in(Units.Meter)));
	}

	private void pivotBanana(final Angle rotation) {
		this.pivotTargetAngle = Units.Degrees.of(
			MathUtil.clamp(
				rotation.in(Units.Degrees),
				Constants.Elevator.MIN_PIVOT_ANGLE.in(Units.Degrees),
				Constants.Elevator.MAX_PIVOT_ANGLE.in(Units.Degrees)));
	}
	
	private void controlPosition(final Distance position) {
		Logger.recordOutput("Elevator/DesiredPosition", position.in(Units.Meters));
		liftMotorA.setControl(new MotionMagicExpoVoltage(position.in(Units.Meters)));
		elevatorCommandedPosition = position;
	}

	private void controlPivot(final Angle rotation) {
		pivot.setControl(new PositionVoltage(rotation));
		pivotCommmandedAngle = rotation;
	}

	private boolean isInTargetPos() {
		if (elevatorTargetPosition.gt(elevatorThresholdForPivot)) {
			return elevatorTargetPosition.isNear(inputs.height, toleranceForFinishedMovement) &&
				   pivotTargetAngle.isNear(inputs.pivotAngle, toleranceForFinishedPivot);
		}
		return elevatorTargetPosition.isNear(inputs.height, toleranceForFinishedMovement) &&
		       Units.Degrees.of(0).isNear(inputs.pivotAngle, toleranceForFinishedPivot);
	}

	private void updateMotors() {
		// are we currently in the elevator danger zone?
		// is the elevator target within the danger zone?
		// is the banana in a dangerous orientation?
		// are we trying to move the banana into a dangerous orientation?
		if (elevatorCommandedPosition != elevatorTargetPosition) {
			if (elevatorTargetPosition.gte(elevatorThresholdForPivot)) {
				controlPosition(elevatorTargetPosition);
			}
			else {
				if (this.inputs.isPivotHomed) {
					controlPosition(elevatorTargetPosition);
				}
			}
		}

		if (pivotCommmandedAngle != pivotTargetAngle) {
			if (pivotTargetAngle.lt(Units.Degrees.of(1))) {
				controlPivot(pivotTargetAngle);
			}
			else {
				if (inputs.height.gt(elevatorThresholdForPivot) && elevatorCommandedPosition.gt(elevatorThresholdForPivot)) {
					controlPivot(pivotTargetAngle);
				}
			}
		}

		if (elevatorTargetPosition.lt(elevatorThresholdForPivot) && pivotCommmandedAngle.gt(Units.Degrees.of(1))) {
			controlPivot(Units.Degrees.of(0));
		}
	}

	private void updateInputs(final ElevatorInputs inputs) {
		BaseStatusSignal
			.refreshAll(
				this.elevatorMotorPosition,
				this.elevatorMotorVelocity,
				this.pivotMotorPosition,
				this.pivotMotorVelocity
			);

		inputs.height = Units.Meters.of(elevatorMotorPosition.getValue().in(Units.Rotations));
		inputs.speed = Units.MetersPerSecond.of(elevatorMotorVelocity.getValue().in(Units.RotationsPerSecond));
		inputs.isElevatorHomed = inputs.height.in(Units.Meters) < 1e-7;
		
		inputs.pivotAngle = pivotMotorPosition.getValue();
		inputs.pivotAngularVelocity = pivotMotorVelocity.getValue();
		inputs.isPivotHomed = inputs.pivotAngle.in(Units.Degrees) < 1;
	}

	@Override
	public void periodic() {
		this.updateInputs(this.inputs);
		this.updateMotors();
		Logger.processInputs("Elevator", this.inputs);
	}

	@Override
	public void simulationPeriodic() {
		TalonFXSimState liftMotorASimState = liftMotorA.getSimState();
		TalonFXSimState liftMotorBSimState = liftMotorB.getSimState();

		elevatorSim.setInputVoltage(liftMotorASimState.getMotorVoltage());
		elevatorSim.update(0.02);

		liftMotorASimState.setRawRotorPosition(elevatorSim.getPositionMeters() * Constants.Elevator.DISTANCE_CONVERSION_RATIO * Constants.Elevator.NUMBER_OF_STAGES);
		liftMotorASimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * Constants.Elevator.DISTANCE_CONVERSION_RATIO * Constants.Elevator.NUMBER_OF_STAGES);
		liftMotorBSimState.setRawRotorPosition(elevatorSim.getPositionMeters() * Constants.Elevator.DISTANCE_CONVERSION_RATIO * Constants.Elevator.NUMBER_OF_STAGES);
		liftMotorBSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * Constants.Elevator.DISTANCE_CONVERSION_RATIO * Constants.Elevator.NUMBER_OF_STAGES);

		Logger.recordOutput("Elevator/Sim/LiftVoltage", liftMotorASimState.getMotorVoltage());
		// Logger.recordOutput("Elevator/Sim/Height", elevatorSim.getPositionMeters());
	}

	public void setDefaultCommand() {
		this.setDefaultCommand(this.toDefaultPosition());
	}

	public void setTargetAlgaeLevel(AlgaePosition targetAlgaeLevel) {
		this.targetAlgaeLevel = targetAlgaeLevel.getValue();
	}

	public void setTargetCoralLevel(CoralPosition targetCoralLevel) {
		this.targetCoralLevel = targetCoralLevel.getValue();
	}

	public void setTargetCageLevel(CagePosition targetCageLevel) {
		this.targetCageLevel = targetCageLevel.getValue();
	}

	public void toggleReefHeightDown() {
		this.targetCoralLevel = MathUtil.clamp(this.targetCoralLevel-1, 1, 4);
	}

	public void toggleReefHeightUp() {
		this.targetCoralLevel = MathUtil.clamp(this.targetCoralLevel+1, 1, 4);
	}

	public void onEjectAlgae() {
		// any algae specific logic here
		onEjectGamePieceGeneric();
	}

	public void onEjectCoral() {
		// any coral specific logic here
		onEjectGamePieceGeneric();
	}

	private void onEjectGamePieceGeneric() {
		// reset the home to whatever it was before...
		this.currentGamePieceType = this.previousGamePieceType;
	}

	/**
	 * Function factory to obtain an executable command to go to a reef height
	 * based off of a provided game piece type (e.g., Coral, Algae). The returned
	 * @c Command will terminate once the elevator reaches the desired target position.
	 *
	 * @param pieceType a @c GamePieceType indicating the desired game piece
	 * @return an executable @c Command that moves the elevator to the appropriate position
	 */
	public Command goToReefHeight(GamePieceType pieceType) {
		return goToGamePieceHeightEndless(pieceType).until(this::isInTargetPos);
	}

	/**
	 * Function factory to obtain an executable command to go to a height
	 * based off of a provided game piece type (e.g., Coral, Algae)
	 *
	 * @param pieceType a @c GamePieceType indicating the desired game piece
	 * @return an executable @c Command that moves the elevator to the appropriate position
	 */
	public Command goToGamePieceHeight(GamePieceType pieceType) {
		return goToGamePieceHeightEndless(pieceType);
	}

	private Command goToGamePieceHeightEndless(GamePieceType pieceType) {
		return new FunctionalCommand(
			/* initialize() */
			() -> {
				// we're after a new piece if and only if "new" is different from "current"; otherwise there's no delta
				if (pieceType.getValue() != this.currentGamePieceType.getValue()) {
					// we've got a new target -- save off the old piece
					this.previousGamePieceType = this.currentGamePieceType;
				}
			},
			/* execute() */
			() -> {
				// update the current target game piece so that the home position is also updated accordingly
				this.currentGamePieceType = pieceType;
				// select the correct set of maps for the given game piece and figure out which key
				// we're using to determine the elevator position/banana angle
				Map<Integer, Distance> elevatorPositionsMap;
				Map<Integer, Angle> bananaAnglesMap;
				int positionKey;

				// TODO: put these into a map and fetch from there instead of these switch-case blocks
				switch (pieceType) {
					case NONE:  // fall-through
					case CORAL: // fall-through
					default: {
						elevatorPositionsMap = this.elevatorPositionsCoral;
						bananaAnglesMap = this.bananaAnglesCoral;
						positionKey = this.targetCoralLevel;
						break;
					}
					case ALGAE: {
						elevatorPositionsMap = this.elevatorPositionsAlgae;
						bananaAnglesMap = this.bananaAnglesAlgae;
						positionKey = this.targetAlgaeLevel;
						break;
					}
					case CAGE: {
						elevatorPositionsMap = this.elevatorPositionsCage;
						bananaAnglesMap = this.bananaAnglesCage;
						positionKey = this.targetCageLevel;
						break;
					}
				}

				// fetch the desired setpoints from each map
				var defaultElevatorHeight = GamePieceType.NONE.getHeight();
				var defaultBananaAngle = GamePieceType.NONE.getPivot();
				var desiredElevatorSetpoint = elevatorPositionsMap.getOrDefault(positionKey, defaultElevatorHeight);
				var desiredBananaSetpoint   = bananaAnglesMap.getOrDefault(positionKey, defaultBananaAngle);

				// feed the setpoints to the actuators
				moveToPosition(desiredElevatorSetpoint);
				pivotBanana(desiredBananaSetpoint);
			},
			/* end() */
			interrupted -> {},
			/* isFinished() */
			() -> false,
			// require "this" subsystem
			this);
	}

	private Command toDefaultPosition() {
		return new RunCommand(() -> {
			pivotBanana(currentGamePieceType.getPivot());
			moveToPosition(currentGamePieceType.getHeight());
		}, this);
	}
}
