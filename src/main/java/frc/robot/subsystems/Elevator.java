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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaePosition;
import frc.robot.Constants.CagePosition;
import frc.robot.Constants.CoralPosition;
import frc.robot.Constants.GamePieceType;

public class Elevator extends SubsystemBase {
	@AutoLog
	public static class ElevatorInputs {
		public Distance height = Units.Meters.zero();
		public boolean isPivotHomed = false;
		public boolean isElevatorHomed = false;
		public AngularVelocity elevatorAngularVelocity = Units.RadiansPerSecond.zero();
		public LinearVelocity speed = Units.MetersPerSecond.zero();
		public Angle pivotAngle = Units.Degrees.of(0);
		public AngularVelocity pivotAngularVelocity = Units.RadiansPerSecond.zero();
		public GamePieceType currentGamePieceType = GamePieceType.NONE;
		public int targetCoralLevel = CoralPosition.NONE.getValue();
		public int targetAlgaeLevel = AlgaePosition.NONE.getValue();
		public int targetCageLevel = CoralPosition.NONE.getValue();
		public Angle pivotCommandedAngle = Units.Degrees.of(20);
		public Angle pivotDesiredAngle = Units.Degrees.of(0);
	}

	public final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

	// --------------------- Internal Higher-Order States ---------------------
	// Target state variables
	private int targetCoralLevel = CoralPosition.NONE.getValue();    // NONE by default
	private int targetAlgaeLevel = AlgaePosition.NONE.getValue();  // NONE by default
	private int targetCageLevel  = CagePosition.NONE.getValue();   // NONE by default
	private GamePieceType currentGamePieceType  = GamePieceType.NONE;  // NONE by default

	// ------------------- Elevator Position Maps -------------------
	// Map of Elevator Positions for Coral
	private final Map<Integer, Distance> elevatorPositionsCoral = Map.of(
		CoralPosition.NONE.getValue(), Units.Inches.of(0.5),
		CoralPosition.L1.getValue(),   Units.Meters.of(0.25),
		CoralPosition.L2.getValue(),   Units.Meters.of(0.472),
		CoralPosition.L3.getValue(),   Units.Meters.of(0.786),
		CoralPosition.L4.getValue(),   Units.Meters.of(1.31));

	// Map of Elevator Positions for Algae
	private final Map<Integer, Distance> elevatorPositionsAlgae = Map.of(
		AlgaePosition.NONE.getValue(), Units.Inches.of(0),
		AlgaePosition.L1.getValue(),   Units.Meters.of(0.05),
		AlgaePosition.L2.getValue(),   Units.Meters.of(0.472),
		AlgaePosition.L3.getValue(),   Units.Meters.of(0.786),
		AlgaePosition.L4.getValue(),   Units.Meters.of(1.31));

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
		CoralPosition.L2.getValue(),   Units.Rotations.of(2.5),
		CoralPosition.L3.getValue(),   Units.Rotations.of(2.5),
		CoralPosition.L4.getValue(),   Units.Rotations.of(6));

	// Map of Banana Angles for Algae
	private final Map<Integer, Angle> bananaAnglesAlgae = Map.of(
		AlgaePosition.NONE.getValue(), Units.Rotations.of(6),
		AlgaePosition.L1.getValue(),   Units.Rotations.of(6),
		AlgaePosition.L2.getValue(),   Units.Rotations.of(6),
		AlgaePosition.L3.getValue(),   Units.Rotations.of(6),
		AlgaePosition.L4.getValue(),   Units.Rotations.of(6));

	// Map of Banana Angles for Cage
	private final Map<Integer, Angle> bananaAnglesCage = Map.of(
		CagePosition.NONE.getValue(),    Units.Degrees.of(0),
		CagePosition.DEEP.getValue(),    Units.Degrees.of(0),
		CagePosition.SHALLOW.getValue(), Units.Degrees.of(0));

	// --------------------- Hardware Interfaces ---------------------
	private final TalonFX liftMotorA;
	private final TalonFX liftMotorB;
	private final TalonFX pivot;
	private boolean haultMode = false;
	private final StatusSignal<Angle> elevatorMotorPosition;
	private final StatusSignal<AngularVelocity> elevatorMotorVelocity;
	private final StatusSignal<ReverseLimitValue> elevatorHomedSignal;
	private final StatusSignal<Current> elevatorMotorStatorCurrent;
	private final StatusSignal<Current> elevatorMotorSupplyCurrent;
	private final StatusSignal<Angle> pivotMotorPosition;
	private final StatusSignal<AngularVelocity> pivotMotorVelocity;
	private final StatusSignal<ReverseLimitValue> pivotHomedSignal;

	private Distance elevatorTargetPosition; // Represents final pos system is trying to reach
	private Angle pivotTargetAngle; // Represents final angle system is trying to reach

	private Distance elevatorCommandedPosition;  // The pos that the motor is currently told to go to
	private Angle pivotCommmandedAngle;  // The angle that the motor is currently told to go to

	private final Distance elevatorThresholdForPivot = Units.Inches.of(1); // The minimum distance that the elevator is allowed to be with a non-zero pivot angle
	private final Angle bananaDangerZoneThreshold = Units.Degrees.of(5);  // TODO: tune this value
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
		this.pivotCommmandedAngle = Units.Degrees.of(20);

		// lift motors are on the CANivore
		liftMotorA = new TalonFX(Constants.CAN.CTRE.elevatorMotorA, Constants.CAN.CTRE.bus);
		liftMotorB = new TalonFX(Constants.CAN.CTRE.elevatorMotorB, Constants.CAN.CTRE.bus);

		// pivot motor is on the RIO CAN bus
		pivot = new TalonFX(Constants.CAN.RIO.bananaPivot, Constants.CAN.RIO.bus);

		final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

		elevatorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteTalonFX;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.CAN.CTRE.elevatorLimitSwitch;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
		elevatorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
		elevatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;  // Gearing, clockwise moves elevator up
		elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		elevatorConfig.SoftwareLimitSwitch
			.withForwardSoftLimitEnable(true)
			.withForwardSoftLimitThreshold(Units.Rotations.of(1.31));
		
		// Peak output amps
		elevatorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
		
		// Supply current limits
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
		elevatorConfig.MotionMagic.MotionMagicExpo_kV = 4;
		elevatorConfig.MotionMagic.MotionMagicExpo_kA = 4;

		liftMotorA.getConfigurator().apply(elevatorConfig);
		liftMotorB.getConfigurator().apply(elevatorConfig);
		liftMotorB.setControl(new Follower(liftMotorA.getDeviceID(), false));


		final TalonFXConfiguration bananaConfig = new TalonFXConfiguration();

		// Configure the reverse limit to read from CANdi S1
		bananaConfig.HardwareLimitSwitch
			.withReverseLimitRemoteCANdiS1(Constants.CAN.RIO.BANANA_CANDI.getInstance())
			.withReverseLimitEnable(true)
			.withReverseLimitAutosetPositionEnable(true)
			.withReverseLimitAutosetPositionValue(Units.Degrees.of(0))
			.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen);

		bananaConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		bananaConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

		this.elevatorMotorPosition = this.liftMotorA.getPosition();
		this.elevatorMotorVelocity = this.liftMotorA.getVelocity();
		this.elevatorMotorStatorCurrent = this.liftMotorA.getStatorCurrent();
		this.elevatorMotorSupplyCurrent = this.liftMotorA.getSupplyCurrent();
		this.elevatorHomedSignal = this.liftMotorA.getReverseLimit();

		this.pivotMotorPosition = this.pivot.getRotorPosition();
		this.pivotMotorVelocity = this.pivot.getRotorVelocity();
		this.pivotHomedSignal = this.pivot.getReverseLimit();


		StatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100),
			elevatorMotorPosition,
			elevatorMotorVelocity,
			elevatorMotorStatorCurrent,
			elevatorMotorSupplyCurrent,
			elevatorHomedSignal,
			pivotMotorPosition,
			pivotMotorVelocity,
			pivotHomedSignal);

		this.setDefaultCommand(toDefaultPosition());
		this.targetCageLevel = CagePosition.SHALLOW.getValue();  // TODO: change this to accept values from SmartDashboard
	}

	private void moveToPosition(final Distance position) {
		this.elevatorTargetPosition = Units.Meters.of(
			MathUtil.clamp(
				position.in(Units.Meters),
				Constants.Elevator.MIN_ELEVATOR_DISTANCE.in(Units.Meters),
				Constants.Elevator.MAX_ELEVATOR_DISTANCE.in(Units.Meters)));
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

	public void controlPositionVelocity(final LinearVelocity voltage){
		liftMotorA.setControl(new VelocityVoltage(voltage.in(Units.MetersPerSecond)));
	}

	private void controlPivot(final Angle rotation, final boolean holdHome) {
		// pivot.setControl(new PositionVoltage(rotation));
		if (holdHome) {
			pivot.setControl(new VoltageOut(-1.5));
			pivotCommmandedAngle = Units.Degrees.of(0);
		} else {
			pivotCommmandedAngle = rotation;
			pivot.setControl(new PositionVoltage(pivotCommmandedAngle));
		}
	}

	public boolean hasCurrentGamePieceType(GamePieceType pieceType) {
		return this.currentGamePieceType == pieceType;
	}

	public boolean isInTargetPos() {
		boolean isElevatorInPosition = elevatorTargetPosition.isNear(inputs.height, toleranceForFinishedMovement);
		boolean isBananaInPosition = pivotTargetAngle.isNear(pivotCommmandedAngle, toleranceForFinishedPivot);
		Logger.recordOutput("Elevator/IsElevatorInPosition", isElevatorInPosition);
		Logger.recordOutput("Elevator/IsBananaInPosition", isBananaInPosition);
		return isElevatorInPosition && isBananaInPosition;
	}

	public void setHaultMode(){
		haultMode = !haultMode;
		currentGamePieceType = (haultMode)? GamePieceType.HAULT : GamePieceType.NONE;
	}

	private void updateMotors() {
		var currentElevatorPosition = Units.Meters.of(elevatorMotorPosition.getValue().in(Units.Rotations));
		var currentBananaAngle = pivotCommmandedAngle;
		// are we currently in the elevator danger zone?
		boolean elevatorInDangerZone = currentElevatorPosition.lt(elevatorThresholdForPivot);
		// is the elevator target within the danger zone?
		boolean elevatorTargetInDangerZone = elevatorTargetPosition.lt(elevatorThresholdForPivot);
		// is the banana in a dangerous orientation?
		boolean bananaAngleInDangerZone = currentBananaAngle.gt(bananaDangerZoneThreshold);
		// are we trying to move the banana into a dangerous orientation?
		boolean bananaTargetInDangerZone = pivotTargetAngle.gt(bananaDangerZoneThreshold);

		Logger.recordOutput("Elevator/ElevatorInDangerZone", elevatorInDangerZone);
		Logger.recordOutput("Elevator/ElevatorTargetInDangerZone", elevatorTargetInDangerZone);
		Logger.recordOutput("Elevator/BananaAngleInDangerZone", bananaAngleInDangerZone);
		Logger.recordOutput("Elevator/BananaTargetInDangerZone", bananaTargetInDangerZone);
		Logger.recordOutput("Elevator/ElevatorTargetPositionMeters", elevatorTargetPosition.in(Units.Meters));
		Logger.recordOutput("Elevator/BananaTargetAngleDegrees", pivotTargetAngle.in(Units.Degrees));
		Logger.recordOutput("Elevator/InTargetPosition", isInTargetPos());

		if(currentGamePieceType == GamePieceType.HAULT){
			controlPositionVelocity(Units.MetersPerSecond.of(0));
			controlPivot(Units.Degrees.of(0), true);
			return;
		}
		if (elevatorInDangerZone && elevatorTargetInDangerZone) {
			// in the danger zone and staying in the danger zone -- safe to move the elevator
			controlPosition(elevatorTargetPosition);
			controlPivot(Units.Degrees.of(0), true);
			return;
		}

		if (elevatorInDangerZone && !elevatorTargetInDangerZone) {
			// in the danger zone but moving out of it -- safe to move the elevator
			controlPosition(elevatorTargetPosition);
			controlPivot(pivotTargetAngle, true);
		}

		if (!elevatorInDangerZone && elevatorTargetInDangerZone) {
			// not in the danger zone but moving into it... check the banana first
			if (bananaAngleInDangerZone || bananaTargetInDangerZone) {
				// banana is in danger -- move the banana but not the elevator
				controlPivot(pivotTargetAngle, true);
				return;
			}

			// banana is safe and will remain safe -- control both
			controlPivot(pivotTargetAngle, !bananaTargetInDangerZone);
			controlPosition(elevatorTargetPosition);
			return;
		}

		if (!elevatorInDangerZone && !elevatorTargetInDangerZone) {
			// no risk to banana -- move both
			controlPivot(pivotTargetAngle, !bananaTargetInDangerZone);
			controlPosition(elevatorTargetPosition);
			return;
		}

		// if (elevatorCommandedPosition != elevatorTargetPosition) {
		// 	if (elevatorTargetPosition.gte(elevatorThresholdForPivot)) {
		// 		controlPosition(elevatorTargetPosition);
		// 	}
		// 	else {
		// 		if (this.inputs.isPivotHomed) {
		// 			controlPosition(elevatorTargetPosition);
		// 		}
		// 	}
		// }

		// if ()
		// if (pivotCommmandedAngle /*current */ != pivotTargetAngle /*desired */) {
		// 	if (pivotTargetAngle.lt(Units.Degrees.of(20))) {
		// 		// go to home and stay home
		// 		controlPivot(pivotCommmandedAngle, true /* hold home */);
		// 		// TODO: clean up
		// 		// if (inputs.isPivotHomed) {
		// 		// 	this.pivotTargetAngle = Units.Degrees.of(0.2);
		// 		// 	controlPivot(Units.Degrees.of(0.2));
		// 		// } else {
		// 		// 	controlPivotHome();
		// 		// }
		// 	}
		// 	// else if 
		// 	else {
		// 		if (inputs.height.gt(elevatorThresholdForPivot) && elevatorCommandedPosition.gt(elevatorThresholdForPivot)) {
		// 			controlPivot(pivotTargetAngle, false);
		// 		}
		// 	}
		// }

		// if (elevatorTargetPosition.lt(elevatorThresholdForPivot) && pivotCommmandedAngle.gt(Units.Degrees.of(1))) {
		// 	controlPivot(Units.Degrees.of(0), true);
		// }
	}

	public void setElevatorMode(GamePieceType type){
		currentGamePieceType = type;
	}
	private void updateInputs(final ElevatorInputs inputs) {
		BaseStatusSignal
			.refreshAll(
				this.elevatorMotorPosition,
				this.elevatorMotorVelocity,
				this.elevatorHomedSignal,
				this.elevatorMotorStatorCurrent,
				this.elevatorMotorSupplyCurrent
			);

		// Second refresh for RIO CAN bus
		BaseStatusSignal.refreshAll(
			this.pivotMotorPosition,
			this.pivotMotorVelocity,
			this.pivotHomedSignal);

		inputs.height = Units.Meters.of(elevatorMotorPosition.getValue().in(Units.Rotations));
		inputs.speed = Units.MetersPerSecond.of(elevatorMotorVelocity.getValue().in(Units.RotationsPerSecond));
		inputs.isElevatorHomed = (elevatorHomedSignal.getValue() == ReverseLimitValue.Open); /*inputs.height.in(Units.Meters) < 1e-7;*/
		
		inputs.pivotAngle = pivotMotorPosition.getValue();
		inputs.pivotAngularVelocity = pivotMotorVelocity.getValue();
		inputs.isPivotHomed = (pivotHomedSignal.getValue() == ReverseLimitValue.ClosedToGround); /*inputs.pivotAngle.in(Units.Degrees) < 10*/;

		inputs.currentGamePieceType = this.currentGamePieceType;
		inputs.targetCoralLevel = this.targetCoralLevel;
		inputs.targetAlgaeLevel = this.targetAlgaeLevel;
		inputs.targetCageLevel = this.targetCageLevel;
		inputs.pivotCommandedAngle = pivotCommmandedAngle;
		inputs.pivotDesiredAngle = pivotTargetAngle;
	}

	@Override
	public void periodic() {
		this.updateInputs(this.inputs);
		Logger.processInputs("Elevator", this.inputs);
		this.updateMotors();
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
		this.targetCoralLevel = MathUtil.clamp(this.targetCoralLevel-1, 0, 4);
		this.targetAlgaeLevel = MathUtil.clamp(this.targetAlgaeLevel-1, 0, 4);
	}

	public void toggleReefHeightUp() {
		this.targetCoralLevel = MathUtil.clamp(this.targetCoralLevel+1, 0, 4);
		this.targetAlgaeLevel = MathUtil.clamp(this.targetAlgaeLevel+1, 0, 4);
	}

	public void onEjectAlgae() {
		// any algae specific logic here
		onEjectGamePieceGeneric();
	}

	public void onEjectCoral() {
		// any coral specific logic here
		onEjectGamePieceGeneric();
	}

	public void onEjectCoralAuto() {
		this.currentGamePieceType = GamePieceType.CORAL;
		pivotBanana(currentGamePieceType.getPivot());
		moveToPosition(currentGamePieceType.getHeight());
		this.targetAlgaeLevel = 0;
		this.targetCoralLevel = 0;
	}

	private void onEjectGamePieceGeneric() {
		// reset the home to whatever it was before...
		this.currentGamePieceType = GamePieceType.NONE;
		pivotBanana(currentGamePieceType.getPivot());
		moveToPosition(currentGamePieceType.getHeight());
		this.targetAlgaeLevel = 0;
		this.targetCoralLevel = 0;
	}

	public Command setTargetAlgaeLevelCommand(AlgaePosition targetAlgaeLevel){
		return new InstantCommand(() -> {
			this.targetAlgaeLevel = targetAlgaeLevel.getValue();
		});
	}

	public Command toggleClimbMode(){
		if(currentGamePieceType == GamePieceType.CAGE){
			return new InstantCommand(() -> {
				setElevatorMode(GamePieceType.NONE);
			});
		}
		return new InstantCommand(() -> {
			setElevatorMode(GamePieceType.CAGE);
			setTargetCageLevel(CagePosition.SHALLOW); // Hard code the climb to SHALLOW because our climb is shallow
		});
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
		return new RunCommand(
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
			// require "this" subsystem
			this);
	}

	private Command toDefaultPosition() {
		return new RunCommand(() -> {
			pivotBanana(currentGamePieceType.getPivot());
			moveToPosition(currentGamePieceType.getHeight());
		}, this);
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
}
