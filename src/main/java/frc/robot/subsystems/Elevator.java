package frc.robot.subsystems;

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
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
	@AutoLog
	public static class ElevatorInputs {
		public Distance height;
		public boolean homePosPivot;
		public boolean homePosElevator;
		public LinearVelocity speed;
		public Angle pivotAngle;
		public AngularVelocity pivoAngularVelocity;
	}

	public final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
	// public final ElevatorInputs inputs = new ElevatorInputs();

	private final TalonFX liftMotorA;
	private final TalonFX liftMotorB;
	private final TalonFX pivot;

	private final StatusSignal<Angle> elevatorMotorPosition;
	private final StatusSignal<AngularVelocity> elevatorMotorVelocity;
	private final StatusSignal<AngularVelocity> pivotMotorVelocity;
	private final StatusSignal<Angle> pivotMotorPosition;

	private Distance elevatorTargetPosition; // Represesnts final pos system is trying to reach
	private Angle pivotTargetAngle; // Represents final angle system is trying to reach

	private Distance elevatorCommandedPosition; // The pos that the motor is currently told to go to
	private Angle pivotCommmandedAngle; // The angle that the motor is currently told to go to

	private final Distance elevatorThresholdForPivot = Units.Inches.of(8); // The minimum distance that the elevator is allowed to be with a non-zero pivot angle
	private final Distance toleranceForFinishedMovement = Units.Millimeters.of(7);
	private final Angle toleranceForFinishedPivot = Units.Degrees.of(2);
	

	// Simulation objects
	private final ElevatorSim elevatorSim = new ElevatorSim(
		LinearSystemId.createElevatorSystem(
			DCMotor.getKrakenX60Foc(2), Units.Pounds.of(12).in(Units.Kilogram), 
			Constants.Elevator.DRUM_RADIUS.in(Units.Meters), 
			Constants.Elevator.ELEVATOR_GEARING / Constants.Elevator.NUMBER_OF_STAGES), 
		DCMotor.getKrakenX60Foc(2), 
		0, 
		Units.Inches.of(75).in(Units.Meters),
		false, 
		0);

	public Elevator() {

		liftMotorA = new TalonFX(Constants.CAN.CTRE.elevatorMotorA);
		liftMotorB = new TalonFX(Constants.CAN.CTRE.elevatorMotorB);

		pivot = new TalonFX(Constants.CAN.CTRE.bananaPivot);

		this.elevatorMotorPosition = this.liftMotorA.getRotorPosition();
		this.elevatorMotorVelocity = this.liftMotorA.getRotorVelocity();

		this.pivotMotorPosition = this.pivot.getRotorPosition();
		this.pivotMotorVelocity = this.pivot.getRotorVelocity();

		liftMotorB.setControl(new Follower(liftMotorA.getDeviceID(), false));

		final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

		final TalonFXConfiguration bananaConfig = new TalonFXConfiguration();

		// Peak output amps
		elevatorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		elevatorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		elevatorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		bananaConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		bananaConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		bananaConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		bananaConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		// Supply current limits
		elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		elevatorConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		elevatorConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		elevatorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		bananaConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		bananaConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		bananaConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		bananaConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		// PID values
		elevatorConfig.Slot0 = Slot0Configs.from(Constants.Elevator.elevatorConfig);
		elevatorConfig.Feedback.SensorToMechanismRatio = Constants.Elevator.DISTANCE_CONVERSION_RATIO;

		bananaConfig.Slot0 = Constants.Elevator.pivotConfig;

		// Motion Magic Params
		// elevatorConfig.MotionMagic.MotionMagicAcceleration = 10;
		// elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 3.833 * Constants.Elevator.DISTANCE_CONVERSION_RATIO;
		elevatorConfig.MotionMagic.MotionMagicExpo_kV = 0.15;
		elevatorConfig.MotionMagic.MotionMagicExpo_kA = 0.01;

		this.liftMotorA.getConfigurator().apply(elevatorConfig);
		this.liftMotorA.setNeutralMode(NeutralModeValue.Brake);

		this.pivot.getConfigurator().apply(bananaConfig);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);

		StatusSignal.setUpdateFrequencyForAll(50, elevatorMotorPosition, elevatorMotorVelocity, pivotMotorPosition, pivotMotorVelocity);
	}

	public void moveToPosition(final Distance position) {
		
		this.elevatorTargetPosition = Units.Meters.of(MathUtil.clamp(position.in(Units.Meters), Constants.Elevator.MIN_ELEVATOR_DISTANCE.in(Units.Meters), Constants.Elevator.MAX_ELEVATOR_DISTANCE.in(Units.Meter)));
	}

	public void pivotBanana(final Angle rotation) {
		
		this.pivotTargetAngle = Units.Degrees.of(MathUtil.clamp(rotation.in(Units.Degrees), Constants.Elevator.MIN_PIVOT_ANGLE.in(Units.Degrees), Constants.Elevator.MAX_PIVOT_ANGLE.in(Units.Degrees)));
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

	public boolean isInTargetPos() {
		if (elevatorTargetPosition.gt(elevatorThresholdForPivot)) {
			return elevatorTargetPosition.isNear(inputs.height, toleranceForFinishedMovement) && pivotTargetAngle.isNear(inputs.pivotAngle, toleranceForFinishedPivot);
		}
		return elevatorTargetPosition.isNear(inputs.height, toleranceForFinishedMovement) && Units.Degrees.of(0).isNear(inputs.pivotAngle, toleranceForFinishedPivot);
	}

	private void updateMotors() {
		if (elevatorCommandedPosition != elevatorTargetPosition) {
			if (elevatorTargetPosition.gte(elevatorThresholdForPivot)) {
				controlPosition(elevatorTargetPosition);
			}
			else {
				if (this.inputs.homePosPivot) {
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


	public void updateInputs(final ElevatorInputs inputs) {
		BaseStatusSignal
			.refreshAll(
				this.elevatorMotorPosition,
				this.elevatorMotorVelocity,
				this.pivotMotorPosition,
				this.pivotMotorVelocity
			);

		inputs.height = Units.Meters.of(elevatorMotorPosition.getValue().in(Units.Rotations));
		inputs.speed = Units.MetersPerSecond.of(elevatorMotorVelocity.getValue().in(Units.RotationsPerSecond));
		inputs.homePosElevator = inputs.height.in(Units.Meters) < 1e-7;
		
		inputs.pivotAngle = pivotMotorPosition.getValue();
		inputs.pivoAngularVelocity = pivotMotorVelocity.getValue();
		inputs.homePosPivot = inputs.pivotAngle.in(Units.Degrees) < 1;
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

		liftMotorASimState.setRawRotorPosition(elevatorSim.getPositionMeters());
		liftMotorASimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());
		liftMotorBSimState.setRawRotorPosition(elevatorSim.getPositionMeters());
		liftMotorBSimState.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond());

		Logger.recordOutput("Elevator/Sim/LiftVoltage", liftMotorASimState.getMotorVoltage());
	}


	public Command setTargetCommand(Distance distance, Angle angle) {
		return new SequentialCommandGroup(
			new InstantCommand(() -> {
				moveToPosition(distance);
				pivotBanana(angle);
			}, this),
			new RunCommand(() -> {}, this).until(this::isInTargetPos)
		); 
	}

	public Command toL1Coral() {
		return setTargetCommand(Units.Feet.of(2.5), Units.Degrees.of(0));
	}

	public Command toL2Coral() {
		return setTargetCommand(Units.Feet.of(3.5), Units.Degrees.of(0));
	}

	public Command toL3Coral() {
		return setTargetCommand(Units.Feet.of(4.5), Units.Degrees.of(0));
	}

	public Command toL4Coral() {
		return setTargetCommand(Units.Feet.of(6.4), Units.Degrees.of(45));
	}

	public Command toL2Algae() {
		return setTargetCommand(Units.Feet.of(4), Units.Degrees.of(0));
	}

	public Command toL3Algae() {
		return setTargetCommand(Units.Feet.of(5), Units.Degrees.of(0));
	}

	public Command processorAlgae() {
		return setTargetCommand(Units.Feet.of(1), Units.Degrees.of(0));
	}

	public Command toHome() {
		return setTargetCommand(Units.Feet.of(0), Units.Degrees.of(0));
	}
}
