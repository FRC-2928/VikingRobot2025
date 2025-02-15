package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ElevatorIOReal implements ElevatorIO{
	public ElevatorIOReal(final Elevator elevator) {
		liftMotorA = new TalonFX(Constants.CAN.CTRE.elevatorMotorA);
		liftMotorB = new TalonFX(Constants.CAN.CTRE.elevatorMotorB);

		pivot = new TalonFX(Constants.CAN.CTRE.bananaPivot);

		this.angle = this.liftMotorA.getRotorPosition();
		this.velocity = this.liftMotorA.getRotorVelocity();

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

		bananaConfig.Slot0 = Constants.Elevator.pivotConfig;

		// Motion Magic Params
		elevatorConfig.MotionMagic.MotionMagicAcceleration = 10;
		elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = 10;

		this.liftMotorA.getConfigurator().apply(elevatorConfig);
		this.liftMotorA.setNeutralMode(NeutralModeValue.Brake);

		this.pivot.getConfigurator().apply(bananaConfig);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);
	}

	private final TalonFX liftMotorA;
	private final TalonFX liftMotorB;

	private final TalonFX pivot;

	private final StatusSignal<Angle> angle;
	private final StatusSignal<AngularVelocity> velocity;

	private final double DISTANCE_CONVERSION_RATIO = 5.0;

	public void moveToPosition(final Distance position) {
		liftMotorA.setControl(new MotionMagicVoltage(DISTANCE_CONVERSION_RATIO * position.in(Units.Meters)));
	}

	public void pivotBanana(final Angle rotation) {
		pivot.setControl(new PositionVoltage(rotation));
	}

	public void updateInputs(final ElevatorIOInputs inputs) {
		inputs.height = Units.Meters.of(angle.getValue().in(Units.Rotations) /  DISTANCE_CONVERSION_RATIO);
		inputs.speed = Units.MetersPerSecond.of(velocity.getValue().in(Units.RotationsPerSecond) /  DISTANCE_CONVERSION_RATIO);
	}
}
