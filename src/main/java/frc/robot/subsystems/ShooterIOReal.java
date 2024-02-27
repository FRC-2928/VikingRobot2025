package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants;
import frc.robot.utils.STalonFX;

public class ShooterIOReal implements ShooterIO {
	public ShooterIOReal(final Shooter shooter) {
		this.absolutePosition = this.encoder.getAbsolutePosition();
		this.velocity = this.flywheels.getVelocity();

		BaseStatusSignal.setUpdateFrequencyForAll(100, this.absolutePosition, this.velocity);
		this.pivot.optimizeBusUtilization();
		this.encoder.optimizeBusUtilization();
		this.flywheels.optimizeBusUtilization();

		this.sensors = this.feeder.getSensorCollection();

		final TalonFXConfiguration pivot = new TalonFXConfiguration();

		pivot.Feedback.FeedbackRemoteSensorID = Constants.CAN.CTRE.shooterPivot;
		pivot.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

		pivot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		pivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Shooter.max.in(Units.Rotations);
		pivot.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		pivot.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Shooter.intake.in(Units.Rotations);

		pivot.Slot0 = Slot0Configs.from(Constants.Shooter.pivotConfig);

		pivot.Audio = Constants.talonFXAudio;

		this.pivot.getConfigurator().apply(pivot);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);
		this.pivot.setInverted(false);

		final TalonFXConfiguration flywheels = new TalonFXConfiguration();

		flywheels.Audio = Constants.talonFXAudio;

		this.flywheels.getConfigurator().apply(flywheels);
		this.flywheels.setNeutralMode(NeutralModeValue.Coast);
		this.flywheels.setInverted(true);

		this.feeder.setNeutralMode(NeutralMode.Coast);
		this.feeder.setInverted(true);
		this.intake.setNeutralMode(NeutralMode.Coast);
		this.intake.setInverted(true);

		this.sysIdPivot = new SysIdRoutine(
			new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(1.0), Units.Volts.of(7.0), null, state -> {
				Logger.recordOutput("SysId/Shooter/Pivot/State", state.toString());
				//this.sysIdPivot.recordState(state);

				BaseStatusSignal
					.setUpdateFrequencyForAll(
						state != State.kNone ? 100 : 0, // don't update unless currently running sysid
						this.pivot.getMotorVoltage(),
						this.pivot.getPosition(),
						this.pivot.getVelocity()
					);
			}),
			new SysIdRoutine.Mechanism(
				voltage -> this.pivot.setControl(new VoltageOut(voltage.in(Units.Volts), true, false, false, false)),
				log -> {
					//*
					Logger
						.recordOutput(
							"SysId/Shooter/Pivot/Voltage",
							Units.Volts.of(this.pivot.getSupplyVoltage().getValueAsDouble())
						);
					Logger
						.recordOutput(
							"SysId/Shooter/Pivot/Position",
							Units.Rotations.of(this.pivot.getPosition().getValueAsDouble())
						);
					Logger
						.recordOutput(
							"SysId/Shooter/Pivot/Velocity",
							Units.RotationsPerSecond.of(this.pivot.getVelocity().getValueAsDouble())
						);
					//*/
					/*
					log
						.motor("pivot")
						.voltage(Units.Volts.of(this.pivot.getMotorVoltage().getValueAsDouble()))
						.current(Units.Amps.of(this.pivot.getTorqueCurrent().getValueAsDouble()))
						.angularPosition(Units.Rotations.of(this.pivot.getPosition().getValueAsDouble()))
						.angularVelocity(Units.RotationsPerSecond.of(this.pivot.getVelocity().getValueAsDouble()))
						.angularAcceleration(
							Units.RotationsPerSecond
								.per(Units.Second)
								.of(this.pivot.getAcceleration().getValueAsDouble())
						);
					*/
				},
				shooter
			)
		);

		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Quasistatic/Forward",
				this.sysIdPivot
					.quasistatic(Direction.kForward)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() >= Constants.Shooter.max.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Quasistatic/Reverse",
				this.sysIdPivot
					.quasistatic(Direction.kReverse)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intake
							.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Dynamic/Forward",
				this.sysIdPivot
					.dynamic(Direction.kForward)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() >= Constants.Shooter.max.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Shooter/Pivot/Dynamic/Reverse",
				this.sysIdPivot
					.dynamic(Direction.kReverse)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intake
							.in(Units.Rotations)
					)
			);
	}

	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public final CANcoder encoder = new CANcoder(Constants.CAN.CTRE.shooterEncoder, Constants.CAN.CTRE.bus);

	public final STalonFX flywheels = new STalonFX(Constants.CAN.CTRE.shooterFlywheels, Constants.CAN.CTRE.bus);
	public final TalonSRX feeder = new TalonSRX(Constants.CAN.Misc.feederLauncher);
	public final TalonSRX intake = new TalonSRX(Constants.CAN.Misc.intakeRoller);

	public final StatusSignal<Double> absolutePosition;
	public final StatusSignal<Double> velocity;
	public final SensorCollection sensors;

	//public final ArmFeedforward flywheelFeedforward = Constants.Shooter.flywheelFeedforward;

	public final SysIdRoutine sysIdPivot;

	@Override
	public void rotate(final Measure<Angle> target) {
		Logger.recordOutput("Shooter/Pivot/Desired", target);
		this.pivot.setControl(new PositionDutyCycle(target.in(Units.Rotations)));
	}

	@Override
	public void runFlywheels(final Measure<Velocity<Angle>> demand) {
		/*
		this.flywheels
			.setControl(
				new VoltageOut(
					this.flywheelFeedforward
						.calculate(
							Units.Rotations.of(this.absolutePosition.getValueAsDouble()).in(Units.Radians),
							demand.in(Units.RotationsPerSecond)
						)
				)
			);
		*/
		this.flywheels.set(demand.in(Units.RotationsPerSecond) / 90.0);
	}

	@Override
	public void runFeeder(final Demand demand) { this.feeder.set(ControlMode.PercentOutput, demand.dir); }

	@Override
	public void runIntake(final Demand demand) { this.intake.set(ControlMode.PercentOutput, demand.dir * 0.9); }

	@Override
	public void updateInputs(final ShooterIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.absolutePosition, this.velocity);

		inputs.angle = Units.Rotations.of(this.absolutePosition.getValueAsDouble());
		inputs.flywheelSpeed = Units.RotationsPerSecond.of(this.velocity.getValueAsDouble());
		inputs.holdingNote = !this.sensors.isFwdLimitSwitchClosed();
	}
}
