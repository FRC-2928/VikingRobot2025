package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.STalonFX;

public class ShooterIOReal implements ShooterIO {
	public ShooterIOReal(final Shooter shooter) {
		this.absolutePosition = this.encoder.getAbsolutePosition();
		this.velocity = this.flywheels.getVelocity();

		BaseStatusSignal.setUpdateFrequencyForAll(100, this.absolutePosition, this.velocity);
		this.pivot.optimizeBusUtilization();
		this.encoder.optimizeBusUtilization();
		this.flywheels.optimizeBusUtilization();
		this.flywheelsFollower.optimizeBusUtilization();

		this.sensors = this.feeder.getSensorCollection();

		final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

		pivotConfig.Feedback.FeedbackRemoteSensorID = Constants.CAN.CTRE.shooterPivot;
		pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

		pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Shooter.max.in(Units.Rotations);
		pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Shooter.intakeGround.in(Units.Rotations);

		pivotConfig.Slot0 = Slot0Configs.from(Constants.Shooter.pivotPositionConfig);
		pivotConfig.Slot1 = Slot1Configs.from(Constants.Shooter.pivotVelocityConfig);

		pivotConfig.Audio = Constants.talonFXAudio;

		this.pivot.getConfigurator().apply(pivotConfig);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);
		this.pivot.setInverted(false);

		final TalonFXConfiguration flywheelsConfig = new TalonFXConfiguration();

		flywheelsConfig.Audio = Constants.talonFXAudio;

		this.flywheels.getConfigurator().apply(flywheelsConfig);
		this.flywheels.setNeutralMode(NeutralModeValue.Coast);
		this.flywheels.setInverted(true);

		this.flywheelsFollower.getConfigurator().apply(flywheelsConfig);
		this.flywheelsFollower.setNeutralMode(NeutralModeValue.Coast);
		this.flywheelsFollower.setControl(new Follower(this.flywheels.getDeviceID(), true));

		this.feeder.setNeutralMode(NeutralMode.Coast);
		this.feeder.setInverted(true);
		this.intake.setNeutralMode(NeutralMode.Coast);
		this.intake.setInverted(true);

		Robot.cont.diag.motors.add(this.pivot);
		Robot.cont.diag.motors.add(this.flywheels);

		this.sysIdPivot = new SysIdRoutine(
			new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(1), Units.Volts.of(7), null, state -> {
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
				voltage -> this.pivot.setControl(new VoltageOut(voltage.in(Units.Volts))),
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
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intakeGround
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
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intakeGround
							.in(Units.Rotations)
					)
			);
	}

	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public final CANcoder encoder = new CANcoder(Constants.CAN.CTRE.shooterEncoder, Constants.CAN.CTRE.bus);

	public final STalonFX flywheels = new STalonFX(Constants.CAN.CTRE.shooterFlywheels, Constants.CAN.CTRE.bus);
	public final STalonFX flywheelsFollower = new STalonFX(Constants.CAN.CTRE.shooterFlywheelsFollower, Constants.CAN.CTRE.bus);
	public final TalonSRX feeder = new TalonSRX(Constants.CAN.Misc.feederLauncher);
	public final TalonSRX intake = new TalonSRX(Constants.CAN.Misc.intakeRoller);

	public final StatusSignal<Double> absolutePosition;
	public final StatusSignal<Double> velocity;
	public final SensorCollection sensors;

	//public final ArmFeedforward flywheelFeedforward = Constants.Shooter.flywheelFeedforward;

	public final SysIdRoutine sysIdPivot;

	@Override
	public void rotate(final Measure<Angle> target) {
		Logger.recordOutput("Shooter/Pivot/ControlPosition", target.in(Units.Degrees));
		this.pivot.setControl(new PositionDutyCycle(target.in(Units.Rotations)));
	}

	@Override
	public void runFlywheels(final Measure<Velocity<Angle>> demand) {
		this.flywheels.set(demand.in(Units.RotationsPerSecond) / 90);
	}

	@Override
	public void runFeeder(final Demand demand) { this.feeder.set(ControlMode.PercentOutput, demand.dir); }

	@Override
	public void runIntake(final Demand demand) { this.intake.set(ControlMode.PercentOutput, demand.dir); }

	@Override
	public void updateInputs(final ShooterIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.absolutePosition, this.velocity);

		inputs.angle = Units.Rotations.of(this.absolutePosition.getValueAsDouble());
		inputs.flywheelSpeed = Units.RotationsPerSecond.of(this.velocity.getValueAsDouble());
		inputs.holdingNote = !this.sensors.isFwdLimitSwitchClosed();
	}
}
