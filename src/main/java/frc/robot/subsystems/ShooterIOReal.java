package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

		this.sensors = this.launcher.getSensorCollection();

		this.intake.setNeutralMode(NeutralMode.Coast);

		final TalonFXConfiguration pivot = new TalonFXConfiguration();
		this.pivot.getConfigurator().refresh(pivot);
		pivot.Feedback.FeedbackRemoteSensorID = Constants.CAN.CTRE.shooterPivot;
		pivot.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
		pivot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		pivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Shooter.max.in(Units.Rotations);
		pivot.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		pivot.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Shooter.intake.in(Units.Rotations);
		this.pivot.getConfigurator().apply(pivot);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);

		this.flywheels.setNeutralMode(NeutralModeValue.Coast);
		this.launcher.setNeutralMode(NeutralMode.Coast);

		this.sysIdPivot = new SysIdRoutine(
			/*
			new SysIdRoutine.Config(
				null,
				null,
				Units.Seconds.of(8),
				state -> Logger.recordOutput("SysId/Pivot/State", state)
			),
			*/
			new SysIdRoutine.Config(),
			new SysIdRoutine.Mechanism(
				voltage -> this.pivot.setControl(new VoltageOut(voltage.in(Units.Volts), true, false, false, false)),
				log -> {
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
				},
				shooter
			)
		);

		SmartDashboard
			.putData(
				"SysId/Pivot/Quasistatic/Forward",
				this.sysIdPivot
					.quasistatic(Direction.kForward)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() >= Constants.Shooter.max.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Pivot/Quasistatic/Reverse",
				this.sysIdPivot
					.quasistatic(Direction.kReverse)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intake
							.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Pivot/Dynamic/Forward",
				this.sysIdPivot
					.dynamic(Direction.kForward)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() >= Constants.Shooter.max.in(Units.Rotations)
					)
			);
		SmartDashboard
			.putData(
				"SysId/Pivot/Dynamic/Reverse",
				this.sysIdPivot
					.dynamic(Direction.kReverse)
					.until(
						() -> this.pivot.getPosition().getValueAsDouble() <= Constants.Shooter.intake
							.in(Units.Rotations)
					)
			);
	}

	public final TalonSRX intake = new TalonSRX(Constants.CAN.Misc.intakeRoller);

	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public final CANcoder encoder = new CANcoder(Constants.CAN.CTRE.shooterEncoder, Constants.CAN.CTRE.bus);

	public final STalonFX flywheels = new STalonFX(Constants.CAN.CTRE.shooterFlywheels, Constants.CAN.CTRE.bus);
	public final TalonSRX launcher = new TalonSRX(Constants.CAN.Misc.shooterLauncher);

	public final StatusSignal<Double> absolutePosition;
	public final StatusSignal<Double> velocity;
	public final SensorCollection sensors;

	public final ArmFeedforward ffw = Constants.Shooter.ffw;

	public final SysIdRoutine sysIdPivot;

	public void applyRotation(final double rotation) {
		this.pivot
			.setControl(
				new VoltageOut(
					this.ffw
						.calculate(
							Units.Rotations.of(this.absolutePosition.getValueAsDouble()).in(Units.Radians),
							rotation
						)
				)
			);
	}

	public void intake(final boolean intake) {

	}

	@Override
	public void updateInputs(final ShooterIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.absolutePosition, this.velocity);

		inputs.angle = Units.Rotations.of(this.absolutePosition.getValueAsDouble());
		inputs.flywheelSpeed = Units.RotationsPerSecond.of(this.velocity.getValueAsDouble());
		inputs.holdingNote = !this.sensors.isFwdLimitSwitchClosed();
	}
}
