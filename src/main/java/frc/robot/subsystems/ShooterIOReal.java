package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.*;
import frc.robot.Constants;
import frc.robot.utils.STalonFX;

public class ShooterIOReal implements ShooterIO {
	public ShooterIOReal() {
		this.absolutePosition = this.encoder.getAbsolutePosition();
		this.sensors = this.launcher.getSensorCollection();
		this.velocity = this.flywheels.getVelocity();

		this.intake.setNeutralMode(NeutralMode.Coast);

		final TalonFXConfiguration pivot = new TalonFXConfiguration();
		this.pivot.getConfigurator().refresh(pivot);
		pivot.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		//pivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Shooter.max.in(Units.Rotations);
		pivot.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25;
		this.pivot.getConfigurator().apply(pivot);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);

		this.flywheels.setNeutralMode(NeutralModeValue.Coast);
		this.launcher.setNeutralMode(NeutralMode.Coast);
	}

	public final TalonSRX intake = new TalonSRX(Constants.CAN.Misc.intakeRoller);

	public final STalonFX pivot = new STalonFX(Constants.CAN.CTRE.shooterPivot, Constants.CAN.CTRE.bus);
	public final CANcoder encoder = new CANcoder(Constants.CAN.CTRE.shooterEncoder, Constants.CAN.CTRE.bus);

	public final STalonFX flywheels = new STalonFX(Constants.CAN.CTRE.shooterFlywheels, Constants.CAN.CTRE.bus);
	public final TalonSRX launcher = new TalonSRX(Constants.CAN.Misc.shooterLauncher);

	public final StatusSignal<Double> absolutePosition;
	public final SensorCollection sensors;
	public final StatusSignal<Double> velocity;

	public final ArmFeedforward ffw = Constants.Shooter.ffw;

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
		inputs.angle = Units.Rotations.of(this.absolutePosition.getValueAsDouble());
		inputs.holdingNote = !this.sensors.isFwdLimitSwitchClosed();
		inputs.flywheelSpeed = Units.RotationsPerSecond.of(this.velocity.getValueAsDouble());
	}
}
