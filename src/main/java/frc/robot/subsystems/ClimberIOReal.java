package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.utils.STalonFX;

public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		final HardwareLimitSwitchConfigs cfg = new HardwareLimitSwitchConfigs();
		this.actuator.getConfigurator().refresh(cfg);
		this.actuator
			.getConfigurator()
			.apply(
				cfg
					.withReverseLimitEnable(true)
					.withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
					.withReverseLimitSource(ReverseLimitSourceValue.RemoteCANcoder)
					.withReverseLimitRemoteSensorID(Constants.CAN.CTRE.climber)
					.withReverseLimitAutosetPositionEnable(true)
					.withReverseLimitAutosetPositionValue(0)
			);
		this.actuator.setInverted(true);
		this.actuator.setNeutralMode(NeutralModeValue.Brake);

		this.ticks = this.actuator.getPosition();
		this.limit = this.actuator.getReverseLimit();
	}

	public final STalonFX actuator = new STalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus);

	public final Servo lock = new Servo(0);

	public final StatusSignal<Double> ticks;
	public final StatusSignal<ReverseLimitValue> limit;

	public final SimpleMotorFeedforward ffw = Constants.Climber.ffw;

	@Override
	public void drive(final Measure<Velocity<Distance>> velocity) {
		this.actuator.setControl(new VoltageOut(this.ffw.calculate(velocity.in(Units.InchesPerSecond))));
	}

	@Override
	public void zero() { this.actuator.setPosition(0); }

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		inputs.ticks = this.ticks.getValueAsDouble();
		inputs.limit = this.limit.getValue() == ReverseLimitValue.ClosedToGround;
	}
}
