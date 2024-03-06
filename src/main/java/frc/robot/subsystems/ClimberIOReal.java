package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.STalonFX;

public class ClimberIOReal implements ClimberIO {
	public ClimberIOReal() {
		final TalonFXConfiguration actuator = new TalonFXConfiguration();

		actuator.HardwareLimitSwitch.ReverseLimitEnable = true;
		actuator.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
		actuator.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.RemoteCANcoder;
		actuator.HardwareLimitSwitch.ReverseLimitRemoteSensorID = Constants.CAN.CTRE.climber;
		actuator.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
		actuator.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
		actuator.Slot0 = Slot0Configs.from(Constants.Climber.configFast);
		actuator.Slot1 = Slot1Configs.from(Constants.Climber.configSlow);

		actuator.Audio = Constants.talonFXAudio;

		this.actuator.getConfigurator().apply(actuator);

		this.actuator.setInverted(true);
		this.actuator.setNeutralMode(NeutralModeValue.Brake);

		this.position = this.actuator.getPosition();
		this.home = this.actuator.getReverseLimit();

		Robot.cont.diag.motors.add(this.actuator);
	}

	public final STalonFX actuator = new STalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus);

	public final Servo lock = new Servo(0);

	public final StatusSignal<Double> position;
	public final StatusSignal<ReverseLimitValue> home;

	private boolean disengaging;
	private double targetDisengagePosition;

	@Override
	public void set(final double position, final boolean fast) {
		Logger.recordOutput("Climber/DriveTarget", position);
		Logger.recordOutput("Climber/Disengaging", this.disengaging);

		if(position - this.position.getValueAsDouble() <= 0) {
			this.actuator.setControl(new PositionDutyCycle(position).withSlot(fast ? 0 : 1));

			this.overrideLock(true);
			this.disengaging = false;

			return;
		}

		if(this.disengaging) {
			if(
				this.position.getValueAsDouble() < this.targetDisengagePosition
					|| this.home.getValue() == ReverseLimitValue.ClosedToGround
			) this.disengaging = false;
			else this.actuator.setControl(new PositionDutyCycle(this.targetDisengagePosition).withSlot(fast ? 0 : 1));
		} else {
			if(this.lock.getAngle() == Constants.Climber.ratchetLocked.in(Units.Degrees)) {
				this.overrideLock(false);
				this.disengaging = true;
				this.targetDisengagePosition = this.position.getValueAsDouble() - Constants.Climber.disengageDistance;
			} else this.actuator.setControl(new PositionDutyCycle(position).withSlot(fast ? 0 : 1));
		}
	}

	@Override
	public void offset(final double offset, final boolean fast) {
		this.set(this.position.getValueAsDouble() + offset, fast);
	}

	@Override
	public void overrideLock(final boolean engaged) {
		this.lock
			.setAngle((engaged ? Constants.Climber.ratchetLocked : Constants.Climber.ratchetFree).in(Units.Degrees));
		Logger.recordOutput("Climber/LockEngaged");
	}

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position, this.home);

		inputs.position = this.position.getValueAsDouble();
		inputs.home = this.home.getValue() == ReverseLimitValue.ClosedToGround;
	}
}
