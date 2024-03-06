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
import edu.wpi.first.wpilibj.Timer;
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

		this.overrideLock(true);
	}

	public final STalonFX actuator = new STalonFX(Constants.CAN.CTRE.climber, Constants.CAN.CTRE.bus);

	public final Servo lock = new Servo(0);

	public final StatusSignal<Double> position;
	public final StatusSignal<ReverseLimitValue> home;

	private boolean disengaging;
	private double targetDisengagePosition;

	private double demand;
	private boolean demandFast;

	@Override
	public void set(final double position, final boolean fast) {
		this.demand = position;
		this.demandFast = fast;
	}

	@Override
	public void offset(final double offset, final boolean fast) {
		this.set(this.position.getValueAsDouble() + offset, fast);
	}

	@Override
	public void overrideLock(final boolean engaged) {
		this.lock.setAngle(engaged ? Constants.Climber.ratchetLocked : Constants.Climber.ratchetFree);
		Logger.recordOutput("Climber/LockEngaged", engaged);
	}

	@Override
	public void updateInputs(final ClimberIOInputs inputs) {
		BaseStatusSignal.refreshAll(this.position, this.home);

		inputs.position = this.position.getValueAsDouble();
		inputs.home = this.home.getValue() == ReverseLimitValue.ClosedToGround;
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Climber/DriveUp", this.demand == 129);
		Logger.recordOutput("Climber/Disengaging", this.disengaging);
		Logger.recordOutput("Climber/RatchetLocked", this.lock.getAngle() == Constants.Climber.ratchetLocked);
		Logger.recordOutput("Climber/Tick", Timer.getFPGATimestamp());

		if(this.demand <= this.position.getValueAsDouble()) {
			this.setpos(this.demand);

			this.overrideLock(true);
			this.disengaging = false;

			Logger.recordOutput("Climber/State", "Eng");
		} else {
			if(this.disengaging) {
				if(
					this.position.getValueAsDouble() < this.targetDisengagePosition
						|| this.home.getValue() == ReverseLimitValue.ClosedToGround
				) {
					this.disengaging = false;

					this.setpos(this.demand);

					Logger.recordOutput("Climber/State", "DisFin");
				} else {
					this.setpos(this.targetDisengagePosition);

					Logger.recordOutput("Climber/State", "DisMove");
				}
			} else {
				if(this.lock.getAngle() == Constants.Climber.ratchetLocked) {
					this.overrideLock(false);

					this.disengaging = true;
					this.targetDisengagePosition = this.position.getValueAsDouble()
						- Constants.Climber.disengageDistance;

					this.setpos(this.targetDisengagePosition);

					Logger.recordOutput("Climber/State", "BegDis");
				} else {
					this.setpos(this.demand);

					Logger.recordOutput("Climber/State", "AlreadyDis");
				}
			}
		}
	}

	private void setpos(final double target) {
		this.actuator.setControl(new PositionDutyCycle(target).withSlot(this.demandFast ? 0 : 1));
		Logger.recordOutput("Climber/TargetPosition", target);
	}
}
