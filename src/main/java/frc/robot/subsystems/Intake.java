package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
	public static enum Feeder {
		Reverse(6), Halt(0), Forward(-6); // TODO: negative is currently forward, rework this to not be a hack

		public double voltage; // Stores the voltage commanded for that demand

		private Feeder(final double input) {
			this.voltage = input;
		}
	}
	
	@AutoLog
	public static class IntakeInputs {
		public boolean troughHasCoral = false;
		public AngularVelocity troughSpeed = Units.RotationsPerSecond.zero();
	}

	public final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

	private final TalonFXS trough;
	
	private final StatusSignal<ForwardLimitValue> troughSensor;
	private final StatusSignal<AngularVelocity> troughSpeed;

	public Intake(){
		trough = new TalonFXS(Constants.CAN.CTRE.troughWheels, Constants.CAN.CTRE.bus);
		// Set Neutral Mode
		trough.setNeutralMode(NeutralModeValue.Brake);

		// Create the Config Object for this TalonFXS
		TalonFXSConfiguration troughConfig = new TalonFXSConfiguration();

		// Motor Arrangement
		troughConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

		// Peak output amps
		troughConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		troughConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		// Supply current limits
		troughConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		troughConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		troughConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		troughConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit
		troughConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// Hardware Limit Switch Config
		troughConfig.HardwareLimitSwitch
			.withForwardLimitType(ForwardLimitTypeValue.NormallyClosed)
			.withForwardLimitEnable(false)
			.withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
			.withReverseLimitEnable(false);

		this.trough.getConfigurator().apply(troughConfig);

		this.troughSpeed = this.trough.getVelocity();
		this.troughSensor = this.trough.getForwardLimit();

		StatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(50), 
			troughSpeed,
			troughSensor
		);
	}

	public boolean holdingGamePeice() {
		return inputs.troughHasCoral;
	}

	private void runTrough(Feeder demand){
		this.trough.setControl(new VoltageOut(demand.voltage));
	}

	public void updateInputs(final IntakeInputs inputs) {
		BaseStatusSignal.refreshAll(this.troughSensor, this.troughSpeed);
		inputs.troughSpeed = this.troughSpeed.getValue();
	 	inputs.troughHasCoral = this.troughSensor.getValue().value != 0;
	}

	// Runs the trough until the command is interrupted, then stops the trough
	public Command runTrough() {
		return new RunCommand(() -> {
			runTrough(Feeder.Forward);
		}, this).finallyDo(
			() -> runTrough(Feeder.Halt)
		);
	}
	public Command runTroughBackwards() {
		return new RunCommand(() -> {
			runTrough(Feeder.Reverse);
		}, this).finallyDo(
			() -> runTrough(Feeder.Halt)
		);
	}
	

	// Runs the trough until the command is interrupted, then stops the trough
	public Command reverseTrough() {
		return new RunCommand(() -> {
			runTrough(Feeder.Reverse);
		}, this).finallyDo(
			() -> runTrough(Feeder.Halt)
		);
	}

	@Override
	public void periodic() {
		updateInputs(this.inputs);
		Logger.processInputs("Intake", this.inputs);
	}
}
