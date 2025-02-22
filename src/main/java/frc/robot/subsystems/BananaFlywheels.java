package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/* 1.Make a method that stops elevator pivot from going lower a certain angle
 * 2. Run fly wheels forward and backward TalonFX
 * 3. Detect coral in banana - Kraken x44, limit switch
 * 	Start spinning flywheel when preparing intake from trough
 * 	Limit switches (detect presence or absence of): elevator can move with coral, coral in trough
 * States: 
 * true, true = coral being intook into banana
 * false, false = no coral in trough or banana
 * true, false = coral in banana
 * false, true = coral in trough
 * if banana doesn't have coral than banana shouldn't move past a certain angle/height
 * 
*/
public class BananaFlywheels extends SubsystemBase {
	public static enum Feeder {
		Reverse(-1), Halt(0), Forward(1);

		public double demand;

		private Feeder(final double input) {
			this.demand = input;
		}

	}
	@AutoLog
	public static class BananaFlywheelsInputs {
		public boolean holdingCoral;
		public AngularVelocity flywheelSpeed;
	}


	private final TalonFX wheels;
	private final StatusSignal<Angle> angle;
	private final StatusSignal<AngularVelocity> velocity;
	private TalonFXConfiguration flyWheelConfig = new TalonFXConfiguration();

	public BananaFlywheels(){
		this.wheels = new TalonFX(Constants.CAN.CTRE.bananaWheels, Constants.CAN.CTRE.bus);
		this.angle = this.wheels.getRotorPosition();
		this.velocity = this.wheels.getRotorVelocity();

		// Peak output amps
		flyWheelConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		flyWheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		flyWheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		flyWheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		// Supply current limits
		flyWheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		flyWheelConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		flyWheelConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		flyWheelConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit

		flyWheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;

		// PID values
		flyWheelConfig.Slot0 = Constants.Banana.flywheelGainsSlot0;

		flyWheelConfig.Audio = Constants.talonFXAudio;

		this.wheels.getConfigurator().apply(flyWheelConfig);
		this.wheels.setNeutralMode(NeutralModeValue.Brake);
	}
	
	public void runFlywheels(final Feeder demand){
		this.wheels.setControl(new VelocityDutyCycle(demand.demand*0.1));
	}
//Gets angle of flywheel and holds it at that position
//Will add the holding coral input once we know which sensor detects the coral
	public void holdPosition(){
		Angle wheelAngle = this.angle.getValue();
		this.wheels.setControl(new PositionVoltage(wheelAngle));


	}
	private boolean holdingCoral() {
		return false;
	}
	public void updateInputs(final BananaFlywheelsInputs inputs) {
		BaseStatusSignal.refreshAll(this.angle, this.velocity);

		inputs.flywheelSpeed = this.velocity.getValue();
	//	inputs.holdingCoral = !this.sensors.isFwdLimitSwitchClosed();
	}

	public Command scoreHeldCoral() {
		return new RunCommand(() -> {
			runFlywheels(Feeder.Forward);
		}, this).until(() -> !this.holdingCoral())
		.andThen(
			new RunCommand(() -> {
				runFlywheels(Feeder.Forward);
			}, this).withTimeout(0.2)
		).andThen(
			new RunCommand(() -> {
				runFlywheels(Feeder.Halt);
			},this)
		);
		// 1. Run the wheels forward until the sensor returns false
		// 2. Run the wheels another 0.2 seconds
		// 3. Stop the wheels
	}
	public Command rejectCoral(){
		return new RunCommand(() -> {
			runFlywheels(Feeder.Forward);
		}, this);
	}

	public Command acceptAlgae(){
		return new RunCommand(() -> {
			runFlywheels(Feeder.Reverse);
		}, this).finallyDo(
			() -> {
				holdPosition();
			}
		);		
	}
}
