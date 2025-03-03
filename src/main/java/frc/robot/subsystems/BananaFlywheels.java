package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Banana.FeederDemand;

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
	@AutoLog
	public static class BananaFlywheelsInputs {
		public boolean holdingCoral = false;
		public Angle flywheelPosition = Units.Rotations.zero();
		public AngularVelocity flywheelSpeed = Units.RotationsPerSecond.zero();
		public Current flywheelStatorCurrent = Units.Amps.zero();
		public Current flywheelSupplyCurrent = Units.Amps.zero();
	}

	private final BananaFlywheelsInputsAutoLogged inputs = new BananaFlywheelsInputsAutoLogged();
	private final TalonFX wheels;
	private final StatusSignal<Angle> angle;
	private final StatusSignal<AngularVelocity> velocity;
	private final StatusSignal<Current> motorStatorCurrent;
	private final StatusSignal<Current> motorSupplyCurrent;
	// TODO: incorporate CANdi read of limit switch
	private TalonFXConfiguration flyWheelConfig = new TalonFXConfiguration();

	/**
	 * Default Constructor
	 */
	public BananaFlywheels() {
		this.wheels = new TalonFX(Constants.CAN.RIO.bananaWheels, Constants.CAN.RIO.bus);
		this.angle = this.wheels.getRotorPosition();
		this.velocity = this.wheels.getRotorVelocity();
		this.motorStatorCurrent = this.wheels.getStatorCurrent();
		this.motorSupplyCurrent = this.wheels.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(Units.Hertz.of(100),
			this.angle,
			this.velocity,
			this.motorStatorCurrent,
			this.motorSupplyCurrent);
		this.wheels.optimizeBusUtilization();

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

	@Override
	public void periodic() {
		this.updateInputs(this.inputs);
		Logger.processInputs("BananaFlywheels", this.inputs);
	}
	
	public void runFlywheels(FeederDemand demand) {
		this.wheels.setControl(new VoltageOut(demand.getValue()*6.0));
	}

	//Gets angle of flywheel and holds it at that position
	//Will add the holding coral input once we know which sensor detects the coral
	public void holdPosition() {
		Angle wheelAngle = this.angle.getValue();
		this.wheels.setControl(new PositionVoltage(wheelAngle));
	}

	public boolean holdingCoral() {
		return true;
	}

	private void updateInputs(final BananaFlywheelsInputs inputs) {
		BaseStatusSignal.refreshAll(
			this.angle, this.velocity, this.motorStatorCurrent, this.motorSupplyCurrent);

		inputs.flywheelSpeed = this.velocity.getValue();
		inputs.flywheelSupplyCurrent = this.motorStatorCurrent.getValue();
		inputs.flywheelStatorCurrent = this.motorSupplyCurrent.getValue();
		// inputs.holdingCoral = !this.sensors.isFwdLimitSwitchClosed();
	}

	/**
	 * Creates a command that will output the coral, then stop when it's out of the banana.
	 * This factory should be used for autonomous or for fully automating coral scoring.
	 * @return a command to score the coral
	 */
	public Command scoreHeldCoral() {
		return new RunCommand(() -> {
			runFlywheels(FeederDemand.FORWARD);
		}, this).until(() -> !this.holdingCoral())
		.andThen(
			new RunCommand(() -> {
				runFlywheels(FeederDemand.FORWARD);
			}, this).withTimeout(0.2)
		).andThen(
			new RunCommand(() -> {
				runFlywheels(FeederDemand.HALT);
			},this)
		);
		// 1. Run the wheels forward until the sensor returns false
		// 2. Run the wheels another 0.2 seconds
		// 3. Stop the wheels
	}

	public Command outputForward(){
		return new RunCommand(() -> {
			runFlywheels(FeederDemand.FORWARD);
		}, this).finallyDo(() -> {
			runFlywheels(FeederDemand.HALT);
		});
	}

	public Command acceptAlgae(){
		return new RunCommand(() -> {
			runFlywheels(FeederDemand.REVERSE);
		}, this).finallyDo(this::holdPosition);
	}
}
