package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
//  Kracken x60 for deploy retract w/ encoder
//  Kracken x60 for motors (all one)
// Minion for Belt
// Minion for trough and limit switch
public class Intake extends SubsystemBase {
	public static enum Feeder {
		Reverse(-1), Halt(0), Forward(1);

		public double demand;

		private Feeder(final double input) {
			this.demand = input;
		}
	}
	@AutoLog
	public static class IntakeInputs {
		public boolean troughHasCoral;
		public AngularVelocity intakeSpeed;
		public Angle pivotAngle;
		public AngularVelocity troughSpeed;
	}


	private final TalonFX wheels;
	private final TalonFXS belt;
	private final TalonFX pivot;
	private final TalonFXS trough;
	private final SensorCollection sensors;
	private final StatusSignal<AngularVelocity> intakeSpeed;
	private final StatusSignal<Angle> pivotAngle;
	private final StatusSignal<AngularVelocity> troughSpeed;
	private TalonFXConfiguration intakeWheelsConfig = new TalonFXConfiguration();
	private TalonFXConfiguration pivotWheelConfig = new TalonFXConfiguration();
	private TalonFXSConfiguration beltConfig = new TalonFXSConfiguration();
	public Intake(){
		this.wheels = new TalonFX(Constants.CAN.CTRE.intakeWheels, Constants.CAN.CTRE.bus);
		this.belt = new TalonFXS(Constants.CAN.CTRE.intakeBelt, Constants.CAN.CTRE.bus);
		this.pivot = new TalonFX(Constants.CAN.CTRE.intakePivot, Constants.CAN.CTRE.bus);
		this.trough = new TalonFXS(Constants.CAN.CTRE.troughWheels, Constants.CAN.CTRE.bus);
		this.intakeSpeed = this.wheels.getRotorVelocity();
		this.pivotAngle = this.pivot.getRotorPosition();
		this.troughSpeed = this.trough.getVelocity();
		this.sensors = null;
		

		// Peak output amps
		beltConfig.CurrentLimits.StatorCurrentLimit = 
		intakeWheelsConfig.CurrentLimits.StatorCurrentLimit = 80.0;
		intakeWheelsConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeWheelsConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		intakeWheelsConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
		// Supply current limits
		intakeWheelsConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		intakeWheelsConfig.CurrentLimits.SupplyCurrentLimit = 60;  	 // max current draw allowed
		intakeWheelsConfig.CurrentLimits.SupplyCurrentLowerLimit = 35;  // current allowed *after* the supply current limit is reached
		intakeWheelsConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;  // max time allowed to draw SupplyCurrentLimit
		intakeWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
		intakeWheelsConfig.Audio = Constants.talonFXAudio;
		pivotWheelConfig = intakeWheelsConfig;
		pivotWheelConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		pivotWheelConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.Intake.max.in(Units.Rotations);
		pivotWheelConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		pivotWheelConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.Intake.pivotMin.in(Units.Rotations);
		// PID values
		pivotWheelConfig.Slot0 =  Constants.Intake.pivotConfig;
		intakeWheelsConfig.Slot0 = Constants.Intake.flywheelGainsSlot0;

		this.wheels.getConfigurator().apply(intakeWheelsConfig);
		this.wheels.setNeutralMode(NeutralModeValue.Brake);
		this.pivot.getConfigurator().apply(pivotWheelConfig);
		this.pivot.setNeutralMode(NeutralModeValue.Brake);
	}
	
	public void runIntake(final Feeder demand){
		this.wheels.setControl(new VelocityDutyCycle(demand.demand*0.1));

	}
	private boolean holdingGamePeice() {
		return false;
	}
	private void runTrough(Feeder demand){
		this.trough.setControl(new VelocityDutyCycle(demand.demand));
	}
	private void runBelt(Feeder demand){
		this.belt.setControl(new VelocityDutyCycle(demand.demand));
	}
	private void rotatePivot(final Angle target) {
		final Angle rot = Units.Radians
			.of(
				MathUtil
					.clamp(
						target.in(Units.Radians),
						0,
						1
					)
			);
		Logger.recordOutput("intake/PivotControl", rot);
		this.pivot.setControl(new PositionDutyCycle(rot.in(Units.Rotations)));
	}
	public void updateInputs(final IntakeInputs inputs) {
		BaseStatusSignal.refreshAll(this.intakeSpeed, this.pivotAngle, this.troughSpeed);
		inputs.intakeSpeed = this.intakeSpeed.getValue();
		inputs.pivotAngle = this.pivotAngle.getValue();
		inputs.troughSpeed = this.troughSpeed.getValue();
	 	inputs.troughHasCoral = !this.sensors.isFwdLimitSwitchClosed();
	}

	public Command IntakeUp() {
		return new RunCommand(() -> {
			rotatePivot(Constants.Intake.max);
		}, this);
	}

	public Command IntakeGround(){
		return new SequentialCommandGroup(
			new RunCommand(() -> {
				rotatePivot(Units.Radians.of(0));
			}, this),
			Commands.deadline(
				new RunCommand(() -> {
					runIntake(Feeder.Forward);
				},this).until(this::holdingGamePeice),
				new RunCommand(() -> {
					runBelt(Feeder.Forward);
				},this),
				new RunCommand(() -> {
					runTrough(Feeder.Reverse);
				},this)
			),
			new RunCommand(() -> {
				rotatePivot(Units.Radians.of(1));
			}, this)
		);
	}
}
