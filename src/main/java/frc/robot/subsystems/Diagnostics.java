package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController.RadioLEDState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.Alert;
import frc.robot.utils.STalonFX;

public class Diagnostics extends SubsystemBase {
	public final class Release extends Command {
		public Release() { this.addRequirements(Robot.cont.drivetrain, Robot.cont.shooter, Robot.cont.climber); }

		@Override
		public void initialize() {
			for(final SwerveModule module : Robot.cont.drivetrain.modules) {
				final ModuleIOReal io = (ModuleIOReal) module.io;
				io.azimuth.setNeutralMode(NeutralModeValue.Coast);
				io.drive.setNeutralMode(NeutralModeValue.Coast);
			}

			((ShooterIOReal) Robot.cont.shooter.io).pivot.setNeutralMode(NeutralModeValue.Coast);
			final ClimberIOReal climber = ((ClimberIOReal) Robot.cont.climber.io);
			climber.actuator.setNeutralMode(NeutralModeValue.Coast);
			//climber.lock.set(0);

			Diagnostics.this.chirp(1000, 100);
			Diagnostics.this.chirp(800, 500);
		}

		@Override
		public void end(final boolean interrupted) {
			for(final SwerveModule module : Robot.cont.drivetrain.modules) {
				final ModuleIOReal io = (ModuleIOReal) module.io;
				io.azimuth.setNeutralMode(NeutralModeValue.Brake);
				io.drive.setNeutralMode(NeutralModeValue.Brake);
			}

			((ShooterIOReal) Robot.cont.shooter.io).pivot.setNeutralMode(NeutralModeValue.Brake);
			final ClimberIOReal climber = ((ClimberIOReal) Robot.cont.climber.io);
			climber.actuator.setNeutralMode(NeutralModeValue.Brake);
			//climber.lock.set(0);

			Diagnostics.this.chirp(800, 100);
			Diagnostics.this.chirp(1000, 500);
		}

		@Override
		public boolean runsWhenDisabled() { return true; }
	}

	private static final class Chirp {
		public Chirp(final int freq, final int ms) {
			this.freq = freq;
			this.us = ms * 1000;
		}

		public int freq;
		public long us;
		public long start = 0;
	}

	public Diagnostics() {
		try {
			this.loggedDashboardChooserSelectedValue = LoggedDashboardChooser.class.getDeclaredField("selectedValue");
			this.loggedDashboardChooserSelectedValue.setAccessible(true);
		} catch(final Exception e) {
			throw new Error(e);
		}
	}

	public final ArrayList<STalonFX> motors = new ArrayList<>();

	private final ArrayList<Chirp> chirps = new ArrayList<>();
	private final Field loggedDashboardChooserSelectedValue;
	private double lastCheck = 0;
	private final Alert invalidAutoRoutine = new Alert(
		"Autonomous Routine '<none>' is not ready for competition",
		Alert.AlertType.ERROR
	);
	private final Alert setupJumperPresent = new Alert("Setup Jumper is plugged in", Alert.AlertType.ERROR);
	private final Alert startingConfiguration = new Alert("Not in Starting Configuration", Alert.AlertType.ERROR);
	private final Alert badVoltage = new Alert(
		"Battery Voltage is 0V, recommended to be 12.5V",
		Alert.AlertType.WARNING
	);

	public final DigitalInput releaseInput = new DigitalInput(0);
	public final Trigger release = new Trigger(() -> DriverStation.isDisabled() && !this.releaseInput.get());

	public void chirp(final boolean good) { this.chirps.add(new Chirp(good ? 500 : 125, 500)); }

	public void chirp(final int freq, final int ms) { this.chirps.add(new Chirp(freq, ms)); }

	public void configureControls() {
		this.release.whileTrue(this.new Release());

		SmartDashboard
			.putData("Diagnostics/C-Stop", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
		SmartDashboard
			.putData("Diagnostics/C-Stop", new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Diagnostics/Release", this.release.getAsBoolean());

		Logger.recordOutput("Diagnostics/Competition", DriverStation.isFMSAttached());

		if(
			(DriverStation.isFMSAttached() || true)
				&& DriverStation.isDisabled()
				&& Timer.getFPGATimestamp() - this.lastCheck >= 1
		) {
			final boolean invalidAutoRoutine;

			String name;
			try {
				name = (String) this.loggedDashboardChooserSelectedValue.get(Robot.cont.autonomousChooser);
				if(name == null) name = "<none>";
				invalidAutoRoutine = !name.contains("[comp]");
			} catch(final Exception e) {
				throw new Error(e);
			}

			final boolean setupJumperPresent = this.release.getAsBoolean();
			final double startingConfigurationAngleDifference = Math
				.abs(Robot.cont.shooter.inputs.angle.minus(Constants.Shooter.startingConfiguration).in(Units.Degrees));
			final boolean startingConfiguration = startingConfigurationAngleDifference > 6;

			this.invalidAutoRoutine.setText("Autonomous routine '" + name + "' not ready for competition!");
			this.invalidAutoRoutine.set(invalidAutoRoutine);
			this.setupJumperPresent.set(setupJumperPresent);
			this.startingConfiguration
				.setText("Not in starting configuration (" + startingConfigurationAngleDifference + "deg off)");
			this.startingConfiguration.set(startingConfiguration);

			final boolean badVoltage = RobotController.getBatteryVoltage() < 12.3;

			this.badVoltage
				.setText(
					"Battery Voltage is " + RobotController.getBatteryVoltage() + "V, recommended to be at least 12.3V"
				);
			this.badVoltage.set(badVoltage);

			final boolean ready = !(invalidAutoRoutine || setupJumperPresent || startingConfiguration);

			Logger.recordOutput("Diagnostics/ReadyForCompetition", ready);
			RobotController.setRadioLEDState(ready ? RadioLEDState.kGreen : RadioLEDState.kRed);

			if(ready) {
				this.chirp(2000, 100);
				this.chirp(1000, 500);
			}

			this.lastCheck = Timer.getFPGATimestamp();
		}

		if(this.chirps.size() > 0) {
			final Chirp chirp = this.chirps.get(0);
			if(chirp.start == 0) chirp.start = Logger.getRealTimestamp();

			for(final TalonFX fx : this.motors)
				fx.setControl(new MusicTone(chirp.freq));

			if(Logger.getRealTimestamp() - chirp.start >= chirp.us) {
				for(final TalonFX fx : this.motors)
					fx.setControl(new MusicTone(0));

				this.chirps.remove(0);
			}
		}
	}
}
