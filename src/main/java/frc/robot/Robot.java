package frc.robot;

import java.lang.reflect.Field;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	public static Robot instance;
	public static RobotContainer cont;

	public RobotContainer container;

	private Command autonomousCommand;

	private double lastAutoCheck = 0;

	public Robot() {
		super();
		Robot.instance = this;
		Robot.cont = new RobotContainer();
	}

	@Override
	public void robotInit() {
		ConduitApi.getInstance().configurePowerDistribution(Constants.CAN.Misc.pdh, ModuleType.kRev.value);

		switch(Constants.mode) {
		case REAL -> {
			Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
			Logger.addDataReceiver(new NT4Publisher());
		}

		case SIM -> {
			Logger.addDataReceiver(new NT4Publisher());
		}

		case REPLAY -> {
			this.setUseTiming(false); // Run as fast as possible
			final String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		}

		Logger.start();

		this.container.diag.chirp(600, 500);
		this.container.diag.chirp(900, 500);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		LoggedPowerDistribution.getInstance(Constants.CAN.Misc.pdh, ModuleType.kRev).periodic();
	}

	// DISABLED //
	@Override
	public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void disabledPeriodic() {
		if(DriverStation.getMatchType() == MatchType.None && Timer.getFPGATimestamp() - this.lastAutoCheck >= 1) {
			boolean good;

			try {
				final Field field = LoggedDashboardChooser.class.getDeclaredField("selectedValue");
				field.setAccessible(true);
				String name = (String) field.get(Robot.cont.autonomousChooser);
				if(name == null) name = "<none>";
				good = !name.contains("[comp]");
			} catch(final Exception e) {
				throw new Error(e);
			}

			if(good) {
				System.err.println("CRITICAL: CURRENT AUTONOMOUS ROUTINE IS NOT SUITED FOR COMPETITION");
				this.container.diag.chirp(2000, 100);
				this.container.diag.chirp(1000, 100);
			}

			Logger.recordOutput("Checks/AutonomousRoutineGood", good);

			this.lastAutoCheck = Timer.getFPGATimestamp();
		}
	}

	@Override
	public void disabledExit() {}

	// AUTONOMOUS //

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().cancelAll();

		// Get selected routine from the dashboard
		this.autonomousCommand = this.container.getAutonomousCommand();

		// schedule the autonomous command (example)
		if(this.autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
