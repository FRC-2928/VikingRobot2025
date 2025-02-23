package frc.robot;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
	public static Robot instance;
	public static RobotContainer cont;
	public static Command commandToRun;
	public static boolean needToLookOtherWay;

	public RobotContainer container;

	private Command autonomousCommand;

	public Robot() {
		super();

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

		Robot.instance = this;
		Robot.cont = new RobotContainer();

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotInit() {
		PathfindingCommand.warmupCommand().schedule();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		LoggedPowerDistribution.getInstance(Constants.CAN.Misc.pdh, ModuleType.kRev);
		Logger.recordOutput("ControllerInputs/ReefHeight", cont.driverOI.targetScoringLevel);
		
	}

	// DISABLED //
	@Override
	public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }

	@Override
	public void disabledPeriodic() {}

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
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {}

	// TELEOP //

	@Override
	public void teleopInit() {
		CommandScheduler.getInstance().cancelAll();

		this.container.drivetrain.setDefaultCommand(this.container.drivetrain.joystickDrive);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	// TEST //

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();

		this.container.drivetrain.setDefaultCommand(this.container.drivetrain.joystickDrive);
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
