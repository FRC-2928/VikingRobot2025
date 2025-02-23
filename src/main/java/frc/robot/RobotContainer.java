package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.BananaFlywheels;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightFXManager;

public class RobotContainer {
	public final LoggedDashboardChooser<Command> autonomousChooser;
	public final LoggedDashboardChooser<String> driveModeChooser;
	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;

	public final Drivetrain drivetrain;
	public final Climber climber;
	public final Elevator elevator;
	public final Intake intake;
	public final BananaFlywheels bananaFlywheels;

	public final LimelightFXManager fxm;

	public static boolean ledState = false;
	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		this.diag = new Diagnostics();
		this.drivetrain = new Drivetrain();
		this.climber = new Climber();
		this.fxm = new LimelightFXManager();
		this.elevator = new Elevator();
		this.intake = new Intake();
		this.bananaFlywheels = new BananaFlywheels();

		this.diag.chirp(600, 500);
		this.diag.chirp(900, 500);

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			Autonomous.createAutonomousChooser()
		);
		this.driveModeChooser = new LoggedDashboardChooser<>(
			"Drive Mode",
			JoystickDrive.createDriveModeChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();

		this.diag.configureControls();
	}

	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }

	public Command scoreCoral(int level, Constants.ReefPosition position) {
		return new SequentialCommandGroup(

		);
	}

	public String getDriveMode() { return this.driveModeChooser.get(); }
}
