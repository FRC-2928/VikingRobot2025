package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.BananaFlywheels;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
	public final LoggedDashboardChooser<String> driveModeChooser;
	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;
	public final Drivetrain drivetrain;
	public final Climber climber;
	public final Elevator elevator;
	public final Intake intake;
	public final BananaFlywheels bananaFlywheels;
	
	public final AutoChooser autoChooser;

	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		this.diag = new Diagnostics();
		Tuning.intakeSpeed.get(); // load the class to put the tuning controls on the dashboard
		this.drivetrain = new Drivetrain();
		this.climber = new Climber();
		this.elevator = new Elevator();
		this.intake = new Intake();
		this.bananaFlywheels = new BananaFlywheels();

		autoChooser = Autonomous.getChoreoAutoChooser();
		SmartDashboard.putData("Autonomous Routine", autoChooser);
		RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

		this.driveModeChooser = new LoggedDashboardChooser<>(
			"Drive Mode",
			JoystickDrive.createDriveModeChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();
	}

	public Command scoreCoral(int level, Constants.ReefPosition position) {
		return new SequentialCommandGroup(
			// todo: add subcommands
		);
	}

	public String getDriveMode() { return this.driveModeChooser.get(); }
}
