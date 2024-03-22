package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFX;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
	public final LoggedDashboardChooser<Command> autonomousChooser;

	public final DriverOI driverOI = new DriverOI(new CommandXboxController(0));
	public final OperatorOI operatorOI = new OperatorOI(new CommandXboxController(1));

	public final Diagnostics diag;

	public final Drivetrain drivetrain;
	public final Shooter shooter;
	public final Climber climber;

	public final LimelightFX fx = new LimelightFX();
	public final LimelightFX.Module fxScreen = this.fx
		.module(LimelightFX.Module.Geometry.Grid24x12, LimelightFX.Module.Rotation.R0);
	public final LimelightFX.Module[] fxStrips = new LimelightFX.Module[] {
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),

		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.Strip16x1, LimelightFX.Module.Rotation.R0), };
	//public final LimelightFX.Behavior<?> fxStateIdle;

	public RobotContainer() {
		Robot.instance.container = this;
		Robot.cont = this;

		System.err.println("1");
		Tuning.flywheelVelocity.get(); // load the class to put the tuning controls on the dashboard
		System.err.println("2");

		this.diag = new Diagnostics();
		System.err.println("3");
		this.drivetrain = new Drivetrain();
		System.err.println("4");
		this.shooter = new Shooter();
		System.err.println("5");
		this.climber = new Climber();
		System.err.println("6");

		//this.fxStateIdle = this.fxScreen.behavior(LimelightFX.Behavior.ImageBehavior.class, 0);
		this.fx.initialize(SerialPort.Port.kUSB2);
		System.err.println("7");

		this.autonomousChooser = new LoggedDashboardChooser<>(
			"Autonomous Routine",
			AutonomousRoutines.createAutonomousChooser()
		);

		this.driverOI.configureControls();
		this.operatorOI.configureControls();

		this.diag.configureControls();
	}

	public Command getAutonomousCommand() { return this.autonomousChooser.get(); }
}
