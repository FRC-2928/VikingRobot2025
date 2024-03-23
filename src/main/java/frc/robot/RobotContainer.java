package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
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

	public final SerialPort fxSerial;
	public final LimelightFX fx = new LimelightFX();
	public final LimelightFX.Module fxScreen = this.fx
		.module(LimelightFX.Module.Geometry.grid, LimelightFX.Module.Rotation.R0);
	public final LimelightFX.Module[] fxStrips = new LimelightFX.Module[] {
		// we have 8 strips, however since the strips are connected
		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0),

		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0),
		this.fx.module(LimelightFX.Module.Geometry.strip.size(32, 1), LimelightFX.Module.Rotation.R0), };
	public final LimelightFX.Behavior<?> fxStateTest;

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

		this.fxStateTest = this.fxScreen.behavior(LimelightFX.Behavior.BlinkBehavior.class, 0);
		this.fx.selector(() -> {
			if(this.shooter.inputs.holdingNote) return this.fxStateTest;
			else return null;
		});

		this.fxSerial = new SerialPort(115200, SerialPort.Port.kUSB1);
		this.fxSerial.reset();
		this.fxSerial.setTimeout(1);
		this.fxSerial.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);

		this.fx.initialize(str -> this.fxSerial.writeString(str) != 0);
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
