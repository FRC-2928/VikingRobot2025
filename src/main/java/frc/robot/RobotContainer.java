package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import choreo.auto.AutoChooser;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AlgaePosition;
import frc.robot.Constants.CoralPosition;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.ReefPosition;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.oi.DriverOI;
import frc.robot.oi.OperatorOI;
import frc.robot.subsystems.BananaFlywheels;
import frc.robot.subsystems.Diagnostics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class RobotContainer {
	public LoggedDashboardChooser<String> driveModeChooser;
	public DriverOI driverOI;
	public OperatorOI operatorOI;

	public Diagnostics diag;
	public Drivetrain drivetrain;
	public Elevator elevator;
	public Intake intake;
	public BananaFlywheels bananaFlywheels;
	
	public AutoChooser autoChooser;
	private static RobotContainer sInstance = null;

	public static synchronized RobotContainer getInstance() {
		if (sInstance != null) {
			return sInstance;
		}

		sInstance = new RobotContainer();
		sInstance.init();
		return sInstance;
	}

	private RobotContainer() {
		// Empty
	}

	private void init() {
		this.driverOI = new DriverOI(new CommandXboxController(0));
		this.operatorOI = new OperatorOI(new CommandXboxController(1));
		Tuning.algaePivotHome.get();
		this.diag = new Diagnostics();
		Tuning.intakeSpeed.get(); // load the class to put the tuning controls on the dashboard
		this.drivetrain = new Drivetrain();
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

	public Command autoScoreCoral(ReefPosition reefPos) {
		return new SequentialCommandGroup(
			CenterLimelight.centerLimeLightPosition(reefPos).alongWith(
				new InstantCommand(() -> elevator.setTargetCoralLevel(CoralPosition.L4))),
			this.elevator.goToReefHeight(GamePieceType.CORAL),
			this.elevator.goToGamePieceHeight(GamePieceType.CORAL).withTimeout(0.4),
			new ParallelDeadlineGroup(
				this.bananaFlywheels.scoreHeldCoral(), 
				this.elevator.goToGamePieceHeight(GamePieceType.CORAL)
			)
		).finallyDo(() -> {
        this.elevator.onEjectCoral();
        this.elevator.setTargetCoralLevel(CoralPosition.NONE);
      });
	}

	public Command raiseElevatorAtReef() {
		return new ConditionalCommand(
			new ParallelCommandGroup(
				this.elevator.goToGamePieceHeight(GamePieceType.CORAL), 
				this.drivetrain.dPadMode()
			),
			new InstantCommand(), 
			this.driverOI.closeToReef)
		.until(this.driverOI.closeToReef.negate());
	}

	public Command telePositionForCoralLeft() {
		return CenterLimelight.centerLimelightLeft();
	}
	public Command telePositionForCoralOveride() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.CORAL),
			drivetrain.dPadMode()
		);
	}

	public Command telePositionForAlgaeOverride() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
			this.bananaFlywheels.outputForward(),
			drivetrain.dPadMode()
		);
	}

	public Command telePositionForCoralRight() {
		return new SequentialCommandGroup(
			CenterLimelight.centerLimelightRight(),
			new ParallelCommandGroup(
				this.elevator.goToGamePieceHeight(GamePieceType.CORAL),
				drivetrain.dPadMode()
			)
		);
	}


	public Command telePositionForAlgae() {
		return new SequentialCommandGroup(
			CenterLimelight.centerLimelightCenter(),
			new ConditionalCommand(
				this.bananaFlywheels.outputForward().withTimeout(5), 
				new InstantCommand(), 
				(() -> (this.bananaFlywheels.holdingCoral() && this.elevator.hasCurrentGamePieceType(GamePieceType.NONE)))),
			this.elevator.setTargetAlgaeLevelCommand(RobotContainer.getInstance().drivetrain.getAlgaeHeight()),
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					
					this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
					this.bananaFlywheels.outputForward()
				),
				drivetrain.dPadMode()
			)
		);
	}

	public Command telePositionForAlgaeOverideL2() {
		return new SequentialCommandGroup(
			this.elevator.setTargetAlgaeLevelCommand(AlgaePosition.L2),
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
					this.bananaFlywheels.outputForward()
				),
				drivetrain.dPadMode()
			)
		);
	}
	
	public Command telePositionForAlgaeOverideL3() {
		return new SequentialCommandGroup(
			this.elevator.setTargetAlgaeLevelCommand(AlgaePosition.L3),
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
					this.bananaFlywheels.outputForward()
				),
				drivetrain.dPadMode()
			)
		);
	}


	public Command pullAlgaeOffReef() {
		return new ParallelCommandGroup(
			this.elevator.goToGamePieceHeight(GamePieceType.ALGAE),
			drivetrain.slowMode()
		).until(() -> 
			this.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.blueReefCenter) > Tuning.reefBackupWithAlgaeRadius.get() 
			&& this.drivetrain.getEstimatedPosition().getTranslation().getDistance(Constants.redReefCenter) > Tuning.reefBackupWithAlgaeRadius.get()
		);
	}

	public Command troughHandoffManual(){
		return new ParallelCommandGroup(
			this.bananaFlywheels.intakeForward(),
			this.intake.runTrough()
		);
  }

	public Command troughHandoffAutomated(){
		return new SequentialCommandGroup(
			// TODO: this doesn't work right -- trough doesn't run even when limit not tripped
			// TODO: need to override the limit switches in Intake and Banana when we want to outtake
			Commands.deadline(
				new SequentialCommandGroup(
					this.bananaFlywheels.outputForward().withTimeout(0.5),
					new InstantCommand(() -> {elevator.onEjectCoral();}),
					new RunCommand(() -> {}).until(elevator::isInTargetPos)
				),
				this.intake.runTroughBackwards()
			),
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					this.intake.runTrough(), // protection to ensure we finish staging the piece
					this.bananaFlywheels.outputForward()
				).until(bananaFlywheels::holdingCoral),
				
				Commands.deadline(
					this.bananaFlywheels.rotateBanana(Units.Rotations.of(Tuning.intakeBananaFlywheelsRotations.get())),
					this.intake.runTrough()
					
				)
			)
		);
	}

	public Command reverseTrough() {
		return this.intake.reverseTrough();
	}

	public String getDriveMode() { return this.driveModeChooser.get(); }
}
