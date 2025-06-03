// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GamePieceType;
import frc.robot.commands.drivetrain.CenterLimelight;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class Superstate extends SubsystemBase {
	public enum RobotStates {
		Drive,
		Intake,
		AutoAlignCoral,
		ManualAlignCoral,
		ScoreCoral,
		UnscoreAlgae;

		public final Trigger isCurrentState;

		private RobotStates() {
			this.isCurrentState = new Trigger(() -> Superstate.globalState == this);
		}
	}

	// Note from Casey: No need to have 2 separate enums for current state and wanted state, they will be the same thing.
	// We can store 2 separate class variables for current and wanted
	// public enum WantedRobotStates {
	// 	Drive,
	// 	Intake,
	// 	AutoAlignCoral,
	// 	ManualAlignCoral,
	// 	ScoreCoral,
	// 	UnscoreAlgae
	// }

	private RobotStates handleStateTransition() {
		previousRobotState = globalState; // What if globalState doesn't get updated in the switch block? Should previousRobotState still update?
		switch (wantedGlobalState) {
			case Intake:
				globalState = RobotStates.Intake;	
				break;
			
			case AutoAlignCoral:
				globalState = RobotStates.AutoAlignCoral;
				break;
			
			case ManualAlignCoral:
				globalState = RobotStates.ManualAlignCoral;
				break;	
			
			case ScoreCoral:
				if (!(RobotContainer.getInstance().driverOI.alignReefLeft.getAsBoolean() || (RobotContainer.getInstance().driverOI.alignReefRight.getAsBoolean()))) {
					globalState = RobotStates.ScoreCoral;
					 // This may be a bug. This break won't happen if the conditional fails, and will fall-through to UnscoreAlgae. Consider putting break outside the if block.
				}
				break;
			
			case UnscoreAlgae:
				if (RobotContainer.getInstance().driverOI.closeToReef.getAsBoolean()) {
					globalState = RobotStates.UnscoreAlgae;
					 // This may be a bug. This break won't happen if the conditional fails, and will fall-through to default
				}
				break;
			default:{
				break;
			}
		}
		return globalState;
	}

	// private void applyStates() {
	// 	// We need to perform actions each frame based on the state + other triggering conditions (such as sensor input or controller button presses).
	// 	// One way to do that is like below to run periodic code that tells the robot what to do each frame.
	// 	// However, we ran into this same issue earlier this season, that behavior can get fairly complex where a single state requires sequential, parallel, or conditional function.
	// 	// To solve this we can use the existing command framework. Create a command (factory) that runs while we are in a particular state and/or button combo.
	// 	// We could then run a function from handleStateTransition() that schedules the command at the start of a state and unschedules when we leave the state.
	// 	// I think it's easier to instead create Triggers such as the one below this method.
	// 	switch(globalState) {
	// 		case Intake:{
	// 			intake();
	// 			break;
	// 		}
	// 		case AutoAlignCoral:{
	// 			autoAlignCoral();
	// 			break;
	// 		}
	// 		default:
	// 			break;
	// 		}

	// }

	// This sample trigger binds the autoAlignCoral() command to the condition where the robot state is AutoAlignCoral.
	// Every state can have a command that runs while we are in that state in order to control robot functions.
	public Superstate() {
		RobotStates.Drive.isCurrentState.whileTrue(intake());
		RobotStates.Intake.isCurrentState.whileTrue(intake());
		RobotStates.AutoAlignCoral.isCurrentState.whileTrue(autoAlignCoral());
		RobotStates.ManualAlignCoral.isCurrentState.whileTrue(intake());
		RobotStates.ScoreCoral.isCurrentState.whileTrue(intake());
		RobotStates.UnscoreAlgae.isCurrentState.whileTrue(intake());
	}

	// NOTE: Now is a great time to focus on keeping code organized. I recommend putting all global/class variable declarations before any methods

	//Global Variables
	public boolean coralInEndEffector;
	public boolean limelightToleranceMet;
	public static RobotStates globalState = RobotStates.Drive;
	public static RobotStates wantedGlobalState = RobotStates.Drive;
	public RobotStates previousRobotState;
	//State Triggers
	// public Trigger isDrive = new Trigger(() -> (globalState == RobotStates.Drive));
	// public Trigger isIntake = new Trigger(() -> (globalState == RobotStates.Intake));
	// public Trigger isautoAlignCoral = new Trigger(() -> (globalState == RobotStates.autoAlignCoral));
	// public Trigger ismaunualAlignCoral = new Trigger(() -> (globalState == RobotStates.manualAlignCoral));
	// public Trigger isScorecoral =  new Trigger(() -> (globalState == RobotStates.Scorecoral));
	// public Trigger isunscoreAlgaie = new Trigger(() -> (globalState == RobotStates.unscoreAlgaie));
	public void periodic() {
		globalState = handleStateTransition();
		Logger.recordOutput("StateMachine/DesiredSuperstate", wantedGlobalState);
        Logger.recordOutput("StateMachine/CurrentSuperstate", globalState);
	}

	@Override
	public void simulationPeriodic() {
		periodic();
	}

	public void setWantedSuperState(RobotStates wantedSuperState) {
        Superstate.wantedGlobalState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(RobotStates wantedSuperState) {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }

	public Command intake() {
		return new RunCommand((() -> {
			// TODO: Implementation
			System.out.println("State is Intake");
			// do some stuff
			// ...

			// Transition to next state when the button is not pressed
			if (!RobotContainer.getInstance().driverOI.alignReefRight.getAsBoolean()) { // For now, using right bumper for intake
				setWantedSuperState(RobotStates.Drive);
			}
		}));
		
	}
	public Command driveCommand() {
		return new InstantCommand();
	}

	public Command autoAlignCoral() {
		return new ParallelCommandGroup( new ConditionalCommand(CenterLimelight.centerLimelightLeft(), CenterLimelight.centerLimelightRight(), RobotContainer.getInstance().driverOI.alignReefLeft)
					.andThen(setWantedSuperStateCommand(RobotStates.ManualAlignCoral)),
					//Set state to drive if bumpers arent pressed
					new RunCommand(() -> {if(!(RobotContainer.getInstance().driverOI.alignReefLeft.getAsBoolean() || (RobotContainer.getInstance().driverOI.alignReefRight.getAsBoolean()))){
						setWantedSuperState(RobotStates.Drive);
					}})
				);
	}

	public Command manualAlignCoral() {
		return new ParallelCommandGroup(
				setWantedSuperStateCommand(RobotStates.ScoreCoral),
				RobotContainer.getInstance().elevator.goToGamePieceHeight(GamePieceType.CORAL),
				RobotContainer.getInstance().drivetrain.dPadMode()
			);
	}
	// public Command manualAlignCoral(){
	// 	return
	// }
	
}
