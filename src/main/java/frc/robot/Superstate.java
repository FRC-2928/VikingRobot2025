// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GamePieceType;
import frc.robot.commands.drivetrain.CenterLimelight;

/** Add your docs here. */
public class Superstate extends SubsystemBase {
	public enum RobotStates{
		Drive,
		Intake,
		autoAlignCoral,
		manualAlignCoral,
		Scorecoral,
		unscoreAlgae
	}
	public enum WantedRobotStates{
		Drive,
		Intake,
		autoAlignCoral,
		manualAlignCoral,
		Scorecoral,
		unscoreAlgae
	}
	private RobotStates handleStateTransition(){
		previousRobotState = globalState;
		switch (wantedGlobalState){
			case Intake:{
				globalState = RobotStates.Intake;	
				break;
			}
			case autoAlignCoral:{
				globalState = RobotStates.autoAlignCoral;
				break;
			}
			case manualAlignCoral:{
				globalState = RobotStates.manualAlignCoral;
				break;	
			}
			case Scorecoral:{
				if(!(RobotContainer.getInstance().driverOI.alignReefLeft.getAsBoolean() || (RobotContainer.getInstance().driverOI.alignReefRight.getAsBoolean()))){
					globalState = RobotStates.Scorecoral;
					break;
				}
			}
			case unscoreAlgae:{
				if(RobotContainer.getInstance().driverOI.closeToReef.getAsBoolean()){
					globalState = RobotStates.unscoreAlgae;
					break;
				}
			}
			default:{
				break;
			}
		}
		return globalState;
	}

	private void applyStates(){
		switch(globalState){
			case Intake:{
				Intake();
				break;
			}
			case autoAlignCoral:{
				autoAlignCoral();
				break;
			}
			default:
				break;
			}

	}
	//Global Variables
	public boolean coralInEndEffector;
	public boolean limelightToleranceMet;
	public RobotStates globalState = RobotStates.Drive;
	public static WantedRobotStates wantedGlobalState = WantedRobotStates.Drive;
	public RobotStates previousRobotState;
	//State Triggers
	// public Trigger isDrive = new Trigger(() -> (globalState == RobotStates.Drive));
	// public Trigger isIntake = new Trigger(() -> (globalState == RobotStates.Intake));
	// public Trigger isautoAlignCoral = new Trigger(() -> (globalState == RobotStates.autoAlignCoral));
	// public Trigger ismaunualAlignCoral = new Trigger(() -> (globalState == RobotStates.manualAlignCoral));
	// public Trigger isScorecoral =  new Trigger(() -> (globalState == RobotStates.Scorecoral));
	// public Trigger isunscoreAlgaie = new Trigger(() -> (globalState == RobotStates.unscoreAlgaie));
	public void periodic(){
		globalState = handleStateTransition();
		applyStates();
		Logger.recordOutput("StateMachine/DesiredSuperstate", wantedGlobalState);
        Logger.recordOutput("StateMachine/CurrentSuperstate", globalState);
	}

	@Override
	public void simulationPeriodic() {
		periodic();
	}

	public void setWantedSuperState(WantedRobotStates wantedSuperState) {
        Superstate.wantedGlobalState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(WantedRobotStates wantedSuperState) {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }

	public void Intake(){
		System.out.println("State is Intake");
	}
	public Command autoAlignCoral(){
		return new ConditionalCommand(CenterLimelight.centerLimelightLeft(), CenterLimelight.centerLimelightRight(), RobotContainer.getInstance().driverOI.alignReefLeft)
			.andThen(new RunCommand(() -> {Superstate.wantedGlobalState = WantedRobotStates.manualAlignCoral;}));
	}
	public Command manualAlignCoral(){
		return new ParallelCommandGroup(
				RobotContainer.getInstance().elevator.goToGamePieceHeight(GamePieceType.CORAL),
				RobotContainer.getInstance().drivetrain.dPadMode()
			);
	}
	// public Command manualAlignCoral(){
	// 	return
	// }
	
}
