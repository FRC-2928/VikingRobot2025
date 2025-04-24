// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drivetrain.CenterLimelight;

/** Add your docs here. */
public class Superstate {
	public enum RobotStates{
		Drive,
		Intake,
		autoAlignCoral,
		manualAlignCoral,
		Scorecoral,
		unscoreAlgaie
	}
	//Global Variables
	public boolean coralInEndEffector;
	public boolean limelightToleranceMet;
	public RobotStates globalState = RobotStates.Drive;
	//State Triggers
	public Trigger isDrive = new Trigger(() -> (globalState == RobotStates.Drive));
	public Trigger isIntake = new Trigger(() -> (globalState == RobotStates.Intake));
	public Trigger isautoAlignCoral = new Trigger(() -> (globalState == RobotStates.autoAlignCoral));
	public Trigger ismaunualAlignCoral = new Trigger(() -> (globalState == RobotStates.manualAlignCoral));
	public Trigger isScorecoral =  new Trigger(() -> (globalState == RobotStates.Scorecoral));
	public Trigger isunscoreAlgaie = new Trigger(() -> (globalState == RobotStates.unscoreAlgaie));
	//Transition Triggers
	public Trigger transitionDrivetoIntake = isDrive
		.and(RobotContainer.getInstance().operatorOI.passOffCoral);
	public Trigger transitionDrivetoautoAlignCoral = isautoAlignCoral
		.and(RobotContainer.getInstance().driverOI.alignReefLeft.or(RobotContainer.getInstance().driverOI.alignReefRight))
		.and(() -> (coralInEndEffector));
	public Trigger transitionDrivetoUnscoreAlgaie = isDrive
		.and(RobotContainer.getInstance().driverOI.alignReefCenter)
		.and(RobotContainer.getInstance().driverOI.closeToReef);
	public Trigger transitionManualAlignCoraltoScoreCoral = ismaunualAlignCoral
		.and(() -> {return !(RobotContainer.getInstance().driverOI.alignReefLeft.getAsBoolean() || (RobotContainer.getInstance().driverOI.alignReefRight.getAsBoolean()));});
	public Trigger transitionAutoAlignCoraltoDrive = isautoAlignCoral
		.and(() -> {return !(RobotContainer.getInstance().driverOI.alignReefLeft.getAsBoolean() || (RobotContainer.getInstance().driverOI.alignReefRight.getAsBoolean()));});
	//Global Varibales
	public Superstate() {
		//Activate Transition Triggers
		transitionDrivetoIntake.onTrue(new RunCommand(() -> {this.globalState = RobotStates.Intake;}));
		//Activate State Triggers
		isDrive.whileTrue(driveCommand());
	}

	public Command driveCommand(){
		return new InstantCommand(() -> {System.out.println("State is A");});
	}

	public Command autoAlignCoral(){
		return new ConditionalCommand(CenterLimelight.centerLimelightLeft(), CenterLimelight.centerLimelightRight(), RobotContainer.getInstance().driverOI.alignReefLeft)
			.andThen(new RunCommand(() -> {this.globalState = RobotStates.autoAlignCoral;}));
	}
	// public Command manualAlignCoral(){
	// 	return
	// }
	
}
