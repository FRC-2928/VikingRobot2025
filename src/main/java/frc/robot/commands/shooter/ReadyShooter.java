// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ReadyShooter extends Command {
	public ReadyShooter(final Measure<Angle> angle) {
		this.angle = angle;
		this.addRequirements(Robot.cont.shooter);
	}

	public final Measure<Angle> angle;

	@Override
	public void execute() { Robot.cont.shooter.io.rotate(this.angle); }

	@Override
	public boolean isFinished() {
		return Math.abs(Robot.cont.shooter.inputs.angle.in(Units.Degrees) - this.angle.in(Units.Degrees)) < 2;
	}
}
