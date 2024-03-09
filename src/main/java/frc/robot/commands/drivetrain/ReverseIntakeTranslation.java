package frc.robot.commands.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.commands.shooter.IntakeGround;

public class ReverseIntakeTranslation extends Command {
	public ReverseIntakeTranslation() { this.addRequirements(Robot.cont.drivetrain); }

	private double end;

	@Override
	public void initialize() { this.end = Timer.getFPGATimestamp() + IntakeGround.lastTime / 2.0; }

	@Override
	public void execute() { Robot.cont.drivetrain.control(Robot.cont.drivetrain.rod(new ChassisSpeeds(2, 0, 0))); }

	@Override
	public boolean isFinished() { return Timer.getFPGATimestamp() >= this.end; }
}
