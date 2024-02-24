package frc.robot.commands.diagnostics;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ShooterIOReal;

public class ZeroPivot extends Command {
	public ZeroPivot() { this.addRequirements(Robot.cont.shooter); }

	@Override
	public void initialize() {
		// todo find current mag sens cfg
		final CANcoder enc = ((ShooterIOReal) Robot.cont.shooter.io).encoder;
		final MagnetSensorConfigs cfg = new MagnetSensorConfigs();
		cfg.MagnetOffset = 0;
	}
}
