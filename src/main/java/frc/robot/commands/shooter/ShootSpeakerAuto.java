package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;

public class ShootSpeakerAuto extends ShootSpeaker {

	private double shotTime = -1;
	private final double postShotWaitTime = 0.25;

	@Override
	public void execute() {
		if(this.shotTime < 0 && this.fired) {
			this.shotTime = Timer.getFPGATimestamp();
		}
		super.execute();
	}

	@Override
	public boolean isFinished() {
		return this.shotTime > 0 && (Timer.getFPGATimestamp() - this.shotTime >= this.postShotWaitTime);
	}
}
