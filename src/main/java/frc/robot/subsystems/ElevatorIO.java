package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public interface ElevatorIO {
	@AutoLog
	public class ElevatorIOInputs {
		public Distance height = Units.Meters.zero();
		public boolean homePos;
		public LinearVelocity speed;
	}

	public default void moveToPosition(final Distance position) {}

	public default void pivotBanana(final double rotation) {}

	public default void updateInputs(final ElevatorIOInputs inputs) {}
}
