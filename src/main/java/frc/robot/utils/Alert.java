package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Predicate;

public class Alert {
	private static Map<String, SendableAlerts> groups = new HashMap<String, SendableAlerts>();

	private final AlertType type;
	private boolean active = false;
	private double activeStartTime = 0.0;
	private String text;

	public Alert(final String text, final AlertType type) { this("Alerts", text, type); }

	public Alert(final String group, final String text, final AlertType type) {
		if(!Alert.groups.containsKey(group)) {
			Alert.groups.put(group, new SendableAlerts());
			SmartDashboard.putData(group, Alert.groups.get(group));
		}

		this.text = text;
		this.type = type;
		Alert.groups.get(group).alerts.add(this);
	}

	public void set(final boolean active) {
		if(active && !this.active) {
			this.activeStartTime = Timer.getFPGATimestamp();
			switch(this.type) {
			case ERROR:
				DriverStation.reportError(this.text, false);
				break;
			case WARNING:
				DriverStation.reportWarning(this.text, false);
				break;
			case INFO:
				System.out.println(this.text);
				break;
			}
		}
		this.active = active;
	}

	public void setText(final String text) {
		if(this.active && !text.equals(this.text)) {
			switch(this.type) {
			case ERROR:
				DriverStation.reportError(text, false);
				break;
			case WARNING:
				DriverStation.reportWarning(text, false);
				break;
			case INFO:
				System.out.println(text);
				break;
			}
		}
		this.text = text;
	}

	private static class SendableAlerts implements Sendable {
		public final List<Alert> alerts = new ArrayList<>();

		public String[] getStrings(final AlertType type) {
			final Predicate<Alert> activeFilter = (final Alert x) -> x.type == type && x.active;
			final Comparator<
				Alert> timeSorter = (final Alert a1, final Alert a2) -> (int) (a2.activeStartTime - a1.activeStartTime);
			return this.alerts
				.stream()
				.filter(activeFilter)
				.sorted(timeSorter)
				.map((final Alert a) -> a.text)
				.toArray(String[]::new);
		}

		@Override
		public void initSendable(final SendableBuilder builder) {
			builder.setSmartDashboardType("Alerts");
			builder.addStringArrayProperty("errors", () -> this.getStrings(AlertType.ERROR), null);
			builder.addStringArrayProperty("warnings", () -> this.getStrings(AlertType.WARNING), null);
			builder.addStringArrayProperty("infos", () -> this.getStrings(AlertType.INFO), null);
		}
	}

	/** Represents an alert's level of urgency. */
	public static enum AlertType {
		/**
		 * High priority alert - displayed first on the dashboard with a red "X" symbol. Use this type for problems which will seriously affect the robot's functionality and thus require immediate attention.
		 */
		ERROR,

		/**
		 * Medium priority alert - displayed second on the dashboard with a yellow "!" symbol. Use this type for problems which could affect the robot's functionality but do not necessarily require immediate attention.
		 */
		WARNING,

		/**
		 * Low priority alert - displayed last on the dashboard with a green "i" symbol. Use this type for problems which are unlikely to affect the robot's functionality, or any other alerts which do not fall under "ERROR" or "WARNING".
		 */
		INFO
	}
}
