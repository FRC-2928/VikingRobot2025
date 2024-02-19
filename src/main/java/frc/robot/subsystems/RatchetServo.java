// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RatchetServo extends SubsystemBase {

  public static State currentState = State.ENGAGED;

	public static enum State {
		ENGAGED,
		DISENGAGED,
	}

  private final Servo servo;
  private double defaultAngle = 84;
  private double servoAngle = defaultAngle;
  private double minAngle = 84.0;
  private double maxAngle = 93.0;

  // Creates a new Servo subsystem
  public RatchetServo(final int channel) {
    this.servo = new Servo(channel);
    engageRatchet();
  }

  public void engageRatchet() {
    this.servo.setAngle(this.minAngle);
    this.currentState = State.ENGAGED;
  }

  public void disengageRatchet() {
    this.servo.setAngle(this.maxAngle);
    this.currentState = State.DISENGAGED;
  }

  public State getCurrentState() {
    return this.currentState;
  }

  public void toggleRatchet() {
    if (this.getCurrentState() == State.ENGAGED) {
      this.disengageRatchet();
    } else {
      this.engageRatchet();  
    }
  }

  public void setDefaultAngle(double angle){
    defaultAngle = angle;
    reset();
  }
  
  // Reset position to resting state
  public void reset() {
    this.servo.setAngle(this.servoAngle);
  }

  /** 
   * Increment servo1 motor position
   * 
   * @param delta Amount to change motor position
   */
  public void incrementServo(double delta) {
    this.servoAngle = saturateLimit(this.servoAngle + delta, this.minAngle, this.maxAngle);
    this.servo.setAngle(this.servoAngle);
  }

  /** 
   * Set the min and max angle range for this servo
   * 
   * @param minAngle Minimum servo angle position
   * @param maxAngle Maximum servo angle position
   */
  public void setAngleRange(int minAngle, int maxAngle) {
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
  }

  // Get the current servo angle
  @AutoLogOutput(key = "Servo Angle")
  public double getCurrentAngle() {
    return this.servoAngle;
  }

  // Limit motor range to avoid moving beyond safe ranges
  public double saturateLimit(double val, double min_angle, double max_angle) {
    double outval = val;
    if(val > max_angle) {
      outval =  max_angle;
    } else if (val < min_angle) {
      outval = min_angle;
    }
    return outval;
  }

  /**
   * Is the servo at its max allowable angle
   * 
   * @return true if at max angle, false if not
   */
  public boolean atMaxAngle() {
    return getCurrentAngle() >= this.maxAngle;
  }

  /**
   * Is the servo at its min allowable angle
   * 
   * @return true if at min angle, false if not
   */
  public boolean atMinAngle() {
    return getCurrentAngle() <= this.minAngle;
  }

}
