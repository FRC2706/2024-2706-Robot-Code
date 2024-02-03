// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile. Users should
 * call reset() when they first start running the controller to avoid unwanted behavior.
 * 
 * This is a modified version of the {@link edu.wpi.first.math.controller.ProfiledPIDController} from WPILib to export the PIDController to
 * an external device like the CANSparkMax or other motor controllers.
 */
public class ProfiledExternalPIDController {
  // The period (in seconds) of the loop that calls the controller
  private final double m_period;

  // Do the endpoints wrap around? e.g. Absolute encoder
  private boolean m_continuous;
  private double m_minimumInput;
  private double m_maximumInput;

  // The error at the time of the most recent call to calculate()
  private double m_positionError;
  private double m_velocityError;

  // The error at the time of the second-most-recent call to calculate() (used to compute velocity)
  private double m_prevError;

  private boolean m_haveMeasurement = false;
  private boolean m_haveGoal = false;
  
  private TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile m_profile;
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  /**
   * Allocates a ProfiledExternalPIDController with a default 20ms period.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public ProfiledExternalPIDController(TrapezoidProfile.Constraints constraints) {
    this(constraints, 0.02);
  }

  /**
   * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and Kd.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   * @param period The period between controller updates in seconds. The default is 0.02 seconds.
   * @throws IllegalArgumentException if period &lt;= 0
   */
  @SuppressWarnings("this-escape")
  public ProfiledExternalPIDController(TrapezoidProfile.Constraints constraints, double period) {
    if (period <= 0) {
        throw new IllegalArgumentException("Controller period must be a non-zero positive number!");
    }
    
    m_period = period;

    m_constraints = constraints;
    m_profile = new TrapezoidProfile(m_constraints);
  }

  /**
   * Sets the goal for the ProfiledPIDController.
   *
   * @param goal The desired goal state.
   */
  public void setGoal(TrapezoidProfile.State goal) {
    m_goal = goal;
    m_haveGoal = true;
  }

  /**
   * Sets the goal for the ProfiledPIDController. Assumes a goal velocity of 0.
   *
   * @param goal The desired goal position.
   */
  public void setGoal(double goal) {
    setGoal(new TrapezoidProfile.State(goal, 0));
  }

  /**
   * Gets the goal for the ProfiledPIDController.
   *
   * @return The goal.
   */
  public TrapezoidProfile.State getGoal() {
    return m_goal;
  }

  /**
   * Returns true if the position error and velocity error is within the given tolerance.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @param positionTolerance Position error which is tolerable.
   * @param velocityTolerance Velocity error which is tolerable.
   * @return True if the error is within the tolerance of the error.
   */
  public boolean atGoal(double positionTolerance, double velocityTolerance) {
    return m_haveMeasurement
        && m_haveGoal
        && Math.abs(getPositionError()) < positionTolerance
        && Math.abs(getVelocityError()) < velocityTolerance;
  }

  /**
   * Returns true if the position error is within the given tolerance.
   *
   * <p>This will return false until at least one input value has been computed.
   *
   * @param positionTolerance Position error which is tolerable.
   * @return True if the error is within the tolerance of the error.
   */
  public boolean atGoal(double positionTolerance) {
    return atGoal(positionTolerance, Double.POSITIVE_INFINITY);
  }

  /**
   * Set velocity and acceleration constraints for goal.
   *
   * @param constraints Velocity and acceleration constraints for goal.
   */
  public void setConstraints(TrapezoidProfile.Constraints constraints) {
    m_constraints = constraints;
    m_profile = new TrapezoidProfile(m_constraints);
  }

  /**
   * Get the velocity and acceleration constraints for this controller.
   *
   * @return Velocity and acceleration constraints.
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return m_constraints;
  }

  /**
   * Returns the current setpoint of the ProfiledPIDController.
   *
   * @return The current setpoint.
   */
  public TrapezoidProfile.State getSetpoint() {
    return m_setpoint;
  }

  /**
   * Gets the period of this controller.
   *
   * @return The period of the controller.
   */
  public double getPeriod() {
    return m_period;
  }

  /**
   * Enables continuous input. The external pid controller must run continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  public void enableContinuousInput(double minimumInput, double maximumInput) {
    m_continuous = true;
    m_minimumInput = minimumInput;
    m_maximumInput = maximumInput;
  }

  /** Disables continuous input. */
  public void disableContinuousInput() {
    m_continuous = false;
  }

  /**
   * Returns the difference between the goal and the measurement.
   *
   * @return The error.
   */
  public double getPositionError() {
    return m_positionError;
  }

  /**
   * Returns the change in error per second between the goal and the measurement.
   *
   * @return The change in error per second.
   */
  public double getVelocityError() {
    return m_velocityError;
  }

  /**
   * Returns the next setpoint for a PID controller to run.
   *
   * @param measurement The current measurement of the process variable.
   * @return A setpoint for a PID controller's next input.
   */
  public double calculatePIDSetpoint(double measurement) {
    m_prevError = m_positionError;
    m_haveMeasurement = true;

    if (m_continuous) {
      // Get error which is the smallest distance between goal and measurement
      double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
      double goalMinDistance =
          MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
      double setpointMinDistance =
          MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);

      // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
      // may be outside the input range after this operation, but that's OK because the controller
      // will still go there and report an error of zero. In other words, the setpoint only needs to
      // be offset from the measurement by the input range modulus; they don't need to be equal.
      m_goal.position = goalMinDistance + measurement;
      m_setpoint.position = setpointMinDistance + measurement;

      m_positionError = goalMinDistance; 
    } else {
      m_positionError = m_goal.position - measurement;
    }

    m_velocityError = (m_positionError - m_prevError) / m_period;

    m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
    return m_setpoint.position;
  }

  /**
   * Returns the next setpoint for a PID controller to run.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return A setpoint for a PID controller's next input.
   */
  public double calculatePIDSetpoint(double measurement, TrapezoidProfile.State goal) {
    setGoal(goal);
    return calculatePIDSetpoint(measurement);
  }

  /**
   * Returns the next setpoint for a PID controller to run.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @return A setpoint for a  PID controller's next input.
   */
  public double calculatePIDSetpoint(double measurement, double goal) {
    setGoal(goal);
    return calculatePIDSetpoint(measurement);
  }

  /**
   * Returns the next setpoint for a PID controller to run.
   *
   * @param measurement The current measurement of the process variable.
   * @param goal The new goal of the controller.
   * @param constraints Velocity and acceleration constraints for goal.
   * @return A setpoint for a PID controller's next input.
   */
  public double calculatePIDSetpoint(
      double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
    setConstraints(constraints);
    return calculatePIDSetpoint(measurement, goal);
  }

  /**
   * Reset the profile to start from the given measurement.
   *
   * @param measurement The current measured State of the system.
   */
  public void reset(TrapezoidProfile.State measurement) {
    m_setpoint = measurement;
    m_positionError = 0;
    m_velocityError = 0;
    m_haveMeasurement = false;
    m_haveGoal = false;
  }

  /**
   * Reset the profile to start from the given measured position and velocity.
   *
   * @param measuredPosition The current measured position of the system.
   * @param measuredVelocity The current measured velocity of the system.
   */
  public void reset(double measuredPosition, double measuredVelocity) {
    reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
  }

  /**
   * Reset the profile to start from the given measured position.
   *
   * @param measuredPosition The current measured position of the system. The velocity is assumed to
   *     be zero.
   */
  public void reset(double measuredPosition) {
    reset(measuredPosition, 0.0);
  }
}
