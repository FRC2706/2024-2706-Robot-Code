// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;
// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber implements DoubleSupplier {
  private static final String tableKey = "TunableNumbers";

  private final String key;
  private boolean hasDefault = false;
  private double defaultValue;
  private Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Create a new LoggedTunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  /**
   * Set the default value of the number. The default value can only be set once.
   *
   * @param defaultValue The default value
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (Config.tuningMode) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
      }
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Config.tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise.
   */
  public boolean hasChanged(int id) {
    if (!Config.tuningMode) return false;
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Runs action if any of the tunableNumbers have changed
   *
   * @param id Unique identifier for the caller to avoid conflicts when shared between multiple *
   *     objects. Recommended approach is to pass the result of "hashCode()"
   * @param action Callback to run when any of the tunable numbers have changed. Access tunable
   *     numbers in order inputted in method
   * @param tunableNumbers All tunable numbers to check
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, TunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(TunableNumber::get).toArray());
    }
  }

  /** Runs action if any of the tunableNumbers have changed */
  public static void ifChanged(int id, Runnable action, TunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}