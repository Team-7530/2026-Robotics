package frc.lib.util;

import edu.wpi.first.wpilibj.Preferences;

/** Gets a value from dashboard in active mode, returns default if not or value in dashboard. */
public class PreferencesValue {
  private String key = "";
  private double defaultNumber = 0.0;
  private double lastHasChangedNumber = defaultNumber;
  public static boolean usePreferences = false;

  /**
   * Create a new PreferencesNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public PreferencesValue(String dashboardKey) {
    this.key = dashboardKey;
  }

  /**
   * Create a new PreferencesNumber with the default double
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public PreferencesValue(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefaultNumber(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public double getDefaultNumber() {
    return defaultNumber;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefaultNumber(double defaultValue) {
    this.defaultNumber = defaultValue;
    if (!usePreferences || !Preferences.containsKey(key)) {
      Preferences.setDouble(key, defaultValue);
    }
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public double get() {
    return usePreferences ? Preferences.getDouble(key, defaultNumber) : defaultNumber;
  }

  /**
   * Get the current value, from dashboard if available and in active mode
   *
   * @return The current value
   */
  public void set(double value) {
    Preferences.setDouble(key, value);
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedNumber) {
      lastHasChangedNumber = currentValue;
      return true;
    }
    return false;
  }
}
