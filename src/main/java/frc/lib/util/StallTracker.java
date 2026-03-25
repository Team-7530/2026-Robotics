package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Small helper for timing diagnostics in robot code.
 *
 * <p>Use the periodic-loop methods for code that runs on a repeated scheduler/timed loop, and the
 * scoped-section methods for measuring a single block of work inside some larger loop.
 */
public class StallTracker {
  private static final double DEFAULT_WARNING_RATE_LIMIT_SECONDS = 1.0;

  private final String title;
  private final boolean enabled;
  private final double warningRateLimitSeconds;

  private double lastWarningSeconds = Double.NEGATIVE_INFINITY;
  private double lastPeriodicStartSeconds = Double.NaN;
  private double currentPeriodicStartSeconds = Double.NaN;
  private double currentScopedStartSeconds = Double.NaN;

  public StallTracker(String title, boolean enabled) {
    this(title, enabled, DEFAULT_WARNING_RATE_LIMIT_SECONDS);
  }

  public StallTracker(String title, boolean enabled, double warningRateLimitSeconds) {
    this.title = title;
    this.enabled = enabled;
    this.warningRateLimitSeconds = warningRateLimitSeconds;
  }

  public void startPeriodicLoop(String loopName, double gapWarningSeconds) {
    if (!enabled) {
      return;
    }

    currentPeriodicStartSeconds = Timer.getFPGATimestamp();
    if (Double.isFinite(lastPeriodicStartSeconds)) {
      double gapSeconds = currentPeriodicStartSeconds - lastPeriodicStartSeconds;
      if (gapSeconds > gapWarningSeconds) {
        reportWarning(String.format("%s loop gap detected: %.1f ms", loopName, gapSeconds * 1000.0));
      }
    }

    lastPeriodicStartSeconds = currentPeriodicStartSeconds;
  }

  public void endPeriodicLoop(String loopName, double durationWarningSeconds) {
    if (!enabled || !Double.isFinite(currentPeriodicStartSeconds)) {
      return;
    }

    double durationSeconds = Timer.getFPGATimestamp() - currentPeriodicStartSeconds;
    currentPeriodicStartSeconds = Double.NaN;
    if (durationSeconds > durationWarningSeconds) {
      reportWarning(String.format("Slow %s loop: %.1f ms", loopName, durationSeconds * 1000.0));
    }
  }

  public void startScopedSection() {
    if (!enabled) {
      return;
    }

    currentScopedStartSeconds = Timer.getFPGATimestamp();
  }

  public void endScopedSection(String sectionName, double durationWarningSeconds) {
    if (!enabled || !Double.isFinite(currentScopedStartSeconds)) {
      return;
    }

    double durationSeconds = Timer.getFPGATimestamp() - currentScopedStartSeconds;
    currentScopedStartSeconds = Double.NaN;
    if (durationSeconds > durationWarningSeconds) {
      reportWarning(String.format("Slow %s: %.1f ms", sectionName, durationSeconds * 1000.0));
    }
  }

  public void reportWarning(String message) {
    if (!enabled) {
      return;
    }

    double nowSeconds = Timer.getFPGATimestamp();
    if ((nowSeconds - lastWarningSeconds) < warningRateLimitSeconds) {
      return;
    }

    lastWarningSeconds = nowSeconds;
    DriverStation.reportWarning(title + ": " + message, false);
  }
}
