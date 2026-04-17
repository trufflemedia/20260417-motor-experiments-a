// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// REVLib imports — these are from the REVLib 2026 vendor library
// Install via: WPILib Command Palette > Manage Vendor Libraries > Install new library (online)
// URL: https://software-metadata.revrobotics.com/REVLib-2026.json
import com.revrobotics.RelativeEncoder;               // Interface for reading NEO/NEO 550 built-in hall-effect encoder
import com.revrobotics.ResetMode;                     // Controls whether safe params are reset on configure()
import com.revrobotics.PersistMode;                   // Controls whether config is saved to SparkMax flash memory
import com.revrobotics.spark.SparkMax;                // Main class for controlling a REV SparkMax motor controller
import com.revrobotics.spark.SparkLowLevel.MotorType; // Enum: kBrushless for NEO/NEO 550, kBrushed for CIM etc.
import com.revrobotics.spark.config.SparkMaxConfig;   // Configuration object for SparkMax settings
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // Enum: kCoast or kBrake

// WPILib imports — core FRC robot programming library
import edu.wpi.first.wpilibj.PowerDistribution;            // Interface for REV PDH or CTRE PDP
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType; // Enum: kRev for PDH, kCtre for PDP
import edu.wpi.first.wpilibj.TimedRobot;                   // Base class — runs periodic methods every 20ms
import edu.wpi.first.wpilibj.Timer;                        // Provides FPGA hardware timestamp
import edu.wpi.first.wpilibj.XboxController;               // Interface for Xbox controller on Driver Station
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Sends data to dashboard for monitoring

/**
 * Main robot class for FRC Team 1018 — 2026 Season
 *
 * Hardware configuration:
 *   - roboRIO 2.0            : Main robot controller
 *   - REV SparkMax (CAN ID 3): Left motor controller  → NEO Brushless V1.1
 *   - REV SparkMax (CAN ID 2): Right motor controller → NEO 550
 *   - REV PDH                : Power Distribution Hub on CAN ID 1
 *   - Xbox Controller        : Driver input on USB port 0
 *
 * Driver controls:
 *   - Left  joystick Y-axis  : Controls NEO V1.1  (CAN ID 3)
 *   - Right joystick Y-axis  : Controls NEO 550   (CAN ID 2)
 *
 * IMPORTANT — NEO 550 vs NEO V1.1 differences:
 *   The NEO 550 is a smaller, lighter motor with a much lower thermal mass.
 *   It heats up significantly faster than the NEO V1.1 and has a stall
 *   current of 100A — it MUST have a conservative smart current limit set
 *   or it can be permanently damaged in seconds. All NEO 550 constants
 *   below are set more conservatively than the NEO V1.1 for this reason.
 *
 * Extends TimedRobot which calls each periodic method every 20ms (50Hz).
 */
public class Robot extends TimedRobot {

  // ─── Hardware Configuration Constants ────────────────────────────────────────

  /** CAN ID of the LEFT SparkMax — drives NEO Brushless V1.1 */
  private static final int kLeftMotorCanId = 3;

  /** CAN ID of the RIGHT SparkMax — drives NEO 550 */
  private static final int kRightMotorCanId = 2;

  /** USB port number of the Xbox controller on the Driver Station laptop */
  private static final int kXboxControllerPort = 0;

  // ─── Demo Safety Constants ────────────────────────────────────────────────────

  /**
   * DEMO RAMP RATE for the NEO V1.1 left motor (seconds from 0 to full throttle).
   *
   * 1.5 seconds gives smooth, predictable acceleration for demo use.
   * The NEO V1.1 has higher thermal mass so it can handle slightly faster ramps,
   * but 1.5 sec keeps the demo safe and controlled.
   *
   * To make faster: lower the value | To make slower: raise the value
   */
  private static final double kNeoRampRate = 1.5;

  /**
   * DEMO RAMP RATE for the NEO 550 right motor (seconds from 0 to full throttle).
   *
   * Set to 2.0 seconds — more conservative than the NEO V1.1 because:
   *   - The NEO 550 has much lower thermal mass and heats up faster
   *   - Slower ramp = smaller current spikes = less heat generated
   *   - Gentler acceleration also helps protect the smaller motor mechanically
   *
   * To make faster: lower the value | To make slower: raise the value
   */
  private static final double kNeo550RampRate = 2.0;

  /**
   * DEMO MAX SPEED for the NEO V1.1 left motor (0.0 to 1.0).
   *
   * Capped at 50% — clearly visible movement without risk of runaway.
   */
  private static final double kNeoMaxSpeed = 0.5;

  /**
   * DEMO MAX SPEED for the NEO 550 right motor (0.0 to 1.0).
   *
   * Capped at 40% — more conservative than the NEO V1.1 because:
   *   - Lower thermal mass means even moderate loads can cause rapid heat buildup
   *   - The NEO 550 is designed for non-drivetrain mechanisms (intakes, etc.)
   *     and may not be in a gearbox suitable for high-speed operation
   *   - Keeping it lower reduces current draw and heat generation significantly
   */
  private static final double kNeo550MaxSpeed = 0.4;

  // ─── Thermal Protection Constants — NEO Brushless V1.1 ───────────────────────

  /**
   * NEO V1.1 warning temperature (°C) — output reduced to 50%.
   * The NEO V1.1 has good thermal mass, warning at 60°C gives ample
   * time to cool before reaching the critical threshold.
   */
  private static final double kNeoTempWarningC  = 60.0;

  /**
   * NEO V1.1 critical temperature (°C) — output cut to zero.
   * Hard cutoff well below the NEO V1.1's internal thermal limit (~100°C).
   */
  private static final double kNeoTempCriticalC = 75.0;

  // ─── Thermal Protection Constants — NEO 550 ──────────────────────────────────

  /**
   * NEO 550 warning temperature (°C) — output reduced to 50%.
   *
   * Set 10°C LOWER than the NEO V1.1 (50°C vs 60°C) because:
   *   - The NEO 550 has much lower thermal mass — it heats up much faster
   *   - By the time we detect rising temperature, it may already be too late
   *   - Warning early gives the motor time to cool before hitting critical
   *   - REV strongly recommends conservative thermal management for the NEO 550
   */
  private static final double kNeo550TempWarningC  = 50.0;

  /**
   * NEO 550 critical temperature (°C) — output cut to zero.
   *
   * Set 10°C LOWER than the NEO V1.1 (65°C vs 75°C) for the same reasons.
   * The NEO 550's small size means internal temps rise faster than the
   * sensor reading — cutting off early gives the windings a safety margin.
   */
  private static final double kNeo550TempCriticalC = 65.0;

  // ─── Current Limit Constants ──────────────────────────────────────────────────

  /**
   * NEO V1.1 smart current limit (amps).
   * 30A is conservative for a NEO V1.1 in a demo context — safe and cool.
   * The NEO V1.1 can handle higher sustained currents but 30A is plenty for demo.
   */
  private static final int kNeoSmartCurrentLimit     = 30;
  private static final int kNeoSecondaryCurrentLimit = 40;

  /**
   * NEO 550 smart current limit (amps).
   *
   * CRITICAL: The NEO 550 has a stall current of 100A and can be permanently
   * damaged in a fraction of a second if current is not limited.
   * REV strongly recommends setting a conservative Smart Current Limit.
   *
   * 20A is the recommended safe limit for general NEO 550 use:
   *   - Provides enough torque for most mechanisms
   *   - Dramatically reduces heat generation under load
   *   - Prevents catastrophic damage if the mechanism stalls
   *
   * The secondary limit of 25A provides a hard instantaneous trip.
   */
  private static final int kNeo550SmartCurrentLimit     = 20;
  private static final int kNeo550SecondaryCurrentLimit = 25;

  // ─── Hardware Object Declarations ────────────────────────────────────────────

  /** Left SparkMax (CAN ID 3) driving a NEO Brushless V1.1 */
  private final SparkMax m_leftMotor;

  /** Right SparkMax (CAN ID 2) driving a NEO 550 */
  private final SparkMax m_rightMotor;

  /** Built-in 42 CPR hall-effect encoder from the NEO V1.1 */
  private final RelativeEncoder m_leftEncoder;

  /** Built-in 42 CPR hall-effect encoder from the NEO 550 */
  private final RelativeEncoder m_rightEncoder;

  /** Xbox controller used by the driver on the Driver Station */
  private final XboxController m_controller;

  /** REV Power Distribution Hub — current and voltage monitoring */
  private final PowerDistribution m_pdh;

  // ─── Constructor ─────────────────────────────────────────────────────────────

  /**
   * Robot constructor — runs once when the robot program starts.
   * Each SparkMax is configured with motor-specific settings appropriate
   * for either the NEO V1.1 or NEO 550.
   */
  public Robot() {

    // ── Left Motor: NEO Brushless V1.1 (CAN ID 3) ────────────────────────────

    // Initialize left SparkMax for the NEO V1.1 — kBrushless REQUIRED for NEO motors
    m_leftMotor = new SparkMax(kLeftMotorCanId, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();

    leftConfig
      // COAST: motor spins freely when not commanded — no heat from braking resistance
      .idleMode(IdleMode.kCoast)

      // NEO V1.1 current limits — 30A sustained, 40A instantaneous spike limit
      .smartCurrentLimit(kNeoSmartCurrentLimit)
      .secondaryCurrentLimit(kNeoSecondaryCurrentLimit)

      // NEO V1.1 demo ramp rate: 1.5 seconds to full throttle
      // Smooth acceleration, reduced current spikes, safe for demo use
      .openLoopRampRate(kNeoRampRate)

      // Normalize output to 12V nominal as battery voltage drops
      .voltageCompensation(12.0);

    leftConfig.encoder
      // NEO V1.1 encoder: 42 CPR hall-effect sensor
      // Converts rotations → inches for a 6-inch wheel
      .positionConversionFactor((Math.PI * 6) / 42.0)  // inches per rotation
      .velocityConversionFactor((Math.PI * 6) / 42.0); // inches per minute

    // Apply NEO V1.1 config — reset safe params, persist to flash
    m_leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get NEO V1.1 encoder AFTER configure() so conversion factors are applied
    m_leftEncoder = m_leftMotor.getEncoder();

    // ── Right Motor: NEO 550 (CAN ID 2) ──────────────────────────────────────

    // Initialize right SparkMax for the NEO 550
    // kBrushless is REQUIRED — the NEO 550 is a brushless motor
    m_rightMotor = new SparkMax(kRightMotorCanId, MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();

    rightConfig
      // COAST: especially important for the NEO 550 — brake mode under any
      // mechanical load will rapidly overheat this small motor
      .idleMode(IdleMode.kCoast)

      // NEO 550 CRITICAL current limits:
      // 20A smart limit — REV's recommended safe limit for this motor
      // The NEO 550 stalls at 100A and can be destroyed in under a second
      // without a proper current limit in place
      .smartCurrentLimit(kNeo550SmartCurrentLimit)
      .secondaryCurrentLimit(kNeo550SecondaryCurrentLimit)

      // NEO 550 demo ramp rate: 2.0 seconds to full throttle
      // More conservative than NEO V1.1 — slower ramp = less current spike
      // = less heat = longer motor life, especially critical for this small motor
      .openLoopRampRate(kNeo550RampRate)

      // Same voltage compensation as left motor
      .voltageCompensation(12.0);

    rightConfig.encoder
      // NEO 550 also uses a 42 CPR hall-effect encoder (same as NEO V1.1)
      // Conversion factors assume a 6-inch wheel — adjust if geared differently
      .positionConversionFactor((Math.PI * 6) / 42.0)  // inches per rotation
      .velocityConversionFactor((Math.PI * 6) / 42.0); // inches per minute

    // Apply NEO 550 config — reset safe params, persist to flash
    m_rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get NEO 550 encoder AFTER configure() so conversion factors are applied
    m_rightEncoder = m_rightMotor.getEncoder();

    // ── Shared Devices ────────────────────────────────────────────────────────

    // Xbox controller on USB port 0 of the Driver Station
    m_controller = new XboxController(kXboxControllerPort);

    // REV PDH on default CAN ID 1
    m_pdh = new PowerDistribution(1, ModuleType.kRev);
  }

  // ─── robotPeriodic ───────────────────────────────────────────────────────────

  /**
   * Called every 20ms regardless of robot mode.
   * Sends telemetry for both motors to SmartDashboard.
   * Each motor displays its own temperature status with motor-specific thresholds.
   */
  @Override
  public void robotPeriodic() {

    // Read current temperatures from both motors
    double leftTemp  = m_leftMotor.getMotorTemperature();  // NEO V1.1 temp in °C
    double rightTemp = m_rightMotor.getMotorTemperature(); // NEO 550 temp in °C

    // ── Demo Settings Confirmation ────────────────────────────────────────────

    // Display active demo settings for each motor — verify at a glance before running
    SmartDashboard.putNumber("NEO Ramp Rate (sec)",    kNeoRampRate);
    SmartDashboard.putNumber("NEO Max Speed (%)",      kNeoMaxSpeed * 100);
    SmartDashboard.putNumber("NEO 550 Ramp Rate (sec)", kNeo550RampRate);
    SmartDashboard.putNumber("NEO 550 Max Speed (%)",  kNeo550MaxSpeed * 100);

    // ── General Telemetry ─────────────────────────────────────────────────────

    SmartDashboard.putNumber("Timer",                    Timer.getFPGATimestamp());
    SmartDashboard.putNumber("PDH Voltage",              m_pdh.getVoltage());

    // ── Left Motor Telemetry (NEO V1.1) ──────────────────────────────────────

    SmartDashboard.putNumber("NEO Encoder Position",     m_leftEncoder.getPosition());
    SmartDashboard.putNumber("NEO Encoder Velocity",     m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("NEO Motor Output",         m_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("NEO Temp (C)",             leftTemp);
    SmartDashboard.putNumber("NEO Current (A)",          m_pdh.getCurrent(3));

    // ── Right Motor Telemetry (NEO 550) ──────────────────────────────────────

    SmartDashboard.putNumber("NEO 550 Encoder Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("NEO 550 Encoder Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("NEO 550 Motor Output",     m_rightMotor.getAppliedOutput());
    SmartDashboard.putNumber("NEO 550 Temp (C)",         rightTemp);
    SmartDashboard.putNumber("NEO 550 Current (A)",      m_pdh.getCurrent(2));

    // ── NEO V1.1 Temperature Status ───────────────────────────────────────────

    // Uses NEO-specific thresholds: Warning 60°C, Critical 75°C
    if (leftTemp >= kNeoTempCriticalC) {
      SmartDashboard.putString("NEO Temp Status",     "RED - CRITICAL - NEO OVERHEATING");
    } else if (leftTemp >= kNeoTempWarningC) {
      SmartDashboard.putString("NEO Temp Status",     "YELLOW - WARNING - NEO getting hot");
    } else {
      SmartDashboard.putString("NEO Temp Status",     "GREEN - OK");
    }

    // ── NEO 550 Temperature Status ────────────────────────────────────────────

    // Uses NEO 550-specific thresholds: Warning 50°C, Critical 65°C
    // These are lower than the NEO V1.1 due to the smaller motor's lower thermal mass
    if (rightTemp >= kNeo550TempCriticalC) {
      SmartDashboard.putString("NEO 550 Temp Status", "RED - CRITICAL - NEO 550 OVERHEATING");
    } else if (rightTemp >= kNeo550TempWarningC) {
      SmartDashboard.putString("NEO 550 Temp Status", "YELLOW - WARNING - NEO 550 getting hot");
    } else {
      SmartDashboard.putString("NEO 550 Temp Status", "GREEN - OK");
    }
  }

  // ─── teleopInit ──────────────────────────────────────────────────────────────

  /**
   * Called once when teleop mode begins.
   * Resets both encoders to zero for consistent position readings.
   */
  @Override
  public void teleopInit() {
    m_leftEncoder.setPosition(0);  // Zero NEO V1.1 encoder
    m_rightEncoder.setPosition(0); // Zero NEO 550 encoder
  }

  // ─── teleopPeriodic ──────────────────────────────────────────────────────────

  /**
   * Called every 20ms during teleop.
   * Each motor is controlled independently with its own joystick axis,
   * speed cap, and motor-specific thermal protection thresholds.
   *
   * Control mapping:
   *   - Left  joystick Y → NEO V1.1 (CAN ID 3) — max 50%, ramp 1.5 sec
   *   - Right joystick Y → NEO 550  (CAN ID 2) — max 40%, ramp 2.0 sec
   *
   * Speed pipeline (applied per motor):
   *   1. Read joystick          (-1.0 to 1.0)
   *   2. Apply deadband         (±0.05 zeroed to eliminate drift)
   *   3. Clamp to max speed     (hard ceiling per motor type)
   *   4. Apply thermal cutoff   (motor-specific thresholds)
   *   5. Send to SparkMax       (firmware applies ramp rate on top)
   */
  @Override
  public void teleopPeriodic() {

    // Read current temperatures for independent thermal protection
    double leftTemp  = m_leftMotor.getMotorTemperature();  // NEO V1.1
    double rightTemp = m_rightMotor.getMotorTemperature(); // NEO 550

    // ── Left Motor: NEO V1.1 Control ─────────────────────────────────────────

    // Step 1: Read left joystick Y-axis for NEO V1.1 control
    double leftSpeed = m_controller.getLeftY();

    // Step 2: Deadband — zero out small stick values caused by drift
    if (Math.abs(leftSpeed) < 0.05) {
      leftSpeed = 0.0;
    }

    // Step 3: Clamp to NEO V1.1 demo max speed (50%)
    // Math.signum preserves direction, multiplied by the speed ceiling
    if (Math.abs(leftSpeed) > kNeoMaxSpeed) {
      leftSpeed = Math.signum(leftSpeed) * kNeoMaxSpeed;
    }

    // Step 4 & 5: Apply NEO V1.1 thermal protection and send to motor
    // SparkMax firmware applies the 1.5 sec ramp on top of whatever value we send
    if (leftTemp >= kNeoTempCriticalC) {
      // Above 75°C: cut output entirely — must cool before running
      m_leftMotor.set(0.0);
    } else if (leftTemp >= kNeoTempWarningC) {
      // Above 60°C: reduce to 50% of already-capped demo speed
      m_leftMotor.set(leftSpeed * 0.5);
    } else {
      // Normal: send demo-capped speed, SparkMax ramp smooths the transition
      m_leftMotor.set(leftSpeed);
    }

    // ── Right Motor: NEO 550 Control ─────────────────────────────────────────

    // Step 1: Read right joystick Y-axis for NEO 550 control
    double rightSpeed = m_controller.getRightY();

    // Step 2: Deadband — zero out small stick values caused by drift
    if (Math.abs(rightSpeed) < 0.05) {
      rightSpeed = 0.0;
    }

    // Step 3: Clamp to NEO 550 demo max speed (40%)
    // More conservative cap than NEO V1.1 — protects the smaller motor
    if (Math.abs(rightSpeed) > kNeo550MaxSpeed) {
      rightSpeed = Math.signum(rightSpeed) * kNeo550MaxSpeed;
    }

    // Step 4 & 5: Apply NEO 550 thermal protection and send to motor
    // NEO 550 thresholds are lower (50°C/65°C) than NEO V1.1 (60°C/75°C)
    // SparkMax firmware applies the 2.0 sec ramp on top of whatever value we send
    if (rightTemp >= kNeo550TempCriticalC) {
      // Above 65°C: cut output entirely — NEO 550 MUST cool before running again
      m_rightMotor.set(0.0);
    } else if (rightTemp >= kNeo550TempWarningC) {
      // Above 50°C: reduce to 50% of already-capped demo speed
      m_rightMotor.set(rightSpeed * 0.5);
    } else {
      // Normal: send demo-capped speed, SparkMax ramp smooths the transition
      m_rightMotor.set(rightSpeed);
    }
  }

  // ─── disabledInit ────────────────────────────────────────────────────────────

  /**
   * Called once when the robot transitions to disabled mode.
   * Both motors are explicitly stopped — critical safety measure.
   * The NEO 550 in particular must never be left in brake mode under load.
   */
  @Override
  public void disabledInit() {
    // Explicitly zero both motors
    m_leftMotor.set(0.0);
    m_rightMotor.set(0.0);

    // WPILib safety stop — disables output at the SparkMax firmware level
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }
}
