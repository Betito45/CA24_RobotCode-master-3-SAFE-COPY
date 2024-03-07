package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.subsystems.leds.LEDs;

public class Intake extends Subsystem {

  // DAVID - WILL NEED TO UPDATE THESE VALUES!!!
  private static final double k_pivotMotorP = 0.12;
  private static final double k_pivotMotorI = 0.0;
  private static final double k_pivotMotorD = 0.001;

  private SparkPIDController m_pivotPID;

  // private final PIDController m_pivotPID = new PIDController(k_pivotMotorP, k_pivotMotorI, k_pivotMotorD);

  //private final DutyCycleEncoder m_pivotEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);
  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.Intake.k_intakeLimitSwitchId);

  public final LEDs m_leds = LEDs.getInstance();

  private AbsoluteEncoder m_pivotEncoder;

  private final double GearRatio = 4.0 ; // DAVID NEED TO FIND OUT WHAT REAL VALUE IS
  private final double MotorRotationsPerDegree = (1 / 360.0) * GearRatio ;


  /*-------------------------------- Private instance variables ---------------------------------*/
  private static Intake mInstance;
  private PeriodicIO m_periodicIO;

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }

  private CANSparkMax mIntakeMotor;
  private CANSparkMax mPivotMotor;

  private Intake() {
    super("Intake");

    mIntakeMotor = new CANSparkMax(Constants.Intake.kIntakeMotorId, MotorType.kBrushed);
    mIntakeMotor.restoreFactoryDefaults();
    mIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    mPivotMotor = new CANSparkMax(Constants.Intake.kPivotMotorId, MotorType.kBrushed);
    mPivotMotor.restoreFactoryDefaults();
    mPivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    mPivotMotor.setSmartCurrentLimit(10);

    m_pivotEncoder = mPivotMotor.getAbsoluteEncoder( Type.kDutyCycle) ;
    m_pivotEncoder.setZeroOffset(0.0) ;  // DAVID - need to get angle here! Use Rev Hardware Client to get value.

    m_pivotPID = mPivotMotor.getPIDController();
    m_pivotPID.setFeedbackDevice(m_pivotEncoder) ;
    m_pivotPID.setP(k_pivotMotorP);
    m_pivotPID.setI(k_pivotMotorI);
    m_pivotPID.setD(k_pivotMotorD);
    m_pivotPID.setFF(0);  // DAVID - ask expert if feed forward is needed for this. Its sort of like an arm


    // DAVID - find someone familiar with REV Smart motion to help here. Maybe 2 revolutiuons per second?
    int smartMotionSlot = 0;
    m_pivotPID.setSmartMotionMaxVelocity(2.0, smartMotionSlot);  // set values in rotations per second
    m_pivotPID.setSmartMotionMinOutputVelocity(0.0, smartMotionSlot);
    m_pivotPID.setSmartMotionMaxAccel(4.0, smartMotionSlot);  // accelarate to full speed in 1/2 second
    m_pivotPID.setSmartMotionAllowedClosedLoopError(0.05, smartMotionSlot);  // david - Find expert to help with this number.



    m_periodicIO = new PeriodicIO();
  }

  private static class PeriodicIO {
    // Input: Desired state
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // Output: Motor set values
    double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE,
    GROUND,
    SOURCE,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER,
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    checkAutoTasks();

    // Pivot control
    double pivot_angle = pivotTargetToAngle(m_periodicIO.pivot_target);

    m_pivotPID.setReference(pivot_angle * MotorRotationsPerDegree, ControlType.kSmartMotion); // sets the motor setpoint without feed forward to compensate for gravity

    //m_pivotPID.setReference(pivot_angle * MotorRotationsPerDegree, ControlType.kSmartMotion, 0, feedForward, ArbFFUnits.kPercentOut);

    // m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getPivotAngleDegrees(), pivot_angle);

    // // If the pivot is at exactly 0.0, it's probably not connected, so disable it
    // if (m_pivotEncoder.get() == 0.0) {
    //   m_periodicIO.intake_pivot_voltage = 0.0;
    // }

    // Intake control
    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
    putString("State", m_periodicIO.intake_state.toString());
  }

  @Override
  public void writePeriodicOutputs() {
    mPivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);

    mIntakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void stop() {
    m_periodicIO.intake_pivot_voltage = 0.0;
    m_periodicIO.intake_speed = 0.0;
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    putNumber("Pivot/Abs Enc (get)", m_pivotEncoder.getPosition());
    // putNumber("Pivot/Abs Enc (getAbsolutePosition)", m_pivotEncoder.getAbsolutePosition());
    putNumber("Pivot/Abs Enc (getPivotAngleDegrees)", getPivotAngleDegrees());
    putNumber("Pivot/Setpoint", pivotTargetToAngle(m_periodicIO.pivot_target));

    putNumber("Pivot/Power", m_periodicIO.intake_pivot_voltage);
    putNumber("Pivot/Current", mPivotMotor.getOutputCurrent());

    putBoolean("Limit Switch", getIntakeHasNote());
  }

  @Override
  public void reset() {
  }

  public double pivotTargetToAngle(PivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_pivotAngleGround;
      case SOURCE:
        return Constants.Intake.k_pivotAngleSource;
      case AMP:
        return Constants.Intake.k_pivotAngleAmp;
      case STOW:
        return Constants.Intake.k_pivotAngleStow;
      default:
        // "Safe" default
        return 180;
    }
  }

  public double intakeStateToSpeed(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case PULSE:
        // Use the timer to pulse the intake on for a 1/16 second,
        // then off for a 15/16 second
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) {
          return Constants.Intake.k_intakeSpeed;
        }
        return 0.0;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        // "Safe" default
        return 0.0;
    }
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public IntakeState getIntakeState() {
    return m_periodicIO.intake_state;
  }

  public double getPivotAngleDegrees() {
    double value = m_pivotEncoder.getPosition() ;

        // double value = m_pivotEncoder.getAbsolutePosition() -
        // Constants.Intake.k_pivotEncoderOffset + 0.5;

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public boolean getIntakeHasNote() {
    // NOTE: this is intentionally inverted, because the limit switch is normally
    // closed
    return !m_IntakeLimitSwitch.get();
  }

  // Pivot helper functions
  public void goToGround() {
    m_periodicIO.pivot_target = PivotTarget.GROUND;
    m_periodicIO.intake_state = IntakeState.INTAKE;
    m_leds.setColor(Color.kYellow);
  }

  public void goToSource() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToAmp() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void goToStow() {
    m_periodicIO.pivot_target = PivotTarget.STOW;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (m_periodicIO.pivot_target == PivotTarget.GROUND && getIntakeHasNote() && isPivotAtTarget()) {
      m_periodicIO.pivot_target = PivotTarget.STOW;
      m_periodicIO.intake_state = IntakeState.NONE;
      m_leds.setColor(Color.kGreen);
    }
  }

  private boolean isPivotAtTarget() {
    return Math.abs(getPivotAngleDegrees() - pivotTargetToAngle(m_periodicIO.pivot_target)) < 5;
  }
}
