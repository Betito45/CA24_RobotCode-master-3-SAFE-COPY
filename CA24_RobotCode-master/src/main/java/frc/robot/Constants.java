package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
  public static class Robot {
    public static final double k_width = 26; // Inches
    public static final double k_length = 28; // Inches
  }

  public static class Intake {
    // Motors
    public static final int kIntakeMotorId = 9;
    public static final int kPivotMotorId = 10;

    // DIO
    public static final int k_pivotEncoderId = 0;
    public static final int k_intakeLimitSwitchId = 2;

    // Absolute encoder offset
    public static final double k_pivotEncoderOffset = 0.166842; // Straight up, sketchy to reset to "up"

    // Pivot set point angles
    public static final double k_pivotAngleGround = 60;
    public static final double k_pivotAngleSource = 190;
    public static final double k_pivotAngleAmp = k_pivotAngleSource;
    public static final double k_pivotAngleStow = 275;

    // Intake speeds
    public static final double k_intakeSpeed = 0.7;
    public static final double k_ejectSpeed = -0.45;
    public static final double k_feedShooterSpeed = -0.5;
  }

  // PCM
  public static final int kPCMId = 0;
  public static final int kIntakeSolenoidForwardId = 2;

  // DIO

  // Shooter
  public static final int kShooterLeftMotorId = 12;
  public static final int kShooterRightMotorId = 13;

  public static final double kShooterP = 0.00005;
  public static final double kShooterI = 0.0;
  public static final double kShooterD = 0.0;
  public static final double kShooterFF = 0.0002;

  public static final double kShooterMinOutput = 0;
  public static final double kShooterMaxOutput = 1;

  // Climber
  public static final int kClimberLeftMotorId = 14;
  public static final int kClimberRightMotorId = 15;
  // public static final double kClimberClimbSpeed = 600.0; // RPM
  // public static final double kClimberReleaseSpeed = -600.0; // RPM

  public static final double kClimberClimbSpeed = 0.5; // percent
  public static final double kClimberReleaseSpeed = -0.5; // percent

  public static final double kClimberGearRatio = 1.0 / 12.0;

  public static final double kClimberP = 0.001;
  public static final double kClimberI = 0.0;
  public static final double kClimberD = 0.0;
  public static final double kClimberMinOutput = -0.5;

  public static final double kClimberMaxOutput = 0.5;

  public static final class ElectronicsIDs {

    public static final int DriverControllerPort = 1;
    public static final int OperatorControllerPort = 2;

    /***************************** DRIVE *****************************/

    // Encoder = 1{locationOnBot}
    public static final int FrontLeftTurnEncoderID = 10;
    public static final int FrontRightTurnEncoderID = 12;
    public static final int BackLeftTurnEncoderID = 9;
    public static final int BackRightTurnEncoderID = 11;

    // DriveMotorID = 2{locationOnBot} // Base
    public static final int FrontLeftDriveMotorID = 8;
    public static final int FrontRightDriveMotorID = 2;
    public static final int BackLeftDriveMotorID = 6;
    public static final int BackRightDriveMotorID = 4;

    // TurnMotorID = 3{locationOnBot} // Side
    public static final int FrontLeftTurnMotorID = 7;
    public static final int FrontRightTurnMotorID = 1;
    public static final int BackLeftTurnMotorID = 5;
    public static final int BackRightTurnMotorID = 3;

 
}

  // Drivetrain
    public static final class DriveConstants {
        /** radians */
        public static final double MaxAngularSpeed = Math.PI; // 1/2 rotation per second

        public static final boolean GyroReversed = false;

        public static final double TrackWidth = Units.inchesToMeters(24); // Distance between left and right wheels
        public static final double WheelBase = Units.inchesToMeters(24); // Distance between front and back wheels
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(WheelBase / 2, TrackWidth / 2), // front left
                new Translation2d(WheelBase / 2, -TrackWidth / 2), // front right
                new Translation2d(-WheelBase / 2, TrackWidth / 2), // back left
                new Translation2d(-WheelBase / 2, -TrackWidth / 2) // back right
        );

        public static final double MaxAcceleration = Units.feetToMeters(36.24); // m/sec^2 from Mr. K's spreadsheet
        public static final double MaxDriveableVelocity = 3.6; // m/s? (CHANGE if this is the wrong unit)

        public static final double PhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(15.1); // 15.1 f/s from Mr. K's
                                                                                               // spreadsheet
        public static final double PhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI; // CHANGE

        public static final double FrontLeftMagnetOffset = -0.197021;
        public static final double FrontRightMagnetOffset = -0.052979;
        public static final double BackLeftMagnetOffset = -0.264893;
        public static final double BackRightMagnetOffset = -0.135254;
        //The comment below is incase we need to reset to default swerve values (DO NOT DELETE)
       // public static final double FrontLeftMagnetOffsetInRadians = 1.5171039327979088;
       // public static final double FrontRightMagnetOffsetInRadians = 1.7456666082143784;
        //public static final double BackLeftMagnetOffsetInRadians = -2.7626938149333;
        //public static final double BackRightMagnetOffsetInRadians = -2.305568464100361;

        public static final double TimestepDurationInSeconds = 0.02;
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        public static final double maxModuleSpeed = 4.5; // M/S



        public static final double WheelRadius = Units.inchesToMeters(2.0);
        public static final double WheelCircumference = 2.0 * WheelRadius * Math.PI;
        public static final double GearRatio = 6.75;
        public static final double VelocityConversionFactor = WheelCircumference / 60
                / GearRatio;

        public static final double MaxRPM = 5820;
        public static final double MaxVelocityPerSecond = MaxRPM * VelocityConversionFactor;

        /* DRIVE ENCODER */
        public static final double DriveKP = 0.04; // REQUIRES TUNING
        public static final double DriveKI = 0.0015;
        public static final double DriveKD = 0;
        public static final double DriveIZone = 0.15;
        public static final double DriveFF = 1.0 / MaxVelocityPerSecond;

        public static final double AutoAlignKP = 0.1; //CHANGE
        public static final double AutoAlignKI = 0.0015;
        public static final double AutoAlignKD = 0;

        /* TURN ENCODER */
        public static final int CANCoderResolution = 4096;
        public static final double PositionConversionFactor = WheelCircumference / GearRatio;
        public static final double TurnKP = 1; // Need to change
        public static final double TurnKI = 0;
        public static final double TurnKD = 0;

        public static final double ModuleMaxAngularVelocity = 3.0 * 2.0 * Math.PI; // #revolutions * radians per
                                                                                   // revolution (rad/sec)
        public static final double ModuleMaxAngularAcceleration = 12 * Math.PI; // radians per second squared

        public static final int StallLimit = 40;
        public static final int FreeLimit = 40;
    }

  public static class Field {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    // TODO: Add left and right subwoofer starting poses
    public static final Pose2d redCenterPose2d = new Pose2d(15.19, 5.50, new Rotation2d(Units.degreesToRadians(180.0)));
    public static final Pose2d blueCenterPose2d = new Pose2d(1.27, 5.50, new Rotation2d(0));
  }

  public static class LEDs {
    public static final int k_PWMId = 0;
    public static final int k_totalLength = 300;
  }
}
