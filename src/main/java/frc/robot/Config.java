package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.lib3512.config.SwerveModuleConstants;

public final class Config {
  /**
   * Instructions for set up of robot.conf file on robot
   *
   * 0. Connect to the robot to the robot using a usb cable or the wifi network.
   * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local
   * (ssh admin@roboRIO-2706-FRC.local)
   * a. There is no password on a freshly flashed roboRIO
   * 2. Go up a directory (cd ..)
   * 3. cd into lvuser/ (cd lvuser/)
   * 4. Create a new file called robot.conf (touch robot.conf)
   * 5. Open the file with vi (vi robot.conf)
   * 6. Press i to enter insert mode
   * 7. Add an integer denoting the robot id. If it's the first robot, use 0,
   * second use 1 etc.
   * 8. Press [ESC] followed by typing :wq in order to save and quit
   * 9. To verify this worked type: more robot.conf
   * 10. If it displays the value you entered, it was successful
   * 11. Type exit to safely exit the ssh session
   */

  private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");

  /**
   * ID of the robot that code is running on
   */
  private static int robotId = -1;
  public static class CANID {
    public static int PIGEON = robotSpecific(30, 27, 27, 27, 30);

    // Arm Subsystem
    public static final int ARM_SPARK_CAN_ID = robotSpecific(5,0,0,0,0,18,18);
    //PCM Can ID 
    public static final int CTRE_PCM_CAN_ID = 1;
  }

  public static final int CANTIMEOUT_MS = 100;

  private static final int SIMULATION_ID = 1;
  /**
   * Returns one of the values passed based on the robot ID
   *
   * @param first The first value (default value)
   * @param more  Other values that could be selected
   * @param <T>   The type of the value
   * @return The value selected based on the ID of the robot
   */
  @SafeVarargs
  public static <T> T robotSpecific(T first, T... more) {
    if (getRobotId() < 1 || getRobotId() > more.length) {
      return first;
    } else {
      return more[getRobotId() - 1];
    }
  }

  /**
   * Obtain the robot id found in the robot.conf file
   *
   * @return The id of the robot
   */
  public static int getRobotId() {

    if (robotId < 0) {
      // Backup in case the FMS is attached, force to comp robot
      if (DriverStation.isFMSAttached()) {
        robotId = 0;
      }

      // Set the Id to the simulation if simulating
      else if (RobotBase.isSimulation()) {
        robotId = SIMULATION_ID;

      // Not simulation, read the file on the roborio for it's robot id.
      } else {
        try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
          robotId = Integer.parseInt(reader.readLine());
        } catch (Exception e) {
          robotId = 0; // DEFAULT TO COMP ROBOT IF NO ID IS FOUND
        }
      }
    }

    return robotId;
  }

  /**
   * ROBOT IDs
   * 
   * ID 0: Competition Robot (Crescendo) (NEEDS UPDATE ON robot.conf)
   * ID 1: Simulation of Comp Robot (Crescendo in Simulation)
   * ID 2: Beetle (Test robot) (NEEDS UPDATE ON robot.conf)
   * ID 3: Poseidon (Charged up) (NEEDS UPDATE ON robot.conf)
   **/

   /** ADD CONSTANTS BELOW THIS LINE */

  public static final Boolean swerveTuning = true;
  
  public static final class Swerve {
    public static final double stickDeadband = 0.1;
 
    public static final int pigeonID = 30;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants Changed*/
    public static final double trackWidth = Units.inchesToMeters(25.787);
    public static final double wheelBase = Units.inchesToMeters(20.472);
    public static final double wheelDiameter = Units.inchesToMeters(3.884);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0);
    public static final double angleGearRatio = (12.8 / 1.0);

    public static final double synchTolerance = 3;

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation Changed */
    public static final double voltageComp = 11.0;

    /* Swerve Current Limiting, Changed */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 50;

    /* Angle Motor PID Values, Changed */
    public static final double angleKP = 1.0;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values, Changed*/
    public static final double driveKP = 0.0; //0.2
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values Changed*/
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.9;
    public static final double driveKA = 0.5;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 2 * Math.PI / angleGearRatio;

    /* Swerve Profiling Values Changed*/
    public static enum TeleopSpeeds {
      SLOW(0.5, 0.5*Math.PI),
      MAX(3.0, 4.0*Math.PI);

      public final double translationalSpeed;
      public final double angularSpeed;

      TeleopSpeeds(double translationalSpeed, double angularSpeed) {
        this.translationalSpeed = translationalSpeed;
        this.angularSpeed = angularSpeed;
      }
    }
    public static final double maxSpeed = 3.0; // meters per second
    public static final double maxAngularVelocity = Math.PI*3.0;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 Changed*/
    public static final class Mod0 {
      public static final int driveMotorID = 24;
      public static final int angleMotorID = 23;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(270.73);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 Changed*/
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 25;
      public static final int canCoderID = 6;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(159.3);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 Changed*/
    public static final class Mod2 {
      public static final int driveMotorID = 20;
      public static final int angleMotorID = 26;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(194.9);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 Changed*/
    public static final class Mod3 {
      public static final int driveMotorID = 27;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 7;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(8.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
 
  public static final class AutoConstants {
    //Changed
    public static final double kMaxSpeedMetersPerSecond = 3; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    //Changed values

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1.35;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
   public static final class BlingConstants {
    public static int CANDLE = 15;
  }

    public static final class Intake {
      public static final int INTAKE = -1;
   }
  public class ArmConfig {
    public static final boolean SET_INVERTED = true;
    public static final boolean setInvered = true;
    public static final boolean INVERT_ENCODER = true;

    public static final int CURRENT_LIMIT = 60;

    
    public static final double MAX_ARM_ANGLE_DEG = 135;
    public static final double MIN_ARM_ANGLE_DEG = 40;

    //soft limit constant for bottom arm
    public static final float arm_forward_limit = (float)Math.toRadians(MAX_ARM_ANGLE_DEG);
    public static final float arm_reverse_limit = (float)Math.toRadians(MIN_ARM_ANGLE_DEG);
    public static final boolean SOFT_LIMIT_ENABLE = true;
    
    //PID constants
    public static final double arm_kP = 1.4;
    public static final double arm_kI = 0.0003;
    public static final double arm_kD = 0.9;
    public static final double arm_kIz = 0.3;
    public static final double arm_kFF = 0;
    public static final double min_output = -1;
    public static final double max_output = 1;

    //ff calculations 
    public static final double gravitationalConstant = 389.0886; //inches/s/s which is equal to 9.81 m/s/s
    public static final double ARM_FORCE = 11.29 *gravitationalConstant; //11.29 lb

    public static final double LENGTH_ARM_TO_COG = 14.56;

    //arm offsets
    //public static final double bottom_arm_offset = 307.800000;
    //public static final double top_arm_offset = 0;

    //syncing encoders
    public static double ENCODER_SYNCING_TOLERANCE = 0.01; //radians


    public static final double ARM_ENCODER_GEAR_RATIO =  1;

    //arm position unit: radians
    public static final double armPositionConversionFactor = 2 * Math.PI / ARM_ENCODER_GEAR_RATIO;
    //arm velocity unit: radians/sec
    public static final double armVelocityConversionFactor = armPositionConversionFactor / 60.0; 
    //offset unit: degrees
    public static final double armAbsEncoderOffset = 27;

    public static final double MAX_VEL = Math.PI * 0.5;
    public static final double MAX_ACCEL = Math.PI * 0.5;

    public static final double MOMENT_TO_VOLTAGE = 0.000005;    
}

    
      //Constants for arm pneumatics 
      public static final int ARMLOW_PNEUMATIC_FORWARD_CHANNEL = 0;
      public static final int ARMLOW_PNEUMATIC_REVERSE_CHANNEL = 1;


          /**
     * Differential Drive Constants
     */
    public static class DIFF {

           // Differential Drive CAN IDs
        public static int DIFF_LEADER_LEFT = robotSpecific(-01, 6, 2, 5, -01, 35);
        public static int DIFF_LEADER_RIGHT = robotSpecific(-01, 3, 1, 3, -01, 33);
        public static int DIFF_FOLLOWER_LEFT = robotSpecific(-01, 5, -1, 7, -01, 37);
        public static int DIFF_FOLLOWER_RIGHT = robotSpecific(-01, 2, -1, 9, -01, 39);

        public static boolean ISNEOS = robotSpecific(true, false, false, false);
        public static boolean HAS_FOLLOWERS = robotSpecific(true, true, false, true, true);
        public static boolean LEFT_FOLLOWER_ISVICTOR = robotSpecific(false, true, false, true);
        public static boolean RIGHT_FOLLOWER_ISVICTOR = robotSpecific(false, true, false, true);
    
        // Invert motors to consider forward as forward (same practice for all objects)
        public static boolean LEADER_LEFT_INVERTED = robotSpecific(false, false, false, false, false, false);
        public static boolean LEADER_RIGHT_INVERTED = robotSpecific(false, false, true, true, false, true);
        public static boolean FOLLOWER_LEFT_INVERTED = robotSpecific(false, false, false, false, false, false);
        public static boolean FOLLOWER_RIGHT_INVERTED = robotSpecific(false, false, false, true, false, false);
    
        public static boolean LEFT_SENSORPHASE = robotSpecific(false, true, true, true);
        public static boolean RIGHT_SENSORPHASE = robotSpecific(false, false, true, true);
    
        // Current limiter Constants
        public static boolean TALON_CURRENT_LIMIT = true;   //Enable or disable motor current limiting.
        public static int TALON_PEAK_CURRENT_AMPS = 80;           //Peak current threshold to trigger the current limit
        public static int TALON_PEAK_TIME_MS = 250;               //Time after current exceeds peak current to trigger current limit
        public static int TALON_CONTIN_CURRENT_AMPS = 40;         //Current to mantain once current limit is triggered 
        
        // Drivetrain idle mode and voltage/current limits
        public static int NEO_RAMSETE_CURRENTLIMIT = 40;
        public static int NEO_DRIVER_CURRENTLIMIT = 80;

        public static IdleMode TELEOP_IDLEMODE = IdleMode.kBrake; 
        public static NeutralMode TELEOP_NEUTRALMODE = NeutralMode.Brake;

        public static IdleMode AUTO_IDLEMODE = IdleMode.kBrake; 
        public static NeutralMode AUTO_NEUTRALMODE = NeutralMode.Brake;

        public static double BRAKE_IN_DISABLE_TIME = 2.0;
    }
    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;
    public static Double DRIVER_JOYSTICK_DEADBAND = 0.1; // TODO: Investigate if this can be better tuned
}