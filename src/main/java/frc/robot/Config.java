package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

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
   * ID 2: Poseidon (Charged Up) (NEEDS UPDATE ON robot.conf)
   * ID 3: Clutch (Rapid React) (NEEDS UPDATE ON robot.conf)
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
    public static final double driveKP = 0.2;
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
      SLOW(1.5, 1.5*Math.PI),
      MAX(3.0, 3.0*Math.PI);

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
}