package frc.robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

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
}
