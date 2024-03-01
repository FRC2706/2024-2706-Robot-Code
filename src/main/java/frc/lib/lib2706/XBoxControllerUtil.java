package frc.lib.lib2706;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxControllerUtil {
    /**
     * This file should not be constructed. It should only have static factory methods.
     */
    private XBoxControllerUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Create a Trigger that works when the DownLeft, Left or UpLeft POV button is pressed.
     * This accounts for the fact that POV buttons are easy to press the wrong button.
     * 
     * @param controller to create the trigger from.
     * @return Trigger that works for the DownLeft, Left or UpLeft buttons
     */
    public static Trigger leftPOV(CommandXboxController controller) {
        return controller.povDownLeft().or(controller.povLeft().or(controller.povUpLeft()));
    }

    /**
     * Create a Trigger that works when the DownRight, Right or UpRight POV button is pressed.
     * This accounts for the fact that POV buttons are easy to press the wrong button.
     * 
     * @param controller to create the trigger from.
     * @return Trigger that works for the DownRight, Right or UpRigh buttons
     */
    public static Trigger rightPOV(CommandXboxController controller) {
        return controller.povDownRight().or(controller.povRight().or(controller.povUpRight()));
    }
}
