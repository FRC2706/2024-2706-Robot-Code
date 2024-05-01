package frc.robot.commands.auto;

import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.AnalogSelectorSubsystem;

public class AutoSelector {    
    private IntegerEntry entryChooseAuto;

    public AutoSelector() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("SelectAuto");
        entryChooseAuto = table.getIntegerTopic("Auto Index").getEntry(-1);
        entryChooseAuto.setDefault(-1);
    }

    public int getAutoId() {
        if (entryChooseAuto.get() == -1 || DriverStation.isFMSAttached()) {
            return getAnalogSelectorIndex();
        } else {
            int index = (int) entryChooseAuto.get();
            if (index == -1) {
                return 0;
            } else {
                return index;
            }
        }       
    }

    public int getAnalogSelectorIndex() {
        System.out.println("Selector Switch used for auto");
        try {
            // Get value from selector
            int index = AnalogSelectorSubsystem.getInstance().getIndex();

            // If m_analogSelectorIndex is -1, set it to 0.
            if(index == -1) {
                return 0;
            } else {
                return index;
            }
        } catch (Exception e) {
            return 0;
        }
    }
    
}