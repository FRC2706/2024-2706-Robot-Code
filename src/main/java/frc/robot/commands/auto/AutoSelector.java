package frc.robot.commands.auto;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AnalogSelectorSubsystem;

public class AutoSelector {
    
    int m_analogSelectorIndex = 0;
    private AnalogSelectorSubsystem analogSelector = null; 

     
    public AutoSelector()
    {
        InitFrcDashboard();

        analogSelector = AnalogSelectorSubsystem.getInstance();        
    }

    void InitFrcDashboard() {
        //use FRC Labview Dashboard
        String[] autoList = {
        };
        SmartDashboard.putStringArray("Auto List", autoList );
    }

    public int getAutoId() {
        return getAnalogSelectorIndex();
    }

    public int getAnalogSelectorIndex() {
        DriverStation.reportWarning("Selector Switch used for auto", false);
        if (analogSelector != null){
            //Get value from selector
            m_analogSelectorIndex = analogSelector.getIndex();
            //If m_analogSelectorIndex is -1, set it to 0.
            if(m_analogSelectorIndex == -1)
            {
                m_analogSelectorIndex = 0;//do nothing
            }
        }
        else
        {
            m_analogSelectorIndex = 0;
        }

        return m_analogSelectorIndex;
    }
    
}