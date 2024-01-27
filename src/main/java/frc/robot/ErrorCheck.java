package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class ErrorCheck {
    public static int count = -1;
    public static boolean errREV(REVLibError error) {
        if (error == REVLibError.kOk) {
            return true;
        }
    DriverStation.reportError("REV DEVICE Error" + error.toString(), true);
    return false;
    }
public static boolean errCTRE(ErrorCode error) {
    if (error == ErrorCode.OK) {
        return true;
    }

    DriverStation.reportError("CTRE Device Error: " + error.toString(), true);
    return false;
    }   
}
