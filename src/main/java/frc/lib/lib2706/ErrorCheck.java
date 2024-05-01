package frc.lib.lib2706;

import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class ErrorCheck {
    private static final int MAXIMUM_RETRIES = 5;
    private static final boolean PRINT_STACK_TRACE = false;
    private static final boolean PRINT_STACK_TRACE_CONFIGURE = true;

    /**
     * Handle checking if a REVLibError is ok or needs to be printed to the console.
     * 
     * @param message A simple and short identifying message
     * @param error The error to check
     * @return True for no error, false means there is an error. Boolean can be ignored if not needed.
     */
    public static boolean errSpark(String message, REVLibError error) {
        if (error == REVLibError.kOk) {
            return true;
        }
        String msg = "[MergeError] - CANSparkMax error. MergeMessage:" + message;
        msg += " Spark error code: " + error.toString() + " \nSee stack trace below.";

        DriverStation.reportError(
            msg,
            PRINT_STACK_TRACE_CONFIGURE);
            
        return false;
    }

    /**
     * Handle checking if a CTRE device error is ok or needs to be printed to the console.
     * 
     * @param message A simple and short identifying message
     * @param error The error to check
     * @return True for no error, false means there is an error. Boolean can be ignored if not needed.
     */
    public static boolean errCTRE(String message, ErrorCode error) {
        if (error == ErrorCode.OK) {
            return true;
        }

        String msg = "[MergeError] - CTRE device error. MergeMessage: " + message;
        msg += " CTRE error code: " + error.toString() + " \nSee stack trace below.";

        DriverStation.reportError(
            msg,
            PRINT_STACK_TRACE);
            
        return false;
    }

    /**
     * Configure a SparkMax setting multiple times until it succeeds.
     * 
     * @param message A simple and short identifying message.
     * @param config The Supplier to call to configure which returns a REVLibError.
     * @return true for success, false for failure.
     */
    public static boolean configureSpark(String message, Supplier<REVLibError> config) {
        REVLibError err = REVLibError.kOk;
        for (int i = 0; i < MAXIMUM_RETRIES; i++) {
            err = config.get();
            if (err == REVLibError.kOk) {
                return true;
            }
        }

        String msg = "[MergeError] - CANSparkMax failed to configure setting. MergeMessage:" + message;
        msg += " Spark error code: " + err.toString() + " \nSee stack trace below.";

        DriverStation.reportError(
            msg,
            PRINT_STACK_TRACE_CONFIGURE);
            
        return false;
    }
    
    public static boolean errREV(REVLibError error) {
        if (error == REVLibError.kOk) {
            return true;
        }
        DriverStation.reportError("REV DEVICE Error" + error.toString(), true);
        return false;
    }

    /**
     * Run burn flash on all the given sparkmaxs.
     * 
     * @param sparkmaxNames A string to identify the sparkmaxs to run burnFlash on.
     * @param sparkmaxs The sparkmaxs to run burn flash on.
     * @return true for success, false for failure.
     */
    public static boolean sparkBurnFlash(String sparkmaxNames, CANSparkBase... sparkmaxs) {
        try {
            Thread.sleep(200);
        } catch (Exception e) {}

        boolean allOk = true;
        for (CANSparkBase sparkmax : sparkmaxs) {
            // Burn flash and record error
            REVLibError error = sparkmax.burnFlash();

            if (error != REVLibError.kOk) {
                allOk = false;
                
                String msg = "[MergeError] - CANSparkMax failed to burn flash on sparkmax(s): " + sparkmaxNames;
                msg += " Spark error code: " + error.toString() + " \nSee stack trace below.";

                DriverStation.reportError(
                    msg,
                    PRINT_STACK_TRACE_CONFIGURE);
            }
        }

        return allOk;
    }

}
