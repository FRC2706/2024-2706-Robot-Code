package frc.lib.lib2706;

import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
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
        DriverStation.reportError(
            String.format(
                "[MergeError] - CANSparkMax error. MergeMessage: %s. Spark error code: %s.",
                error.toString()), 
            PRINT_STACK_TRACE);
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

        DriverStation.reportError(
            String.format(
                "[MergeError] - CTRE device error. MergeMessage: %s. CTRE error code: %s.",
                message,
                error.toString()), 
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

        DriverStation.reportError(String.format(
            "[MergeError] - CANSparkMax failed to configure setting. MergeMessage: %s. Spark error code: %s \nSee stack trace below.", 
            message,
            err.toString()), 
            PRINT_STACK_TRACE_CONFIGURE);
            
        return false;
    }

}
