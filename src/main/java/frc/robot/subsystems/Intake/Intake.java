package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib6328.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;


public class Intake extends SubsystemBase {
    private IntakeIO intakeIO;
    private IntakeIOValuesAutoLogged intakeIOValues = new IntakeIOValuesAutoLogged(); 
    private double tolerance = 10;//Check this value
    private double setV = 0;
    public static double TUNING_MODE = -1;


    private LoggedTunableNumber setVoltage =
        new LoggedTunableNumber("Intake/setVoltage", 0);

    public Intake(IntakeIO io) {
        System.out.println("[Init]Creating Intake");
        intakeIO = io;
    }

    @Override
    public void periodic(){
        LoggedTunableNumber.ifChanged(hashCode(), ()->{setV = setVoltage.get(); System.out.println("Voltage changed" + setV);}, setVoltage);

        intakeIO.updateValues(intakeIOValues);
        Logger.processInputs("Intake", intakeIOValues);

        if(DriverStation.isDisabled()){
            intakeIO.stop();
        } 
    }

    public void stop(){
        intakeIO.stop();
    }

    public void setSetV(double voltage){
        if(voltage == TUNING_MODE){
            intakeIO.setVoltage(setV);
        }else{
            intakeIO.setVoltage(voltage);
        }        
    }
}
