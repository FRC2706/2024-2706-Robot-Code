package frc.robot.StateMachines.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib6328.LoggedTunableNumber;
import frc.robot.Config;
import frc.robot.Mechanisms.Shooter.ShooterIO;
import frc.robot.Mechanisms.Shooter.ShooterIOValuesAutoLogged;

import org.littletonrobotics.junction.Logger;


public class Shooter extends SubsystemBase {
    private ShooterIO shooterIO;
    private ShooterIOValuesAutoLogged shooterValues; 

    private ShooterState shooterState = new ShooterState();

    private LoggedTunableNumber kP =
        new LoggedTunableNumber("Shooter/kP", Config.ShooterConstants.kP);
    private LoggedTunableNumber kI =
        new LoggedTunableNumber("Shooter/kI", Config.ShooterConstants.kI);
    private LoggedTunableNumber kD =
        new LoggedTunableNumber("Shooter/kD", Config.ShooterConstants.kD);
    private LoggedTunableNumber kFF =
        new LoggedTunableNumber("Shooter/kFF", Config.ShooterConstants.kFF);
    private LoggedTunableNumber setPointRPM =
        new LoggedTunableNumber("Shooter/setPointRPM", 0);

    public Shooter(ShooterIO io) {
        System.out.println("[Init]Creating Shooter BAE");
        shooterIO = io;
        shooterIO.setPID(kP.get(), kI.get(), kD.get());
        shooterIO.setFF(kFF.get());
    }

    @Override
    public void periodic(){
        LoggedTunableNumber.ifChanged(hashCode(), 
        ()->shooterIO.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(), 
        ()->shooterIO.setFF(kFF.get()), kFF);

        shooterIO.updateValues(shooterValues);
        Logger.processInputs("Shooter", shooterValues);

        if(DriverStation.isDisabled()){
            shooterIO.stop();
        } 

        
        Logger.recordOutput("Shooter/AppliedVolts", shooterValues.flywheelAppliedVolts);
        Logger.recordOutput("Shooter/OutputCurrent", shooterValues.flywheelOutputCurrent);
        Logger.recordOutput("Shooter/PositionRotations", shooterValues.flywheelPositionRotations);
        Logger.recordOutput("Shooter/VelocityRPM", shooterValues.flywheelVelocityRPM);
    }

    public void setSetPoint(double RPM){
        if(Config.LoggingConstants.tuningMode){
            shooterIO.setRPM(setPointRPM.get());
        }else{
            shooterIO.setRPM(RPM);
        }
    }
}
