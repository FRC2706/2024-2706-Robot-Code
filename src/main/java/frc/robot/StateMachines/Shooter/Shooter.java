package frc.robot.StateMachines.Shooter;

import edu.wpi.first.math.MathUtil;
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
    private double tolerance = 10;//Check this value

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

        shooterState.isInRange(()->{
            if(MathUtil.isNear(shooterState.getDesiredVelocity(), shooterValues.flywheelVelocityRPM, tolerance))
            return true;
            else return false;
        });

        Logger.recordOutput("Shooter/CurrentState", shooterState.updateState());
    }

    /**
     * This should be called every loop cycle.
     * @param h the value of a hypotenouse drown from the robot distance 
     */
    public void setRobotDistance2Target(double h){
        shooterState.updateDistance(h);
    }

    public void setSetPoint(double RPM){
        if(Config.LoggingConstants.tuningMode){
            shooterIO.setRPM(setPointRPM.get());
        }else{
            shooterIO.setRPM(RPM);
        }
    }
}
