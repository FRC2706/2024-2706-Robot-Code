package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Config;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {

    private DigitalInput sensor7; //DigitInput instance
    private DigitalInput sensor8;
    private DigitalInput sensor9;

    Debouncer sensor7Debouncer;
    Debouncer sensor8Debouncer;
    Debouncer sensor9Debouncer;

    private BooleanPublisher sensor7Pub;
    private BooleanPublisher sensor8Pub;
    private BooleanPublisher sensor9Pub;
    
    private boolean sensor7Result;
    private boolean sensor8Result;
    private boolean sensor9Result;

    private static final IntakeSubsystem INSTANCE_INTAKE = new IntakeSubsystem();

    private CANSparkMax m_intake;
    int targetSpeed = 0;
    double kMaxOutput = 1;
    double kMinOutput = -1;

    public boolean isActive() {
        return m_intake != null;
    }

    public static IntakeSubsystem getInstance() {
        if (INSTANCE_INTAKE.isActive())
            return INSTANCE_INTAKE;
        else
            return null;
    }


    private IntakeSubsystem() {
        if (Config.Intake.INTAKE != -1) {
            initializeSubsystem();
        } else {
            m_intake = null;
        }
    }

    private void initializeSubsystem() {
        
        m_intake = new CANSparkMax(Config.Intake.INTAKE, MotorType.kBrushless);
        m_intake.restoreFactoryDefaults();
        m_intake.setInverted(true);
        m_intake.setSmartCurrentLimit(70);

        sensor7 = new DigitalInput(7);
        sensor8 = new DigitalInput(8);
        sensor9 = new DigitalInput(9);

        sensor7Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        sensor8Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
        sensor9Debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

        NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("sensors");

        sensor7Pub = sensorTable.getBooleanTopic("sensor 7 result").publish(PubSubOption.periodic(0.02));
        sensor8Pub = sensorTable.getBooleanTopic("sensor 8 result").publish(PubSubOption.periodic(0.02));
        sensor9Pub = sensorTable.getBooleanTopic("sensor 9 result").publish(PubSubOption.periodic(0.02));

    }

    public boolean isSensor7True() {
        return sensor7Result;
    }
    
    public boolean isSensor8True() {
        return sensor8Result;
    }

    public boolean isSensor9True() {
        return sensor9Result;
    }


    public void setMotorRPM(Double speed) {
        m_intake.set(speed);
    }

    public void setVoltage(double voltage){
        m_intake.setVoltage(voltage);
    }
    @Override
    public void periodic() {
        sensor7Result = sensor7Debouncer.calculate(!sensor7.get());
        sensor8Result = sensor8Debouncer.calculate(!sensor8.get());
        sensor9Result = sensor9Debouncer.calculate(!sensor9.get());

        sensor7Pub.accept(sensor7Result);
        sensor8Pub.accept(sensor8Result);
        sensor9Pub.accept(sensor9Result);
        
    }

    public void stop()
    {
        m_intake.stopMotor();
    }

}