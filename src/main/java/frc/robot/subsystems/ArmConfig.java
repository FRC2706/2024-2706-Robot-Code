package frc.robot.subsystems;

import frc.robot.Config;

public class ArmConfig {
    public static final boolean SET_INVERTED = true;

    public static final int CURRENT_LIMIT = 60;

    //soft limit constant for bottom arm
    public static final float arm_forward_limit = (float)Math.toRadians(135);
    public static final float arm_reverse_limit = (float)Math.toRadians(40);
    public static final boolean SOFT_LIMIT_ENABLE = true;
    
    //PID constants
    public static final double arm_kP = 1.4;
    public static final double arm_kI = 0.0003;
    public static final double arm_kD = 0.9;
    public static final double arm_kIz = 0.3;
    public static final double arm_kFF = 0;
    public static final double min_output = -1;
    public static final double max_output = 1;


    //duty cycle encoders
    //public static final int bottom_duty_cycle_channel = 7;
    //public static final int top_duty_cycle_channel = 0;

    //arm offsets
    //public static final double bottom_arm_offset = 307.800000;
    //public static final double top_arm_offset = 0;

    //syncing encoders
    public static double ENCODER_SYNCING_TOLERANCE = 0.01; //radians

    //public static final double BOTTOM_NEO_GEAR_RATIO =  Config.robotSpecific(62.5,0.0,0.0,0.0);
    //public static final double TOP_NEO_GEAR_RATIO =  Config.robotSpecific(62.5,0.0,0.0,0.0);


    //public static final double bottomArmPositionConversionFactor = 2 * Math.PI / BOTTOM_NEO_GEAR_RATIO;
    //public static final double bottomArmVelocityConversionFactor = bottomArmPositionConversionFactor / 60.0;

    //public static final double topArmPositionConversionFactor = 2 * Math.PI / BOTTOM_NEO_GEAR_RATIO;
    //public static final double topArmVelocityConversionFactor = bottomArmPositionConversionFactor / 60.0;


    public static final double MAX_VEL = Math.PI * 3;
    public static final double MAX_ACCEL = Math.PI *3;
    
}
