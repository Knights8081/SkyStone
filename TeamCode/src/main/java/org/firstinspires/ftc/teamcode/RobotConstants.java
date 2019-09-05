package org.firstinspires.ftc.teamcode.pd.constants;

/**
 * This in Not and opmode
 * List of constants for the robot.
 * Contains Arm, Servo, and Idol Hand Constants.
 *
 * Pull by (RobotConstants.NAME)
 *
 * @author Luke Frazer
 */
public class RobotConstants {

    /* Servo Constants */
    public static final double CLAW_MIN_RANGE       =   0.05;
    public static final double CLAW_MAX_RANGE       =   1.3;
    public static final double CLAW_HOME            =   0.08;
    public static final double CLAW_SPEED           =   0.05;

    /* Idol Hand Constants */
    public static final double IDOLHAND_MIN_RANGE   =   0.05;
    public static final double IDOLHAND_MAX_RANGE   =   1.3;

    /* Autonomous Constants */
    public static final double DRIVE_GEAR_REDUCTION    = 1.667 ;     // This is < 1.0 if geared UP (was .667)
    public static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double COUNTS_PER_MOTOR_REV    = 560 ;
    public static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double DRIVE_SPEED             = 0.15;
    public static final double FAST_DRIVE             = 0.3;
    public static final double TURN_SPEED              = 0.15;
    public static final double FAST_TURN              = 0.3;


}
