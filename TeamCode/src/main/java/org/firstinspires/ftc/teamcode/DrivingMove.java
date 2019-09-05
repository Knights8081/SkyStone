package org.firstinspires.ftc.teamcode.pd.movement;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;

/**
 * This class holds the intelligence that drives the robot.
 *
 * @author Luke Frazer
 */
public final class DrivingMove {
    /**
     * This function is used to determine the values of each motor based on the given inputs
     *  and set the powers of those motors for the robot
     * @param robot the robot that is running
     * @param drive the value given by the left stick y axis
     * @param strafe the value given by the left stick x axis
     * @param rotate the value given by the right stick x axis
     */
    public static void drive(final HardwareNut robot, final double drive, final double strafe, final double rotate) {
        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        robot.getLeftDrive().setPower(frontLeftPower);
        robot.getRightDrive().setPower(frontRightPower);
        robot.getLeftArm().setPower(backLeftPower);
        robot.getRightArm().setPower(backRightPower);
    }
}