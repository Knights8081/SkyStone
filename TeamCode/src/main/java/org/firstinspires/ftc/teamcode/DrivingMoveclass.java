package org.firstinspires.ftc.teamcode.test.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.movement.DrivingMove;
import org.firstinspires.ftc.teamcode.pd.old.robotclasshardware;

/**
 * This class holds the intelligence that drives the robot.
 *
 * @author Luke Frazer
 */
@TeleOp (name="Driving Move Class", group="PushBot")
@Disabled
public class DrivingMoveclass extends LinearOpMode {

    private final robotclasshardware robot = new robotclasshardware();        //reference for robot hardware


    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
      robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
        }

    }
}
