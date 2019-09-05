package org.firstinspires.ftc.teamcode.test.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.sun.tools.javac.util.Position;

//import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.pd.movement.DrivingMove;
import com.qualcomm.robotcore.hardware.TouchSensor;



/**
 * Created 10/18/2017
 *
 * This Opmode is used to control the robot during the teleOp period of the competition
 *
 * @author Anna Field
 */
@TeleOp(name="Nut: MechanumWheelsOnePad_NEW", group="Nut")


public class MechanumWheelsOnePad_NEW extends OpMode {

    private final HardwareNut robot = new HardwareNut();
    private ElapsedTime runtime = new ElapsedTime();
//    reference for robot hardware

    /* Game pad controller reference declarations */

    /**
     * Code to run ONCE when the driver hits INIT
     */
//    DigitalChannel digitalTouch;  // Hardware Device Object

    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Reset encoders
        turnOnStopAndReset();

        //Turn on RUN_USING_ENCODER
        turnOnRunUsingEncoder();



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Dingus");

    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//        Move.runMovementTest(robot);  //This is a test function that only is run for debugging issues involving movement

    }
    private void turnOnRunUsingEncoder() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnOnStopAndReset() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        /* CONTROL THE Collector --------------------------------------------------------------------*/
        if (gamepad2.left_bumper) {
            robot.getLeftCollect().setPosition(.5);
        }

        if (gamepad2.right_bumper) {
            robot.getRightCollect().setPosition(.5);
        }

        else if (gamepad2.left_trigger > .1) {
            robot.getLeftCollect().setPosition(2);
        }

        else if (gamepad2.right_trigger > .1) {
            robot.getRightCollect().setPosition(-2);
        }

        else if (gamepad2.x) {
            robot.getLeftCollect().setPosition(-2);
            robot.getRightCollect().setPosition(2);
        }

        if (gamepad1.left_bumper) {
            robot.getBasearm().setPower(-.7);
        }
        else if (gamepad1.right_bumper) {
            robot.getBasearm().setPower(.7);
        }
        else {
            robot.getBasearm().setPower(0);
        }

        if (gamepad1.right_trigger > .1) {
            robot.getLiftArm().setPower(.7);
        }
        else if (gamepad1.left_trigger > .1) {
            robot.getLiftArm().setPower(-.7);
        }
        else {
            robot.getLiftArm().setPower(0.0);
        }

        if (gamepad2.dpad_up) {
            robot.getRightCollect().setPosition(2);
        }

        else if (gamepad2.dpad_down) {
            robot.getRightCollect().setPosition(.5);
        }

        if (gamepad2.dpad_right) {
            robot.getCollector().setPower(-.9);
        }

        else if (gamepad2.dpad_left) {
            robot.getCollector().setPower(.9);
        }
        else {
            robot.getCollector().setPower(0);
        }
        if (gamepad1.dpad_right) {
            robot.getpin().setPosition(-2);
        }
        else if (gamepad1.dpad_left) {
            robot.getpin().setPosition(2);
        }
        if (gamepad1.a) {
            robot.getGatelatch().setPosition(-2);
        }
        else if (gamepad1.b) {
            robot.getGatelatch().setPosition(2);
        }

        if (gamepad2.a) {
            robot.getClawLift().setPower(.8);
        }
        else if (gamepad2.y) {
            robot.getClawLift().setPower(-.8);
        }
        else {
            robot.getClawLift().setPower(0);
        }

        /* DRIVE ROBOT --------------------------------------------------------------------------*/
        double drive = -gamepad1.left_stick_y;   // Power for forward and back motion; Negative because the gamepad is weird
        double strafe = gamepad1.left_stick_x;  // Power for left and right motion
        double rotate = gamepad1.right_stick_x;  // Power for rotating the robot

        DrivingMove.drive(robot, drive, strafe, rotate);
    }
}
