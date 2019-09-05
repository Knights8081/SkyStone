/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.test.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;
import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.constants.RobotConstants;


import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Auto_RR_Crater_NEW", group ="Concept")

public class Auto_RR_Crater_NEW extends LinearOpMode {

    private static final String VUFORIA_KEY = "AXe+G0X/////AAAAGWX2PnVHaUK0lbybIB3Luad39Qzmm36RMGPUaiMP4ybPeYkYi7gHFkzHdn5vRGY+J7+0ZFIAVXqKf4PgJP8TpGTqnw8HM/aCP3BpmmYd5nmJxfwxQxzwydbjTovUJGcGqm+FBJVI93jCnHT68WPIv0Aki5yLpXXU6FwlYuzNMixa7FOTIdew60kBpM70uhIX8Y5yx+GKTB2WL3Sl0UKPE7jwWep0BsQe0gidcJdMXQAVU13jBrpXBKUSmKIuuQhWFUax9d/yaOns9mheTCH9S4RgQ96Ln4LwB0SzpIB2/rgQHkSgm7MWmCzsDn95/hRMYhSH82CevjgHqvTvRNIqwZ+caom47CVfgxzmi2iJNu+d";

    private final HardwareNut robot = new HardwareNut();   // Use a hardwareNut
    private final ConceptTensorFlowObjectDetection Tensor = new ConceptTensorFlowObjectDetection();

    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        //Reset encoders
        turnOnStopAndReset();

        //Turn on RUN_USING_ENCODER
        turnOnRunUsingEncoder();

        robot.init(hardwareMap);
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        robot.getGatelatch().setPosition(2);
        sleep(3000);
        robot.getpin().setPosition(-2);
        sleep(1000);
        encoderDrive(RobotConstants.FAST_DRIVE, 10,10,10);
        encoderDrive(RobotConstants.FAST_TURN, -6,6,10);
        robot.getCamera().setPosition(2);


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }


            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() > 0) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();

                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                            if (goldMineralX != -1) {
                                telemetry.addData("Gold Mineral Position", "YE");
                                sleep(500);
                                First(10);

                            } else if (silverMineral1X != -1 || silverMineral2X != -1) {
                                telemetry.addData("Gold Mineral Position", "NAW");
                                sleep(500);
                                Second(10);
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }


        if (tfod != null)

        {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    public void First ( double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        tfod.deactivate();
        robot.getCamera().setPosition(-2);
        telemetry.addData("Gold Mineral Position", "YE");
        encoderDrive(RobotConstants.FAST_TURN, -8, 8, 10);
        encoderDrive(RobotConstants.FAST_DRIVE, 12, 12, 10);
        encoderDrive(RobotConstants.FAST_TURN, 8, -8, 10);
        encoderDrive(RobotConstants.FAST_DRIVE, 15, 15, 10);

//        encoderDrive(RobotConstants.FAST_DRIVE, -12, -12, 10);
//        encoderDrive(RobotConstants.FAST_TURN, -8, 8, 10);
//        encoderDrive(RobotConstants.FAST_DRIVE, 41, 41, 10);
//        encoderDrive(RobotConstants.FAST_TURN, -13, 13, 10);
//        encoderDrive(RobotConstants.FAST_DRIVE, 40, 40, 10);
//        robot.getRightCollect().setPosition(2);
//        robot.getCollector().setPower(.9);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//        sleep(1000);
//        encoderDrive(RobotConstants.FAST_DRIVE, -3, 3, 10);
//        encoderDrive(RobotConstants.FAST_DRIVE, -68, -68, 10);
//        encoderDrive(.05, -10, -10, 10);
        stop();
    }

    public void Second ( double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        encoderDrive(RobotConstants.FAST_TURN, 12, -12, 10);
        sleep(1000);
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() > 0) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();

                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if (goldMineralX != -1) {
                            tfod.deactivate();
                            robot.getCamera().setPosition(-2);
                            telemetry.addData("Gold Mineral Position", "YE");
                            encoderDrive(RobotConstants.FAST_TURN, -7, 7, 10);
                            encoderDrive(RobotConstants.FAST_DRIVE, 10, 10, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, -10, -10, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -19, 19, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 38, 38, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -16, 16, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 45, 45, 10);
//                            robot.getRightCollect().setPosition(2);
//                            robot.getCollector().setPower(.9);
//                            runtime.reset();
//                            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//                                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                                telemetry.update();
//                            }
//                            sleep(1000);
//                            encoderDrive(RobotConstants.FAST_TURN, -2, 2, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, -68, -68, 10);
//                            encoderDrive(.05, -10, -10, 10);
//                            robot.getRightClaw().setPosition(.5);
//                            sleep(1000);
                            stop();

                        } else if (silverMineral1X != -1 || silverMineral2X != -1) {
                            telemetry.addData("Gold Mineral Position", "NAW");
                            sleep(500);
                            Thrid(10);
                        }
                    }
                }
                telemetry.update();
            }
        }
    }


    public void Thrid ( double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        encoderDrive(RobotConstants.FAST_TURN, 9, -9, 10);
        sleep(500);
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() > 0) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();

                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
//                        if (goldMineralX != -1) {
//                            tfod.deactivate();
//                            robot.getCamera().setPosition(-2);
//                            telemetry.addData("Gold Mineral Position", "YE");
                            encoderDrive(RobotConstants.FAST_TURN, -3, 3, 10);
                            encoderDrive(RobotConstants.FAST_DRIVE, 12, 12, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, -14, -14, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -34, 34, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 41, 41, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -15, 15, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 44, 44, 10);
//                            robot.getRightCollect().setPosition(2);
//                            robot.getCollector().setPower(.9);
//                            runtime.reset();
//                            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//                                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//                                telemetry.update();
//                            }
//                            sleep(1000);
//                            encoderDrive(RobotConstants.FAST_TURN, -2, 2, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, -68, -68, 10);
//                            encoderDrive(.05, -10, -10, 10);
                            stop();

//                        } else if (silverMineral1X != -1 || silverMineral2X != -1) {
//                            tfod.deactivate();
//                            robot.getCamera().setPosition(-2);
//                            telemetry.addData("Gold Mineral Position", "YE");
//                            encoderDrive(RobotConstants.FAST_TURN, -3, 3, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 12, 12, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, -14, -14, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -34, 34, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 41, 41, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -13, 13, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, 44, 44, 10);
//                            encoderDrive(RobotConstants.FAST_TURN, -8, 8, 10);
//                            robot.getRightClaw().setPosition(2);
//                            sleep(1500);
//                            encoderDrive(RobotConstants.FAST_TURN, 7, -7, 10);
//                            encoderDrive(RobotConstants.FAST_DRIVE, -67, -67, 10);
//                            encoderDrive(.05, -3, -3, 10);
//                            stop();
//                        }
                    }
                }
                telemetry.update();
            }
        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }




    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int newLeftDriveTarget = robot.getLeftDrive().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
            int newRightDriveTarget = robot.getRightDrive().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);
            int newLeftArmTarget = robot.getLeftArm().getCurrentPosition() + (int) (leftInches * RobotConstants.COUNTS_PER_INCH);
            int newRightArmTarget = robot.getRightArm().getCurrentPosition() + (int) (rightInches * RobotConstants.COUNTS_PER_INCH);

            robot.getLeftDrive().setTargetPosition(newLeftDriveTarget);
            robot.getRightDrive().setTargetPosition(newRightDriveTarget);
            robot.getLeftArm().setTargetPosition(newLeftArmTarget);
            robot.getRightArm().setTargetPosition(newRightArmTarget);

            // Turn On RUN_TO_POSITION
            turnOnRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();
            setAllPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.TURN_SPEE
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && allMotorsBusy()) {

                // Display it for the driver.
                //              telemetry.addData("Path1",  "Running to %7d :%7d", newLeftDriveTarget,  newRightDriveTarget);
                //              telemetry.addData("Path2",  "Running at %7d :%7d",
                //                      robot.getLeftDrive().getCurrentPosition(),      Hid this so I could see the xy values
                //                      robot.getRightDrive().getCurrentPosition());
                //              telemetry.update();
            }

            // Stop all motion;
            setAllPower(0.0);

            // Turn off RUN_TO_POSITION
            turnOnRunUsingEncoder();

            //  sleep(250);   // optional pause after each move
        }
    }

    /**
     * Check if all motors are busy
     *
     * @return true if all motors are busy, false if any motor is not busy
     */
    private boolean allMotorsBusy() {
        return robot.getLeftDrive().isBusy() && robot.getRightDrive().isBusy() && robot.getLeftArm().isBusy() && robot.getRightArm().isBusy();
    }

    /**
     * Check if any motor is busy
     *
     * @return true if any motor is busy, false if all are not busy
     */
    private boolean anyMotorBusy() {
        return robot.getLeftDrive().isBusy() || robot.getRightDrive().isBusy() || robot.getLeftArm().isBusy() || robot.getRightArm().isBusy();
    }

    /**
     * Set all motors to mode: RUN_TO_POSITION
     */
    private void turnOnRunToPosition() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getRightArm().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Set all motors to mode: RUN_USING_ENCODER
     */
    private void turnOnRunUsingEncoder() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Set all motors to mode: STOP_AND_RESET_ENCODER
     */
    private void turnOnStopAndReset() {
        robot.getLeftDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightDrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLeftArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getRightArm().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Set the power of all motors to specified value
     *
     * @param power - value for power
     */
    private void setAllPower(final double power) {
        robot.getLeftDrive().setPower(power);
        robot.getRightDrive().setPower(power);
        robot.getLeftArm().setPower(power);
        robot.getRightArm().setPower(power);
    }

}

