/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import android.hardware.camera2.CameraDevice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlowAuton6407Skystone", group = "Concept")
//@Disabled
public class TensorFlowAuton6407Skystone extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private enum GoldMineralLocation { LEFT, CENTER, RIGHT};
    private GoldMineralLocation mineralLocation;

    Hardware6407Skystone robot = new Hardware6407Skystone();   // Use a 6407's hardware

    ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV =  537.6;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.25;

    //static final double ENCODER_TUNING = .941176;
    static final double ENCODER_TUNING = 1;
    static final double waitTime = 20;



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
    private static final String VUFORIA_KEY = "Abi/xEj/////AAAAma5a1fl3IEVQqyNvKDCwEsU7alNBpuyGTQDqtsVY/vE2BDhnW/ry5nF/MIfr/DTpwVx5VOyjXbZ8zA1zG4uBQuSvSBDZavUQOtKonDkJfrQnMDhRpAYH/NyGiiu8sBnJe1VJDbRVQF0zcDvs4HbuviRX6aiF93VdO0WYpXJQ9k6x/zNNGpphx2FrvdJyOXYJDapOr7dnqFTTT3MEm3PazaVci+ABKdA7CBzB6A5JO3d7x1MQP/jxQIMphhgnwUQpCMbA1nYQm6aVtAFiMnlzU6u2s36rxjSMMpTA6ws4IAPjhvg7N4LcMawZ93llcyUNaFYNvI1Cc44SqWBoXYLv0UQthT9BAqJFtZaRcZBFA9zZ";

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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        boolean mineralFound = false;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        if (opModeIsActive()) {
            com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);

            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            robot.LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.LRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.RRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //CameraDevice.getInstance().setFlashTorchMode(true);

            while (opModeIsActive()) {

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null && !mineralFound) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
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
                            if (goldMineralX  ==  -1) {
                                mineralLocation = GoldMineralLocation.LEFT;
                                mineralFound = true;
                                telemetry.addData("Gold Mineral Position", "LEFT");
                            } else if (goldMineralX > silverMineral1X ) {
                                mineralLocation = GoldMineralLocation.RIGHT;
                                mineralFound = true;
                                telemetry.addData("Gold Mineral Position", "RIGHT");
                            } else if (goldMineralX < silverMineral1X ) {
                                mineralLocation = GoldMineralLocation.CENTER;
                                mineralFound = true;
                                telemetry.addData("Gold Mineral Position", "CENTER");
                            } else if (time > 21) {
                                mineralLocation = GoldMineralLocation.CENTER;
                                mineralFound = true;
                                telemetry.addData("Gold Mineral Position", "UNKNOWN");
                            }
                        }

                        if (mineralFound) {
                            // Put your movement data here!
                            // For example;
                            if (mineralLocation == GoldMineralLocation.LEFT) {
                                // Movement inforamtion for going to the left mineral
                                elevator(-1,.5);
                                elevator(.5,3);
                                encoderDrive(DRIVE_SPEED,6,6,1);
                                encoderDrive(DRIVE_SPEED,-8,8,1);
                                encoderDrive(DRIVE_SPEED,24,24,4);

                                stop();

                            } else if (mineralLocation == GoldMineralLocation.RIGHT) {
                                // Movement information for going the right
                                elevator(-1,.5);
                                elevator(.5,3);
                                encoderDrive(DRIVE_SPEED,6,6,1);
                                encoderDrive(DRIVE_SPEED,8,-8,1);
                                encoderDrive(DRIVE_SPEED,24,24,4);

                                stop();
                            } else if (mineralLocation == GoldMineralLocation.CENTER){
                                // movement information for going to the center
                                elevator(-1,.5);
                                elevator(.5,3);
                                encoderDrive(DRIVE_SPEED,30,30,4);

                                stop();
                            } else if(runtime.time() > waitTime) {
                                elevator(-1,.5);
                                elevator(.5,3);
                                encoderDrive(DRIVE_SPEED,30,30,4);

                                stop();
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

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

    public void elevator(double power, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            robot.Elevator1.setPower(power);
            robot.Elevator2.setPower(power);
            robot.Elevator3.setPower(power);
        }
        robot.Elevator1.setPower(0);
        robot.Elevator2.setPower(0);
        robot.Elevator3.setPower(0);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.LFront.getCurrentPosition() + (int) (-1 * leftInches * COUNTS_PER_INCH * ENCODER_TUNING);
            newLeftRearTarget = robot.LRear.getCurrentPosition() + (int) (-1 * leftInches * COUNTS_PER_INCH * ENCODER_TUNING);
            newRightFrontTarget = robot.RFront.getCurrentPosition() + (int) (-1 * rightInches * COUNTS_PER_INCH * ENCODER_TUNING);
            newRightRearTarget = robot.RRear.getCurrentPosition() + (int) (-1 * rightInches * COUNTS_PER_INCH * ENCODER_TUNING);
            robot.LFront.setTargetPosition(newLeftFrontTarget);
            robot.LRear.setTargetPosition(newLeftRearTarget);
            robot.RFront.setTargetPosition(newRightFrontTarget);
            robot.RRear.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.LFront.setPower(Math.abs(speed));
            robot.LRear.setPower(Math.abs(speed));
            robot.RFront.setPower(Math.abs(speed));
            robot.RRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.LFront.isBusy() && robot.LRear.isBusy() && robot.RFront.isBusy() && robot.RRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",

                        robot.LFront.getCurrentPosition(),
                        robot.LRear.getCurrentPosition(),
                        robot.RFront.getCurrentPosition(),
                        robot.RRear.getCurrentPosition());
                telemetry.addData("Distance (cm)",
                        telemetry.update());
            }

            // Stop all motion;
            robot.LFront.setPower(0);
            robot.RFront.setPower(0);
            robot.LRear.setPower(0);
            robot.RRear.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
