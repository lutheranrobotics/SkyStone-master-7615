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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware7615
{
    /* Public OpMode members. */
    public DcMotor  LFront   = null;
    public DcMotor  RRear   = null;
    public DcMotor  RFront  = null;
    public DcMotor  LRear   = null;
  //  DcMotor  Arm = null ;
    Servo RClaw = null;
    Servo LClaw = null;
  //  Servo URClaw = null;
  //  Servo ULClaw = null;
    Servo ClawJoint = null;
  //  Servo UClawJoint = null;
   // public Servo    LClaw    = null;
    Servo jewel   = null;
   // public Servo    RClaw   = null;
   Servo jewel2 = null;
//    ColorSensor sensorColor;
//    DistanceSensor sensorDistance;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware7615(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Define Touch Sensor/Limit Switch
       // limit_switch = hwMap.touchSensor.get("limit_switch");

        // Define and Initialize Motors
        LFront  = hwMap.get(DcMotor.class, "LFront");
        RFront = hwMap.get(DcMotor.class, "RFront");
        LRear  = hwMap.get(DcMotor.class, "LRear");
        RRear = hwMap.get(DcMotor.class, "RRear");
      //  Arm    = hwMap.dcMotor.get ( "Arm");
        RClaw = hwMap.servo.get ("RClaw");
        LClaw = hwMap.servo.get ("LClaw");
        jewel = hwMap.servo.get ("jewel");
        jewel2 = hwMap.servo.get ("jewel2");
  //      URClaw = hwMap.servo.get("URClaw");
  //      ULClaw = hwMap.servo.get("ULClaw");
        ClawJoint = hwMap.servo.get("ClawJoint");
  //      UClawJoint = hwMap.servo.get("UClawJoint");
//        sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
//        sensorDistance = hwMap.get(DistanceSensor.class, "sensor_color_distance");
 //       sensorColor.enableLed(true);
        LFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        LRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        RFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        RRear.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
     //   Arm.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        // Set all motors to zero power
        LFront.setPower(0);
        RFront.setPower(0);
       //Arm.setPower(0);
        RRear.setPower(0);
        LRear.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        LClaw  = hwMap.get(Servo.class, "LClaw");
        jewel = hwMap.get(Servo.class, "jewel");
        RClaw = hwMap.get(Servo.class, "RClaw");
        jewel2 = hwMap.get(Servo.class, "jewel2");



    }
}
