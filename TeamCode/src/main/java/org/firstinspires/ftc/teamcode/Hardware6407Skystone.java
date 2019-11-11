package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware6407Skystone {
    public DcMotor LFront       = null;
    public DcMotor RRear        = null;
    public DcMotor RFront       = null;
    public DcMotor LRear        = null;
    public DcMotor Elevator1    = null;
    public DcMotor Elevator2    = null;
    public DcMotor Elevator3    = null;
    public BNO055IMU imu        = null;
    public Servo IntakeRight    = null;
    public Servo IntakeLeft     = null;
    public Servo ElevatorRight  = null;
    public Servo ElevatorLeft   = null;
    public DcMotor Intake       = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware6407Skystone(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFront      = hwMap.get(DcMotor.class, "LFront");
        RFront      = hwMap.get(DcMotor.class, "RFront");
        LRear       = hwMap.get(DcMotor.class, "LRear");
        RRear       = hwMap.get(DcMotor.class, "RRear");
        Elevator1   = hwMap.get(DcMotor.class, "Elevator1");
        Elevator2   = hwMap.get(DcMotor.class, "Elevator2");
        Elevator3   = hwMap.get(DcMotor.class, "Elevator3");
        Intake      = hwMap.get(DcMotor.class, "intake");

        // Define and initialize ALL installed servos.
        IntakeLeft      =hwMap.get(Servo.class, "IntakeLeft");
        IntakeRight     =hwMap.get(Servo.class, "IntakeRight");
        ElevatorLeft    =hwMap.get(Servo.class, "ElevatorLeft");
        ElevatorRight   =hwMap.get(Servo.class, "ElevatorRight");

        //Define and initialize ALL installed sensors
        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled                   = true;
        parameters.loggingTag                       = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu                                         =hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        LFront.setDirection(DcMotor.Direction.REVERSE);
        LRear.setDirection(DcMotor.Direction.REVERSE);
        RFront.setDirection(DcMotor.Direction.FORWARD);
        RRear.setDirection(DcMotor.Direction.FORWARD);
        Elevator1.setDirection(DcMotor.Direction.REVERSE);
        Elevator2.setDirection(DcMotor.Direction.REVERSE);
        Elevator3.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        LFront.setPower(0);
        RFront.setPower(0);
        RRear.setPower(0);
        LRear.setPower(0);
        Elevator1.setPower(0);
        Elevator2.setPower(0);
        Elevator3.setPower(0);
        Intake.setPower(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevator3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor ZeroPower Behavior
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
