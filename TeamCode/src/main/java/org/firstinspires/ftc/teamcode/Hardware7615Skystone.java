package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware7615Skystone {
    public DcMotor LFront       = null;
    public DcMotor RRear        = null;
    public DcMotor RFront       = null;
    public DcMotor LRear        = null;
    public BNO055IMU imu        = null;
    public Servo Hamm    = null;
    public Servo Rory    = null;
    public Servo Grab    = null;
    public Servo Turn    = null;
    public Servo Lift    = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware7615Skystone(){
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


        // Define and initialize ALL installed servos.
        Hamm      =hwMap.get(Servo.class, "Hamm");
        Rory      =hwMap.get(Servo.class, "Rory");
        Grab      =hwMap.get(Servo.class, "Grab");
        Turn      =hwMap.get(Servo.class, "Turn");
        Lift      =hwMap.get(Servo.class, "Lift");
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


        // Set all motors to zero power
        LFront.setPower(0);
        RFront.setPower(0);
        RRear.setPower(0);
        LRear.setPower(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Set motor ZeroPower Behavior
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
