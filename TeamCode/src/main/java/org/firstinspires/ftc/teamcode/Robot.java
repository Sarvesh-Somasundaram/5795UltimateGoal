package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot extends OpMode {

    /*
    Configuration

    Control Hub
    bright - 0
    bleft & horizontal encoder - 1
    fright & right encoder - 2
    fleft & left encoder - 3
    brrrservo - 0

    Expansion Hub
    brrr - 0
     */


    public DcMotor backLeft, backRight, frontLeft, frontRight, verticalLeft, verticalRight, horizontal, intake, wobble;
    public DcMotorEx brrr;

    public static final double NEW_P = 2.0;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0;

    public boolean isPressed = false;

    public final double COUNTS_PER_INCH = 43.4670116429;
    public Servo shooterServo, wobbleServo;

    public double power = 1;
    public double x = 0;

    public int increment=0;

    public long setTime = System.currentTimeMillis();
    public long start = 0;
    boolean hasRun = false;

    public void init() {

        backLeft = hardwareMap.dcMotor.get("bleft");
        frontLeft = hardwareMap.dcMotor.get("fleft");
        frontRight = hardwareMap.dcMotor.get("fright");
        backRight = hardwareMap.dcMotor.get("bright");
        verticalLeft = hardwareMap.dcMotor.get("fleft");
        verticalRight = hardwareMap.dcMotor.get("fright");
        horizontal = hardwareMap.dcMotor.get("bleft");
        intake = hardwareMap.dcMotor.get("intake");
        wobble = hardwareMap.dcMotor.get("wobble");
        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        wobbleServo = hardwareMap.servo.get("wobbles");

        shooterServo = hardwareMap.servo.get("brrrservo");

        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        intake.setPower(0);
        brrr.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleServo.setPosition(0.47);

    }

    public void loop() {

    }

}
