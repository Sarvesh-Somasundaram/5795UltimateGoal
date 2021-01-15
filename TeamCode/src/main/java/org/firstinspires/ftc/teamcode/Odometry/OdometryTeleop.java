package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@TeleOp
public class OdometryTeleop extends OpMode {

    public DcMotor backLeft, backRight, frontLeft, frontRight, verticalLeft, verticalRight, horizontal, intake, wobble;
    public DcMotorEx brrr;

    public static final double NEW_P = 2.0;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0;

    public boolean isPressed = false;

    public Servo shooterServo, wobbleServo;
    public long setTime = System.currentTimeMillis();

    private File path1 = AppUtil.getInstance().getSettingsFile("path1.txt");
    private File path2 = AppUtil.getInstance().getSettingsFile("path2.txt");
    private File path3 = AppUtil.getInstance().getSettingsFile("path3.txt");

    double x;
    double y;


    final double CPR = 1892.37242833;

    //Hardware Map Names for drive motors and odometry wheels
    String frName = "fright", brName = "bright", flName = "fleft", blName = "bleft";
    String verticalLeftEncoderName = flName, verticalRightEncoderName = frName, horizontalEncoderName = blName;

    OdometryCoordinatePosition positionUpdate;

    @Override
    public void init() {

        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        brrr.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.servo.get("brrrservo");
        shooterServo.setPosition(0.312);

        intake = hardwareMap.dcMotor.get("intake");
        intake.setPower(0);

        wobble = hardwareMap.dcMotor.get("wobble");
        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleServo = hardwareMap.servo.get("wobbles");
        wobbleServo.setPosition(0.47);

        driveMotorMap(frName, brName, flName, blName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Create and start OdometryCoordinatePosition thread to constantly update the coordinate positions
        positionUpdate = new OdometryCoordinatePosition(verticalLeft, verticalRight, horizontal, CPR, 75);
        Thread positionThread = new Thread(positionUpdate);
        positionThread.start();

        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

    }

    public void loop() {
        setTime = System.currentTimeMillis();

        PIDFCoefficients pidOrig = brrr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        brrr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        PIDFCoefficients pidModified = brrr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad1.left_bumper) {
            frontLeft.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
            frontRight.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
            backRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
            backLeft.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
        }
        else {
            frontLeft.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            frontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
            backRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        }


        if (gamepad1.a) {
            isPressed = true;
            shoot();
        }


        if (gamepad1.left_trigger > 0) {
            intake.setPower(-1);
        }
        else if (gamepad1.right_trigger > 0) {
            intake.setPower(gamepad1.right_trigger);
        }
        else {
            intake.setPower(0);
        }

        if (gamepad1.dpad_down) {

            if(wobbleServo.getPosition() > 0.46){
                wobble.setTargetPosition(425);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while(wobble.isBusy()) {
                }
                wobbleServo.setPosition(0);

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            else {
            }
        }
        else if (gamepad1.dpad_up) {

            if (wobbleServo.getPosition() < 0.1) {
                wobbleServo.setPosition(0.47);

                wobble.setTargetPosition(-425);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while(wobble.isBusy()) {
                }
                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            else {
            }
        }

        else if (gamepad1.dpad_right){

            if(wobbleServo.getPosition() > 0.46) {
                wobble.setTargetPosition(320);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while (wobble.isBusy()) {
                }
                wobbleServo.setPosition(0);

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            else {
            }
        }
        else if (gamepad1.dpad_left){

            if (wobbleServo.getPosition() < 0.1) {
                wobble.setTargetPosition(-320);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while(wobble.isBusy()) {
                }
                wobbleServo.setPosition(0.47);

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            else {
            }
        }
        else {
            wobble.setPower(0);
        }


        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", positionUpdate.returnXCoordinate() / CPR);
        telemetry.addData("Y Position", positionUpdate.returnYCoordinate() / CPR);
        telemetry.addData("Orientation (Degrees)", positionUpdate.returnOrientation());

        telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

        telemetry.addData("Wobble counts", wobble.getCurrentPosition());
        telemetry.addData("wobble servo position", wobbleServo.getPosition());
        telemetry.addData("Speed", brrr.getVelocity());


        telemetry.update();






    }

    public void shoot()  {

        while (isPressed) {

            brrr.setPower(-0.88);

            while(System.currentTimeMillis() - setTime < 1400) {

                telemetry.addData("Revving", true);
                telemetry.update();


            }


            while(System.currentTimeMillis() - setTime < 1650) {
                shooterServo.setPosition(0.5);
                telemetry.addData("Shot Number", 1);
                telemetry.update();
            }

            shooterServo.setPosition(0.312);

            while(System.currentTimeMillis() - setTime < 2025) {

            }


            while(System.currentTimeMillis() - setTime < 2250) {
                shooterServo.setPosition(0.5);
                telemetry.addData("Shot Number", 2);
                telemetry.update();


            }

            shooterServo.setPosition(0.312);

            while(System.currentTimeMillis() - setTime < 2650) {

            }


            while(System.currentTimeMillis() - setTime < 2875) {
                shooterServo.setPosition(0.5);

                telemetry.addData("Shot Number", 3);
                telemetry.update();

            }

            shooterServo.setPosition(0.312);

            isPressed = false;

            brrr.setPower(0);
        }
    }

    // Custom goToPosition function
    public void goToPosition(double targetX, double targetY, double drivePow, double desiredOrientation, double distanceErr, double turnErr, double turnPow){

        double distanceToTargX = targetX - positionUpdate.returnXCoordinate();
        double distanceToTargY = targetY - positionUpdate.returnYCoordinate();

        double pivotCorrection = desiredOrientation - positionUpdate.returnOrientation();

        double distance = Math.hypot(distanceToTargX, distanceToTargY);

        while (distance > distanceErr || pivotCorrection > turnErr) {

            distanceToTargX = targetX - positionUpdate.returnXCoordinate();
            distanceToTargY = targetY - positionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToTargX, distanceToTargY);

            double robotAngle = Math.toDegrees(Math.atan2(distanceToTargX, distanceToTargY));

            double movementXComponent = calculateX(robotAngle, drivePow);
            double movementYComponent = calculateY(robotAngle, drivePow);

            pivotCorrection = desiredOrientation - positionUpdate.returnOrientation();

            double robotTurn = Range.clip(Math.toRadians(pivotCorrection)/ Math.toRadians(30), -1, 1) * turnPow;

            frontLeft.setPower(-movementYComponent - movementXComponent - robotTurn);
            frontRight.setPower(movementYComponent - movementXComponent - robotTurn);
            backLeft.setPower(movementYComponent - movementXComponent + robotTurn);
            backRight.setPower(-movementYComponent - movementXComponent + robotTurn);

        }

        frontLeft.setPower(0);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setPower(0);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setPower(0);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setPower(0);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Calculating power in the X direction
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    // Calculating power in the Y direction
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    private void driveMotorMap(String frName, String brName, String flName, String blName, String lEncoderName, String rEncoderName, String hEncoderName){
        frontRight = hardwareMap.dcMotor.get(frName);
        backRight = hardwareMap.dcMotor.get(brName);
        frontLeft = hardwareMap.dcMotor.get(flName);
        backLeft = hardwareMap.dcMotor.get(blName);

        verticalLeft = hardwareMap.dcMotor.get(lEncoderName);
        verticalRight = hardwareMap.dcMotor.get(rEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}
