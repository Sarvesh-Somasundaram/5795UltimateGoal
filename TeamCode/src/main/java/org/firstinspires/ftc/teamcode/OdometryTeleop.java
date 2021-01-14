package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;


@TeleOp(name = "Odometry TeleOp")
public class OdometryTeleop extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back, intake;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    public DcMotorEx brrr;
    public Servo shooterServo;

    public static final double NEW_P = 2.0;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0;

    public double x = 0;


    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fright", rbName = "bright", lfName = "fleft", lbName = "bleft";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        intake = hardwareMap.dcMotor.get("intake");

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        brrr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        PIDFCoefficients pidModified = brrr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);


        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        shooterServo = hardwareMap.servo.get("brrrservo");

        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);
        shooterServo.setPosition(0.7);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();


        brrr.setPower(x);
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                left_front.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
                right_front.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
                right_back.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
                left_back.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);

            }

            else {
                left_front.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                right_front.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                right_back.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
                left_back.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            }



            if (gamepad2.right_stick_y > 0) {
                brrr.setPower(gamepad2.right_stick_y);

            }
            else {
                brrr.setPower(x);
            }


            if (gamepad2.right_bumper) {

                shooterServo.setPosition(0.9);
            }

            else if (gamepad2.left_bumper){

                shooterServo.setPosition(0.7);

            }



            if (gamepad1.left_trigger > 0) {

                intake.setPower(1);


            }

            else  {

                intake.setPower(0);


            }

            if (gamepad2.dpad_up){
                x = x + 0.1;
            }

            else if (gamepad2.dpad_down){
                x = x - 0.1;
            }

            else if (gamepad2.dpad_left) {
                x = x- 0.01;
            }

            else if (gamepad2.dpad_right) {
                x = x + 0.01;
            }

            else {

            }




            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());

            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModified.p, pidModified.i, pidModified.d);


            telemetry.addData("Speed", brrr.getVelocity());
            telemetry.addData("Power", brrr.getPower());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void goToPosition(double targetXPos, double targetYPos, double robotPower, double desiredRobotOrientation, double allowableDistanceError){

        double distanceToXTarg = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarg = targetYPos - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarg, distanceToYTarg);

        while (opModeIsActive() && distance > allowableDistanceError) {

            distanceToXTarg = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarg = targetYPos - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarg, distanceToYTarg);

            double robotAngle = Math.toDegrees(Math.atan2(distanceToXTarg, distanceToYTarg));

            double robotMovementXComp = calculateX(robotAngle, robotPower);
            double robotMovementYComp = calculateY(robotAngle, robotPower);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();



            left_front.setPower(-robotMovementYComp - robotMovementXComp);
            right_front.setPower(-robotMovementYComp + robotMovementXComp);
            left_back.setPower(-robotMovementYComp + robotMovementXComp);
            right_back.setPower(-robotMovementYComp - robotMovementXComp);

        }

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }


    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    public void shoot()  {
        shooterServo.setPosition(0.9);
        sleep(200);
        shooterServo.setPosition(0.7);
    }

    public void turn(double angle) {
        double currentAngle = globalPositionUpdate.returnOrientation();

        double pivot = angle - currentAngle;

        while (opModeIsActive() && pivot > 10) {

            currentAngle = globalPositionUpdate.returnOrientation();
            pivot = angle - currentAngle;

            double pivotComp = pivot / 360;
            left_back.setPower(pivotComp);
            left_front.setPower(pivotComp);
            right_front.setPower(-pivotComp);
            right_back.setPower(-pivotComp);

        }

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);

    }



    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}