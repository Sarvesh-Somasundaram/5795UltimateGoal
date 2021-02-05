package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Competition Teleop")
public class Teleop extends OpMode {

    // Declaring all the motors, amnd Servos
    public DcMotor backLeft, backRight, frontLeft, frontRight, verticalLeft, verticalRight, horizontal, intake, wobble;
    public DcMotorEx brrr; // brrr is our shooter motor, named after an inside joke based on the sound the motor makes

    // Declaring PIDF coefficients for shooter control
    public static final double NEW_P = 8.0;
    public static final double NEW_I = 0.0;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.0;

    public double newAngle;

    // Declaring the imu
    BNO055IMU imu;

    private boolean isPressed = false;
    double robotOrientation;

    public Servo shooterServo, wobbleServo;
    public long setTime = System.currentTimeMillis();

    public int powerIsPressed = 0;

    // Number of ticks per rotation
    final double CPR = 1892.37242833;

    //Hardware Map Names for drive motors and odometry wheels
    String frName = "fright", brName = "bright", flName = "fleft", blName = "bleft";
    String verticalLeftEncoderName = flName, verticalRightEncoderName = frName, horizontalEncoderName = blName;

    // Start tracking position for odometry
    OdometryCoordinatePosition positionUpdate;

    @Override
    public void init() {
        //Initializing the motors, servos, and pidf coefficients

        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        brrr.setDirection(DcMotorSimple.Direction.REVERSE);
        brrr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brrr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F, MotorControlAlgorithm.PIDF);
        brrr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        shooterServo = hardwareMap.servo.get("brrrservo");
        shooterServo.setPosition(0.312);

        intake = hardwareMap.dcMotor.get("intake");


        wobble = hardwareMap.dcMotor.get("wobble");
        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleServo = hardwareMap.servo.get("wobbles");

        driveMotorMap(frName, brName, flName, blName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        //Create and start OdometryCoordinatePosition thread to constantly update the coordinate positions
        positionUpdate = new OdometryCoordinatePosition(verticalLeft, verticalRight, horizontal, CPR, 75);
        Thread positionThread = new Thread(positionUpdate);
        positionThread.start();

        // Making sure the drive motors are not moving
        frontLeft.setPower(0);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setPower(0);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setPower(0);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setPower(0);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize IMU hardware map value
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        // Altering the loop stuck values because some loops in our code take a while to complete
        msStuckDetectInit     = 5000;
        msStuckDetectInitLoop = 5000;
        msStuckDetectStart    = 11500;
        msStuckDetectLoop     = 11500;
        msStuckDetectStop     = 2800;

    }

    public void loop() {

        // Start the timer for the movement functions later
        setTime = System.currentTimeMillis();

        // Getting the orientation of the robot
        robotOrientation = getXAngle();

        // Movement functions, left bumper makes the drive train move in slow mode
        if (gamepad1.left_bumper) {
            frontLeft.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * .35);
            frontRight.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * .35);
            backRight.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
            backLeft.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * .35);
        }

        // right bumper causes the drivetrain to brake
        else if(gamepad1.right_bumper) {
            frontLeft.setPower(0);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setPower(0);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setPower(0);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setPower(0);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // normal movement
        else {
            frontLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            frontRight.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            backRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
            backLeft.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        }

        // Button to shoot 3 discs
        if (gamepad1.a) {
            isPressed = true;
            shoot();

        }

        // functions to power the intake mechanism
        if (gamepad1.left_trigger > 0) {
            intake.setPower(gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger > 0) {
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
        }

        // These functions are used to control the wobble arm for picking up and depositing
        if (gamepad1.dpad_down) {
            // Check if the position of the servo is closed to make sure the arm doesn't break
            // moves the servo down and opens the servo to get ready to pick up
            if(wobbleServo.getPosition() == 0){
                wobble.setTargetPosition(425);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while(wobble.isBusy()) {
                }
                wobbleServo.setPosition(0.7);

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }
        else if (gamepad1.dpad_up) {
            // get the time and initialize it to setTime
            setTime = System.currentTimeMillis();

            // Check if the servo position is open to make sure the arm doesn't break
            // closes the servo and moves back into the robot
            if (wobbleServo.getPosition() > 0.65) {
                wobbleServo.setPosition(0);

                // waiting until the arm is closed
                setTime = System.currentTimeMillis();
                while(System.currentTimeMillis()-setTime < 600){

                }

                wobble.setTargetPosition(-425);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while(wobble.isBusy()) {
                }

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }

        }

        else if (gamepad1.dpad_right){

            // checking to see if the servo is closed to make sure the arm doesnt break
            // put the arm back down onto the wall of the field to deposit the wobble
            // goal and opens the servo arm
            if(wobbleServo.getPosition() == 0) {
                wobble.setTargetPosition(320);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while (wobble.isBusy()) {
                    wobbleServo.setPosition(0.7);
                }

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }
        else if (gamepad1.dpad_left){

            // Checks to see if the servo is open to make sure the arm doesn't break
            // brings the arm back into the robot

            if (wobbleServo.getPosition() > 0.65) {

                wobble.setTargetPosition(-320);
                wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobble.setPower(0.3);

                while(wobble.isBusy()) {
                    // if the position is not hit, then break out of the loop
                    setTime = System.currentTimeMillis();
                    if (System.currentTimeMillis() - setTime > 1200) {
                        break;
                    }
                }
                wobbleServo.setPosition(0);

                wobble.setPower(0);
                wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

        }

        // this button does a single shot for the power shots at endgame
        if (gamepad1.x) {
            brrr.setPower(-0.78);

            setTime = System.currentTimeMillis();

            while(System.currentTimeMillis() - setTime < 1400) {
                brrr.setPower(-0.78);
                telemetry.addData("Revving", true);
                telemetry.update();
            }

            brrr.setPower(-0.78);

            while(System.currentTimeMillis() - setTime < 1650) {
                brrr.setPower(-0.78);
                shooterServo.setPosition(0.5);
                brrr.setPower(-0.78);
                telemetry.addData("Shot Number", 1);
                telemetry.update();
            }

            brrr.setPower(-0.78);
            shooterServo.setPosition(0.312);
            brrr.setPower(0);

        }

        // this button turns the robot by 5 degrees and by 7 degrees depending on how many times
        // it is pressed for lining up for powershot
        if (gamepad1.y) {
            double currentAngle = getXAngle();

            // if it is the first time the button is pressed, the robot turns by 5 degrees to the left
            if(powerIsPressed == 0) {
                newAngle =  currentAngle - 5;
                turn(newAngle, 0.5, 0.2);
                // powerIsPressed is set to 1 for second time press
                powerIsPressed = 1;
                currentAngle = getXAngle();
                // newAngle is changed by 2 degrees to the left
                newAngle = currentAngle - 2;
            }

            // if it is the second time it is pressed, the robot moves by the newAngle calculated
            // previously by 2 degrees
            else if (powerIsPressed == 1) {
                turn(newAngle, 0.5, 0.2);
                //changes the powerIsPressed to 0
                powerIsPressed = 0;
            }

        }


        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", positionUpdate.returnXCoordinate() / CPR);
        telemetry.addData("Y Position", positionUpdate.returnYCoordinate() / CPR);
        telemetry.addData("Orientation (Degrees)", getXAngle());

        telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

        telemetry.addData("Wobble counts", wobble.getCurrentPosition());
        telemetry.addData("wobble servo position", wobbleServo.getPosition());


        telemetry.addData("Speed", brrr.getVelocity());


        telemetry.update();

    }

    // Getting the orientation fo the robot from the imu
    private double getXAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    // function used to turn the robot by a specfic angle
    public void turn(double angle, double turnPow, double minTurnPow) {
        double currentAngle = getXAngle();

        double pivot = angle - currentAngle;

        /* the algorithm checks to see what the pivot change required is and then if that value
        lies in any of the below ranges, the power values are multiplied by -1 to get the most
        optimal turn direction, either clockwise or counter clockwise.
        */
        if (pivot >180 || ((pivot > -180) && (pivot < 0))) {
            minTurnPow = minTurnPow * -1;
            turnPow = turnPow*-1;
        }

        // while the pivot change is less than 2, calculate the turnPower
        // and turn based on that power
        while (Math.abs(pivot) > 2) {

            currentAngle = getXAngle();
            pivot = angle - currentAngle;


            double turnPower = Range.clip(Math.toRadians(Math.abs(pivot)) / Math.toRadians(180), -1, 1) * turnPow;

            if (Math.abs(pivot) > 40) {
                backLeft.setPower(turnPower);
                frontLeft.setPower(-turnPower);
                frontRight.setPower(-turnPower);
                backRight.setPower(turnPower);
            }
            // if the angle gets too small, and the turnPower becomes too small, then turn the
            // drivetrain using the minTurnPower value
            else {
                backLeft.setPower(minTurnPow);
                frontLeft.setPower(-minTurnPow);
                frontRight.setPower(-minTurnPow);
                backRight.setPower(minTurnPow);
            }

            telemetry.addData("Orientation", currentAngle);
            telemetry.addData("Pivot", pivot);
            telemetry.update();
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

    // this method is used to shoot 3 shots in a row in under 1.5 seconds
    public void shoot()  {

        /* use this while loop so the code doesnt break out of the loop prematurely, this is also
        the reason we had to increase the loop stuck detect values
         */
        while (isPressed) {
            // the shooter servo is set to start revving up
            brrr.setPower(-0.875);

            setTime = System.currentTimeMillis();
            // these while loops are implemented to give the motor time to rev back up to full
            // speed after each shot, assisted by the PID control
            while(System.currentTimeMillis() - setTime < 1480) {
                brrr.setPower(-0.875);
                telemetry.addData("Revving", true);
                telemetry.update();
            }

            brrr.setPower(-0.875);

            while(System.currentTimeMillis() - setTime < 1650) {
                brrr.setPower(-0.875);
                shooterServo.setPosition(0.5);
                brrr.setPower(-0.875);
                telemetry.addData("Shot Number", 1);
                telemetry.update();
            }

            brrr.setPower(-0.875);
            shooterServo.setPosition(0.312);
            brrr.setPower(-0.875);

            while(System.currentTimeMillis() - setTime < 2025) {
                brrr.setPower(-0.875);
            }

            brrr.setPower(-0.875);

            while(System.currentTimeMillis() - setTime < 2250) {
                brrr.setPower(-0.875);
                shooterServo.setPosition(0.5);
                brrr.setPower(-0.875);
                telemetry.addData("Shot Number", 2);
                telemetry.update();
            }

            brrr.setPower(-0.875);
            shooterServo.setPosition(0.312);
            brrr.setPower(-0.875);

            while(System.currentTimeMillis() - setTime < 2650) {
                brrr.setPower(-0.875);
            }

            brrr.setPower(-0.875);

            while(System.currentTimeMillis() - setTime < 2875) {
                brrr.setPower(-0.875);
                shooterServo.setPosition(0.5);
                brrr.setPower(-0.875);
                telemetry.addData("Shot Number", 3);
                telemetry.update();
            }

            brrr.setPower(-0.875);
            shooterServo.setPosition(0.312);

            brrr.setPower(0);
            isPressed = false;

        }
    }

    // Custom moveToPosition function, will implement later to assist lining up for shooting
    public void moveToPosition(double targetX, double targetY, double drivePow, double desiredOrientation, double distanceErr, double turnErr, double turnPow){

        double distanceToTargX = targetX - positionUpdate.returnXCoordinate();
        double distanceToTargY = targetY - positionUpdate.returnYCoordinate();

        double pivotCorrection = desiredOrientation - positionUpdate.returnOrientation();

        double distance = Math.hypot(distanceToTargX, distanceToTargY);

        while (distance > distanceErr) {

            distanceToTargX = targetX - positionUpdate.returnXCoordinate();
            distanceToTargY = targetY - positionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToTargX, distanceToTargY);

            double robotAngle = Math.toDegrees(Math.atan2(distanceToTargX, distanceToTargY));

            double movementXComponent = calculateX(robotAngle, drivePow);
            double movementYComponent = calculateY(robotAngle, drivePow);

            pivotCorrection = desiredOrientation - positionUpdate.returnOrientation();

            double robotTurn = Range.clip(Math.toRadians(pivotCorrection) / Math.toRadians(30), -1, 1) * turnPow;

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

        isPressed = false;
    }

    // Calculating power in the X direction
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    // Calculating power in the Y direction
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    //initialize the drivetrain motors
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
