package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


@Autonomous(name = "Odometry Auto")
public class MyOdometryOpMode extends LinearOpMode {

    //Drive motors and odometry wheels and servos are declared here
    public DcMotor backLeft, backRight, frontLeft, frontRight, verticalLeft, verticalRight, horizontal, intake, wobble;
    public DcMotorEx brrr; // brrr is our shooter motor, named after an inside joke based on the sound the motor makes

    public Servo shooterServo, wobbleServo, dropServo;

    // declaring the imu from the expansion hub
    BNO055IMU imu;

    //Declaring the PIDF coefficients for PID control
    public static final double NEW_P = 9.6;
    public static final double NEW_I = 0.0;
    public static final double NEW_D = 0.0;
    public static final double NEW_F = 12.0;

    // Declaring and initializing the setTime to the current time
    public long setTime = System.currentTimeMillis();
    // Declare and initialize the counts per inch
    final double CPR = 1892.37242833;

    //Hardware Map Names for drive motors and odometry wheels
    String frName = "fright", brName = "bright", flName = "fleft", blName = "bleft";
    String verticalLeftEncoderName = flName, verticalRightEncoderName = frName, horizontalEncoderName = blName;
    // Start tracking position for odometry
    OdometryCoordinatePosition positionUpdate;
    // Start the webcam and the disk recognition pipeline
    OpenCvCamera webCam;
    DiskDeterminationPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors, and initialize the shooter motor to use the PID control
        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        brrr.setDirection(DcMotorSimple.Direction.REVERSE);
        brrr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F, MotorControlAlgorithm.PIDF);
        brrr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        //Initialize the servos and other mechanism motors and drivetrain motors
        shooterServo = hardwareMap.servo.get("brrrservo");
        shooterServo.setPosition(0.312);

        intake = hardwareMap.dcMotor.get("intake");

        wobble = hardwareMap.dcMotor.get("wobble");
        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleServo = hardwareMap.servo.get("wobbles");
        wobbleServo.setPosition(0);

        dropServo = hardwareMap.servo.get("drop");

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

        // initialize the webCam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // initialize pipeline to the diskDetermination pipeline
        pipeline = new DiskDeterminationPipeline();
        webCam.setPipeline(pipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        //Initialize hardware map values
        driveMotorMap(frName, brName, flName, blName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();


        waitForStart();

        setTime = System.currentTimeMillis();

        //Create and start OdometryCoordinatePosition thread to constantly update the coordinate positions
        positionUpdate = new OdometryCoordinatePosition(verticalLeft, verticalRight, horizontal, CPR, 75);
        Thread positionThread = new Thread(positionUpdate);
        positionThread.start();

        // Reverse encoder values to face the correct direction
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Determine what position the disk is in and save the position to a String to store the
        // first recognized values
        sleep(500);
        String positionVal = pipeline.position.toString();

        telemetry.addData("Num rings", positionVal);
        telemetry.update();

        if (positionVal.equals("FOUR")) {
            /* Movement starts here for the four ring path, look below the opmode loop for more
            information on what each of the methods used in the path do. In all paths, the power
            shots are hit first, and then the first wobble goal is deposited, then the paths
            differ after that point.
             */
            brrr.setPower(-0.85);
            moveToPosition(15 * CPR, -8 * CPR, 0.5, 0, 3 * CPR, 3, 0.8);
            sleep(80);
            moveToPosition(16.5 * CPR, -53 * CPR, 0.5, 0, 3 * CPR, 3, 1);
            dropServo.setPosition(0.45);
            sleep(250);
            brrr.setPower(-0.85);
            moveToPosition(0*CPR, -53*CPR, 0.5, 0, 3*CPR, 5, 0.9);
            sleep(150);
            turn(1, 0.7, 0.4, -0.84);
            brrr.setPower(-0.835);
            shoot();
            intake.setPower(-1);
            moveToPosition(0*CPR, -50*CPR, 0.3, 0, 1*CPR, 3, 0.6);
            intake.setPower(-1);
            sleep(655);
            brrr.setPower(-0.85);
            moveToPosition(0*CPR, -55*CPR, 0.6, 0, 3*CPR, 3, 0.7);
            sleep(380);
            brrr.setPower(-0.836);
            singleShot();
            sleep(300);
            intake.setPower(-0.85);
            brrr.setPower(-0.85);
            moveToPosition(-5*CPR, -34*CPR, 0.3, 0, 2*CPR, 3, 0.6);
            intake.setPower(-0.85);
            sleep(805);
            brrr.setPower(-0.85);
            moveToPosition(-2*CPR, -55*CPR, 0.6, 0, 3*CPR, 2, 0.7);
            sleep(100);
            brrr.setPower(-0.84);
            turn(0, 0.6, 0.4, -0.835);
            shoot();
            intake.setPower(0);
            moveToPosition(-10*CPR, -100*CPR, 0.8, 0, 5*CPR, 360, 0);
            turn(178, 0.9, 0.9, 0);
            wobbleDown();
            wobbleServo.setPosition(0.7);
            wobbleUp(0.3);
            sleep(270);
            turn(0, 0.9, 0.9, 0);
            wobbleDown();
            moveToPosition(-11*CPR, -50*CPR, 0.8, 0, 5*CPR, 5, 1);
            sleep(200);
            turn(-1, 0.9, 0.6, 0);
            sleep(150);
            moveToPosition(-11*CPR, -34*CPR, 0.65, 0, 5*CPR, 5, 0.8);
            sleep(250);
            wobbleServo.setPosition(0);
            sleep(200);
            wobbleUp(0.2);
            moveToPosition(-13*CPR, -95*CPR, 0.9, 0, 5*CPR, 5, 0.9);
            wobbleServo.setPosition(0.7);
            sleep(400);
            turn(5, 0.9, 0.9, 0);
            moveToPosition(-10*CPR, -65*CPR, 0.8, 0, 6*CPR, 5, 0.9);
            brake();
            requestOpModeStop();
        }

        else if (positionVal.equals("ONE")) {
            // Movement starts here for the one ring path, this path intakes one ring and shootes it
            // in addition to all the other movements
            brrr.setPower(-0.76);
            moveToPosition(15 * CPR, -8 * CPR, 0.4, 0, 2 * CPR, 2, 0.8);
            sleep(80);
            moveToPosition(16.5 * CPR, -53 * CPR, 0.34, 0, 2 * CPR, 2, 1);
            dropServo.setPosition(0.45);
            sleep(250);
            brrr.setPower(-0.74);
            singleShot();
            sleep(350);
            moveToPosition(22 * CPR, -53 * CPR, 0.25, 0, 1 * CPR, 2, 0.6);
            sleep(250);
            brrr.setPower(-0.74);
            singleShot();
            sleep(400);
            moveToPosition(28 * CPR, -53 * CPR, 0.24, 0, 1 * CPR, 2, 0.6);
            sleep(300);
            brrr.setPower(-0.74);
            singleShot();
            brrr.setPower(0);
            turn(0, 0.8, 0.26, 0);
            moveToPosition(10*CPR, -73*CPR, 0.8, 0, 5*CPR, 2, 0.5);
            sleep(90);
            turn(178, 0.8, 0.4, 0);
            wobbleDown();
            wobbleServo.setPosition(0.7);
            sleep(150);
            wobbleUp(0.3);
            turn(-3, 0.7, 0.4, 0);
            intake.setPower(-1);
            moveToPosition(0*CPR, -40*CPR, 0.6, 0, 3* CPR, 2, 0.6);
            wobbleDown();
            moveToPosition(-13*CPR, -40*CPR, 0.5, 0, 3*CPR, 2, 0.6);
            sleep(120);
            moveToPosition(-15*CPR, -28*CPR, 0.435, 0, 3*CPR, 2, 0.6);
            sleep(120);
            intake.setPower(0);
            wobbleServo.setPosition(0);
            sleep(500);
            wobbleUp(0.15);
            brrr.setPower(-0.886);
            moveToPosition(5*CPR, -50*CPR, 0.55, 0, 3*CPR, 2, 0.5);
            turn(0, 0.6, 0.35, 0);
            sleep(300);
            singleShot();
            moveToPosition(7*CPR, -69*CPR, 0.8, 0, 3*CPR, 2, 0.55);
            turn(175, 0.8, 0.5, 0);
            wobbleDown();
            wobbleServo.setPosition(0.7);
            wobbleUp(0.25);
            setTime = System.currentTimeMillis();
            while(opModeIsActive() && (System.currentTimeMillis() - setTime < 300)) {
                backLeft.setPower(-1);
                frontLeft.setPower(1);
                frontRight.setPower(-1);
                backRight.setPower(1);
            }
            brake();
            requestOpModeStop();
        }

        else {
            // Movement starts here for the zero ring path
            brrr.setPower(-0.76);
            moveToPosition(15 * CPR, -8 * CPR, 0.4, 0, 2 * CPR, 2, 0.8);
            sleep(80);
            moveToPosition(16.5 * CPR, -53 * CPR, 0.34, 0, 2 * CPR, 2, 1);
            dropServo.setPosition(0.45);
            sleep(250);
            brrr.setPower(-0.74);
            singleShot();
            sleep(350);
            moveToPosition(22 * CPR, -53 * CPR, 0.25, 0, 1 * CPR, 2, 0.6);
            sleep(250);
            brrr.setPower(-0.735);
            singleShot();
            sleep(400);
            moveToPosition(28 * CPR, -53 * CPR, 0.24, 0, 1 * CPR, 2, 0.6);
            sleep(300);
            brrr.setPower(-0.74);
            singleShot();
            brrr.setPower(0);
            turn(0, 0.5, 0.26, 0);
            moveToPosition(-16 * CPR, -57 * CPR, 0.5, 0, 2 * CPR, 2, 0.6);
            sleep(200);
            turn(178, 0.8, 0.34, 0);
            wobbleDown();
            wobbleServo.setPosition(0.7);
            sleep(150);
            wobbleUp(0.3);
            turn(0, 0.5, 0.32, 0);
            wobbleDown();
            moveToPosition(-15 * CPR, -27 * CPR, 0.3, 0, 3 * CPR, 2, 0.6);
            sleep(350);
            wobbleServo.setPosition(0);
            sleep(300);
            wobbleUp(0.3);
            sleep(100);
            moveToPosition(-15 * CPR, -65 * CPR, 0.5, 0, 2 * CPR, 2, 0.6);
            turn(178, 0.64, 0.34, 0);
            wobbleDown();
            sleep(100);
            wobbleServo.setPosition(0.7);
            wobbleUp(0.2);
            wobbleServo.setPosition(0);
            sleep(170);
            requestOpModeStop();
        }

        while(opModeIsActive()){

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", positionVal);

            // Don't burn CPU cycles busy-looping this sample
            sleep(50);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", positionUpdate.returnXCoordinate() / CPR);
            telemetry.addData("Y Position", positionUpdate.returnYCoordinate() / CPR);
            telemetry.addData("Orientation (Odometry)", positionUpdate.returnOrientation());
            telemetry.addData("Orientation (IMU)", getXAngle());
            // Display the actual encoder position values as well
            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            // output the speed of the shooter motor to see if PID is working in keeping the
            // speed constant
            telemetry.addData("Speed", brrr.getVelocity());
            telemetry.update();
        }
        //Stop the thread
        positionUpdate.stop();
    }
    // This method moves the wobble goal up
    public void wobbleUp(double power) {

        wobble.setTargetPosition(-425);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble.setPower(power);

        while(opModeIsActive() && wobble.isBusy()) {
        }

        wobble.setPower(0);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    // this method moves the wobble goal down
    public void wobbleDown() {
        wobble.setTargetPosition(425);
        wobble.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobble.setPower(0.3);

        while(opModeIsActive() && wobble.isBusy()) {
        }

        wobble.setPower(0);
        wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // this method is used to determine the number of disks
    public static class DiskDeterminationPipeline extends OpenCvPipeline {

        // An enum to define ring positions
        public enum RingPosition {FOUR, ONE, NONE}
        // Define colors
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        // Values that define the size and location of detection box
        static final Point TOPLEFT_ANCHOR_POINT = new Point(198,179);

        static final int WIDTH = 25;
        static final int HEIGHT = 35;
        final int FOUR_RINGS = 150;
        final int ONE_RING = 135;

        Point pointA = new Point(
                TOPLEFT_ANCHOR_POINT.x,
                TOPLEFT_ANCHOR_POINT.y);

        Point pointB = new Point(
                TOPLEFT_ANCHOR_POINT.x + WIDTH,
                TOPLEFT_ANCHOR_POINT.y + HEIGHT);

        // Variables to perform calculation and decisions
        Mat box_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Position value which is accessed by OpMode thread
        private volatile RingPosition position = RingPosition.FOUR;

        // COnverts YCrCb to Cb
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);
            box_Cb = Cb.submat(new Rect(pointA, pointB));
        }

        // Processing the frame to evaluate number of rings
        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            avg1 = (int) Core.mean(box_Cb).val[0];
            // Draws rectangle on input camera frame using the points defined earlier, pointA and pointB
            Imgproc.rectangle(input, pointA, pointB, BLUE, 2);

            position = RingPosition.NONE; // Record the analysis of how many rings
            if(avg1 > FOUR_RINGS){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }
            // Another rectangle
            Imgproc.rectangle(input, pointA, pointB, GREEN, -1);
            return input;
        }

        // Getter that returns the analysis
        public int getAnalysis() {
            return avg1;
        }
    }

    // this method is the same as in the teleop code, a turn function which turns based on the
    // imu orientation value
    public void turn(double angle, double turnPow, double minTurnPow, double brrrpower) {
        double currentAngle = getXAngle();

        double pivot = angle - currentAngle;

        if (pivot < 0) {
            minTurnPow = minTurnPow * -1;
        }

        brrr.setPower(brrrpower);

        while (opModeIsActive() && Math.abs(pivot) > 2) {
            brrr.setPower(brrrpower);

            currentAngle = getXAngle();
            pivot = angle - currentAngle;

            double turnPower = Range.clip(Math.toRadians(pivot) / Math.toRadians(180), -1, 1) * turnPow;

            if (Math.abs(pivot) > 50) {
                backLeft.setPower(turnPower);
                frontLeft.setPower(-turnPower);
                frontRight.setPower(-turnPower);
                backRight.setPower(turnPower);
            }
            else {
                backLeft.setPower(minTurnPow);
                frontLeft.setPower(-minTurnPow);
                frontRight.setPower(-minTurnPow);
                backRight.setPower(minTurnPow);
            }
            telemetry.addData("Orientation", currentAngle);
            telemetry.update();
        }
        brrr.setPower(brrrpower);
        brake();
    }
    // the singleShot method does one single shot, used in the power shot section of the Auto
    public void singleShot() {
        shooterServo.setPosition(0.5);
        sleep(60);
        shooterServo.setPosition(0.312);
    }
    // Custom function for quick firing three discs into high goal, used from the teleop code
    public void shoot()  {
        brrr.setPower(-0.825);
        shooterServo.setPosition(0.5);
        sleep(250);
        shooterServo.setPosition(0.312);
        brrr.setPower(-0.83);
        sleep(180);
        shooterServo.setPosition(0.5);
        brrr.setPower(-0.835);
        sleep(250);
        shooterServo.setPosition(0.312);
        brrr.setPower(-0.83);
        sleep(180);
        shooterServo.setPosition(0.5);
        brrr.setPower(-0.835);
        sleep(250);
        shooterServo.setPosition(0.312);
        brrr.setPower(-0.825);
        shooterServo.setPosition(0.312);

        brrr.setPower(0);
    }
    // Custom moveToPosition function, using odometry
    public void moveToPosition(double targetX, double targetY, double drivePow, double desiredOrientation, double distanceErr, double turnErr, double turnPow){
        // getting the required distance from current position to the target values
        double distanceToTargX = targetX - positionUpdate.returnXCoordinate();
        double distanceToTargY = targetY - positionUpdate.returnYCoordinate();
        // calculate the pivot correction value to see how much the robot needs to turn
        double pivotCorrection = desiredOrientation - getXAngle();
        // calculate the distance using the target values
        double distance = Math.hypot(distanceToTargX, distanceToTargY);

        // this loops until the distance is met and the pivot correction is maintained
        while (opModeIsActive() && (distance > distanceErr || Math.abs(pivotCorrection) > turnErr)) {

            // Updating the distance to the target values on each loop of the while loop to see if distance is reached
            distanceToTargX = targetX - positionUpdate.returnXCoordinate();
            distanceToTargY = targetY - positionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToTargX, distanceToTargY);

            // Calculating the angle the target distance is at relative to the robot
            double robotAngle = Math.toDegrees(Math.atan2(distanceToTargX, distanceToTargY));

            //Calculating the X and Y power components for the motors
            double movementXComponent = calculateX(robotAngle, drivePow);
            double movementYComponent = calculateY(robotAngle, drivePow);

            // Calculating pivotCorrection to see if the desired orientation has been reached or not
            pivotCorrection = desiredOrientation - getXAngle();

            // Calculating the turn power for motors
            double robotTurn = Range.clip(Math.toRadians(pivotCorrection)/ Math.toRadians(180), -1, 1) * turnPow;

            //Set Power to motors for movement
            frontLeft.setPower(-movementYComponent - movementXComponent - robotTurn);
            frontRight.setPower(movementYComponent - movementXComponent - robotTurn);
            backLeft.setPower(movementYComponent - movementXComponent + robotTurn);
            backRight.setPower(-movementYComponent - movementXComponent + robotTurn);

            //Update position (x, y, theta) values
            telemetry.addData("X Position", positionUpdate.returnXCoordinate() / CPR);
            telemetry.addData("Y Position", positionUpdate.returnYCoordinate() / CPR);
            telemetry.addData("Orientation (IMU)", getXAngle());
            telemetry.update();
        }
//      Stop the motors and set Mode to Brake
        brake();
    }

    // function to brake all drivetrain motors
    public void brake() {
        frontLeft.setPower(0);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setPower(0);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setPower(0);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setPower(0);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Mapping the motors and encoders to hardware map
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

    // Calculating power in the X direction
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    // Calculating power in the Y direction
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
    // getting the robot orientation using the imu gyro from the expansion hub
    private double getXAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }
}