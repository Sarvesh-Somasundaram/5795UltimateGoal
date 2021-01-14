package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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

import java.io.File;


/**
 * Created by Sarthak on 10/4/2019.
 */
@Autonomous(name = "Odometry Auto")
public class MyOdometryOpMode extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;
    public DcMotorEx brrr;
    public Servo shooterServo;
    public boolean isPressed = false;
    public long setTime = System.currentTimeMillis();
    private File valuesFile = AppUtil.getInstance().getSettingsFile("values.txt");
    double x;
    double y;


    final double COUNTS_PER_INCH = 1892.37242833;

//    public static final double NEW_P = 2.0;
//    public static final double NEW_I = 0.1;
//    public static final double NEW_D = 0.2;
//    public static final double NEW_F = 0;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "fright", rbName = "bright", lfName = "fleft", lbName = "bleft";
    String verticalLeftEncoderName = lfName, verticalRightEncoderName = rfName, horizontalEncoderName = lbName;

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    OpenCvCamera webCam;
    SkystoneDeterminationPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {

        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        shooterServo = hardwareMap.servo.get("brrrservo");
        shooterServo.setPosition(0.312);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        pipeline = new SkystoneDeterminationPipeline();
        webCam.setPipeline(pipeline);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        setTime = System.currentTimeMillis();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        String positionVal = pipeline.position.toString();

        goToPosition(10*COUNTS_PER_INCH, 0*COUNTS_PER_INCH, 0.5, 90, 2*COUNTS_PER_INCH, 5, 1);



//        try {
//            Scanner scanner = new Scanner(valuesFile);
//            while (scanner.hasNextLine()) {
//
//                String first_xy = scanner.nextLine();
//                if (first_xy.equals("EOF")) {
//                    break;
//                }
//
//                else {
//                    String[] xy = first_xy.split(",");
//                    x = Double.parseDouble(xy[0]);
//                    y = Double.parseDouble(xy[1]);
//
//                    telemetry.addData("x", x);
//                    telemetry.addData("y", y);
//                    telemetry.update();
//
//                    goToPosition(x*COUNTS_PER_INCH, y*COUNTS_PER_INCH, 0.5, 0, 3*COUNTS_PER_INCH, 3);
//
//                }
//
//                sleep(300);
//
//
//            }
//            scanner.close();
//        } catch (FileNotFoundException e) {
//            e.printStackTrace();
//        }

        while(opModeIsActive()){

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", positionVal);

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());


            telemetry.addData("Speed", brrr.getVelocity());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(190,170);

        static final int REGION_WIDTH = 25;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
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


    public void goToPosition(double targetXPos, double targetYPos, double robotPower, double desiredRobotOrientation, double allowableDistanceError, double allowableTurnError, double turnPower){

        double distanceToXTarg = targetXPos - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarg = targetYPos - globalPositionUpdate.returnYCoordinate();

        double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

        double distance = Math.hypot(distanceToXTarg, distanceToYTarg);

        while (opModeIsActive() && (distance > allowableDistanceError || pivotCorrection > allowableTurnError)) {

            distanceToXTarg = targetXPos - globalPositionUpdate.returnXCoordinate();
            distanceToYTarg = targetYPos - globalPositionUpdate.returnYCoordinate();
            distance = Math.hypot(distanceToXTarg, distanceToYTarg);

            double robotAngle = Math.toDegrees(Math.atan2(distanceToXTarg, distanceToYTarg));

            double robotMovementXComp = calculateX(robotAngle, robotPower);
            double robotMovementYComp = calculateY(robotAngle, robotPower);

            pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

            double robotTurn = Range.clip(Math.toRadians(pivotCorrection)/ Math.toRadians(30), -1, 1) * turnPower;


            left_front.setPower(-robotMovementYComp - robotMovementXComp - robotTurn);
            right_front.setPower(robotMovementYComp - robotMovementXComp - robotTurn);
            left_back.setPower(robotMovementYComp - robotMovementXComp + robotTurn);
            right_back.setPower(-robotMovementYComp - robotMovementXComp + robotTurn);

        }

        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }

    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
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

//        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }


}