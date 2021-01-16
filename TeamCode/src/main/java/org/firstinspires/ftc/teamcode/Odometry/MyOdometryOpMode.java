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



@Autonomous(name = "Odometry Auto")
public class MyOdometryOpMode extends LinearOpMode {

    //Drive motors and odometry wheels
    public DcMotor backLeft, backRight, frontLeft, frontRight, verticalLeft, verticalRight, horizontal, intake, wobble;

    public DcMotorEx brrr;

    public Servo shooterServo, wobbleServo, dropServo;

    public boolean shoot = false;

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

    OpenCvCamera webCam;
    DiskDeterminationPipeline pipeline;


    @Override
    public void runOpMode() throws InterruptedException {

        brrr = (DcMotorEx)hardwareMap.get(DcMotor.class, "brrr");
        brrr.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterServo = hardwareMap.servo.get("brrrservo");
        shooterServo.setPosition(0.312);

        intake = hardwareMap.dcMotor.get("intake");

        wobble = hardwareMap.dcMotor.get("wobble");
        wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wobbleServo = hardwareMap.servo.get("wobbles");
        wobbleServo.setPosition(0.47);

        dropServo = hardwareMap.servo.get("drop");
        dropServo.setPosition(0.45);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        pipeline = new DiskDeterminationPipeline();
        webCam.setPipeline(pipeline);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
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

        sleep(200);
        // Determine what position the disk is in
        String positionVal = pipeline.position.toString();
        sleep(300);

        if (positionVal.equals("FOUR")) {

        }

        else if (positionVal.equals("ONE")) {

        }

        else {

            //Movements start here
            goToPosition(0*CPR, 10*CPR, 0.5, 0, 2*CPR, 5, 0);

            while(backRight.isBusy() && backLeft.isBusy() && frontRight.isBusy() && frontLeft.isBusy()) {

                intake.setPower(1);
            }
            intake.setPower(0);

        }



//        brrr.setPower(0.73);
//        goToPosition(0*CPR, -55*CPR, 0.5, 0, 1.5*CPR, 1, 0.4);
//        sleep(500);
//        goToPosition(4*CPR, -57*CPR, 0.5, 0, 1.5*CPR, 1, 0.5);
//        sleep(200);
//        shooterServo.setPosition(0.5);
//        sleep(60);
//        shooterServo.setPosition(0.312);
//        goToPosition(8*CPR, -57*CPR, 0.5, 0, 1.5*CPR, 1, 0.5);
//        sleep(500);
//        shooterServo.setPosition(0.5);
//        sleep(60);
//        shooterServo.setPosition(0.312);
//        goToPosition(14*CPR, -57*CPR, 0.5, 0, 1.5*CPR, 1, 0.5);
//        sleep(500);
//        shooterServo.setPosition(0.5);
//        sleep(60);
//        shooterServo.setPosition(0.312);


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
//                    goToPosition(x*CPR, y*CPR, 0.5, 0, 3*CPR, 3);
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
            telemetry.addData("X Position", positionUpdate.returnXCoordinate() / CPR);
            telemetry.addData("Y Position", positionUpdate.returnYCoordinate() / CPR);
            telemetry.addData("Orientation (Degrees)", positionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());


            telemetry.addData("Speed", brrr.getVelocity());
            telemetry.update();
        }

        //Stop the thread
        positionUpdate.stop();

    }

    public static class DiskDeterminationPipeline extends OpenCvPipeline
    {

        // An enum to define ring positions
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        // Define colors
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);


        // Values that define the size and location of detection box
        static final Point TOPLEFT_ANCHOR_POINT = new Point(190,170);

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
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            box_Cb = Cb.submat(new Rect(pointA, pointB));
        }

        // Processing the frame to evaluate number of rings
        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(box_Cb).val[0];

            // Draws rectangle on input camera frame using the points defined earlier, pointA and pointB
            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    BLUE,
                    2);

            position = RingPosition.FOUR; // Record the analysis
            if(avg1 > FOUR_RINGS){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            // Another rectangle
            Imgproc.rectangle(
                    input,
                    pointA,
                    pointB,
                    GREEN,
                    -1); // Negative value here means infill

            return input;
        }

        // Getter that returns the analysis
        public int getAnalysis()
        {
            return avg1;
        }
    }

    // Custom function for quick firing three discs into high goal
    public void shoot()  {

        while (shoot) {

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

            brrr.setPower(0);

            shoot = false;

        }
    }


    // Custom goToPosition function
    public void goToPosition(double targetX, double targetY, double drivePow, double desiredOrientation, double distanceErr, double turnErr, double turnPow){

        double distanceToTargX = targetX - positionUpdate.returnXCoordinate();
        double distanceToTargY = targetY - positionUpdate.returnYCoordinate();

        double pivotCorrection = desiredOrientation - positionUpdate.returnOrientation();

        double distance = Math.hypot(distanceToTargX, distanceToTargY);

        while (opModeIsActive() && (distance > distanceErr || pivotCorrection > turnErr)) {

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


}