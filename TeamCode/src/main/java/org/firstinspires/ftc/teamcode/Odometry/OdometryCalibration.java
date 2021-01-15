package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {

    //Drive motors and Odometry wheels
    DcMotor backLeft, backRight, frontLeft, frontRight, verticalLeft, verticalRight, horizontal;

    //IMU Sensor
    BNO055IMU imu;

    //Hardware Map Names for drive motors and odometry wheels
    String frName = "fright", brName = "bright", flName = "fleft", blName = "bleft";
    String verticalLeftEncoderName = flName, verticalRightEncoderName = frName, horizontalEncoderName = blName;

    final double PIVOT_SPEED = 0.5;

    //The amount of encoder counts per inch the robot moves
    final double COUNTS_PER_INCH = 1892.37242833;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values
        initHardwareMap(frName, brName, flName, blName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

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

        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();

        waitForStart();

        //Begin calibration
        while(getXAngle() < 90 && opModeIsActive()){
            frontRight.setPower(PIVOT_SPEED);
            backRight.setPower(PIVOT_SPEED);
            frontLeft.setPower(PIVOT_SPEED);
            backLeft.setPower(PIVOT_SPEED);
            if(getXAngle() < 60) {
                setPowerMotors(PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerMotors(PIVOT_SPEED/3, PIVOT_SPEED/3, PIVOT_SPEED/3, PIVOT_SPEED/3);
            }

            telemetry.addData("IMU Angle", getXAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerMotors(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getXAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getXAngle();


        //Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) + (Math.abs(verticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = horizontal.getCurrentPosition()/ Math.toRadians(getXAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
                telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
                //Display calculated constants
                telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
                telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

                //Display raw values
                telemetry.addData("IMU Angle", getXAngle());
                telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
                telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

                //Update values
                telemetry.update();
        }
    }

    private void initHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        frontRight = hardwareMap.dcMotor.get(rfName);
        backRight = hardwareMap.dcMotor.get(rbName);
        frontLeft = hardwareMap.dcMotor.get(lfName);
        backLeft = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
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

    // Returns robot orientation angle
    private double getXAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    // Sets power to all motors
    private void setPowerMotors(double rf, double rb, double lf, double lb){
        frontRight.setPower(rf);
        backRight.setPower(rb);
        frontLeft.setPower(lf);
        backLeft.setPower(lb);
    }

}