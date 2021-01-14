package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Coordinate Position Test", group = "Odometry Testing")
public class CoordinatePositionTest extends LinearOpMode {

    //Odometry encoder wheels
    DcMotor verticalRight, verticalLeft, horizontal;

    //The amount of encoder ticks per inch
    final double CPR = 1892.37242833;

    @Override
    public void runOpMode() {

        //Assign the hardware map to the odometry wheels
        verticalLeft = hardwareMap.dcMotor.get("fleft");
        verticalRight = hardwareMap.dcMotor.get("fright");
        horizontal = hardwareMap.dcMotor.get("bleft");


        //Reset the encoders
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();


        //Create and start GlobalCoordinatePosition thread to update the coordinate positions
        OdometryCoordinatePosition globalPositionUpdate = new OdometryCoordinatePosition(verticalLeft, verticalRight, horizontal, CPR, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // Reverse the encoders that need to be to get the right values
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        while(opModeIsActive()){
            //Display coordinates, angles, and encoder values

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / CPR);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / CPR);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("Horizontal encoder position", horizontal.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();
    }
}