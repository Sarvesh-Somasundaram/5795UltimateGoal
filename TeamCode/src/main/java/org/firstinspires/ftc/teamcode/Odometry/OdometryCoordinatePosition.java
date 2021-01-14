package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;


public class OdometryCoordinatePosition implements Runnable {

    //Odometry wheels
    private DcMotor verticalLeft, verticalRight, horizontal;

    //Thead run condition
    private boolean isActive = true;

    //Position variables used for calculations
    double rightWheelPosition = 0, leftWheelPosition = 0, horizontalWheelPosition = 0,  orientationChange = 0;
    private double robotXCoordinatePosition = 0, robotYCoordinatePosition = 0, robotOrientation = 0;
    private double previousRightWheelPosition = 0, previousLeftWheelPosition = 0, previousHorizontalWheelPosition = 0;

    //Algorithm constants to find
    private double encoderWheelDistance;
    private double horizontalEncoderOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files with algorithm constants to access in the code
    private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int verticalLeftMultiplier = 1;
    private int verticalRightMultiplier = 1;
    private int horizontalMultiplier = 1;

    //This is a constructor fot the thread specifying all the encoders, counts per inch, sleep time and constants the algorithm uses.
    public OdometryCoordinatePosition(DcMotor verticalLeft, DcMotor verticalRight, DcMotor horizontal, double CPR, int threadSleepDelay){
        this.verticalLeft = verticalLeft;
        this.verticalRight = verticalRight;
        this.horizontal = horizontal;
        sleepTime = threadSleepDelay;

        encoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * CPR;
        this.horizontalEncoderOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());

    }

    // Updates the position
    private void coordinatePositionUpdate(){
        //Get Current Positions
        leftWheelPosition = (verticalLeft.getCurrentPosition() * verticalLeftMultiplier);
        rightWheelPosition = (verticalRight.getCurrentPosition() * verticalRightMultiplier);

        double leftChange = leftWheelPosition - previousLeftWheelPosition;
        double rightChange = rightWheelPosition - previousRightWheelPosition;

        //Calculate the angle by taking the difference between the left and right changes and dividing by the wheel distance
        orientationChange = (leftChange - rightChange) / (encoderWheelDistance);
        robotOrientation = ((robotOrientation + orientationChange));

        //Get the components of the motion
        horizontalWheelPosition = (horizontal.getCurrentPosition()*horizontalMultiplier);
        double actualHorizontalChange = horizontalWheelPosition - previousHorizontalWheelPosition;
        double horizontalChange = actualHorizontalChange - (orientationChange*horizontalEncoderOffset);

        double p = ((rightChange + leftChange) / 2);

        //Calculate and update the position values
        robotXCoordinatePosition = robotXCoordinatePosition + (p* Math.sin(robotOrientation) + horizontalChange* Math.cos(robotOrientation));
        robotYCoordinatePosition = robotYCoordinatePosition + (p* Math.cos(robotOrientation) - horizontalChange* Math.sin(robotOrientation));

        // Set the current positions to the previous positions
        previousLeftWheelPosition = leftWheelPosition;
        previousRightWheelPosition = rightWheelPosition;
        previousHorizontalWheelPosition = horizontalWheelPosition;
    }

    // Returns coordinate and angle values when called
    public double returnXCoordinate(){ return robotXCoordinatePosition; }

    public double returnYCoordinate(){ return robotYCoordinatePosition; }

    public double returnOrientation(){ return Math.toDegrees(robotOrientation) % 360; }

    //Stops the thread
    public void stop(){ isActive = false; }

    // This runs the thread
    @Override
    public void run() {
        while(isActive) {
            coordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}