package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "UltimateGoalTeleOp", group = "Competition")

public class TeleOp extends Robot {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        setTime = System.currentTimeMillis();

        PIDFCoefficients pidOrig = brrr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        brrr.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        PIDFCoefficients pidModified = brrr.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        brrr.setPower(x);


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

      else  {

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

          else{

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



        telemetry.addData("Runtime", "%.03f", getRuntime());
        telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                pidOrig.p, pidOrig.i, pidOrig.d);
        telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                pidModified.p, pidModified.i, pidModified.d);


        telemetry.addData("Speed", brrr.getVelocity());
        telemetry.addData("Power", brrr.getPower());
        telemetry.addData("Wobble counts", wobble.getCurrentPosition());
        telemetry.addData("wobble servo position", wobbleServo.getPosition());

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
}
