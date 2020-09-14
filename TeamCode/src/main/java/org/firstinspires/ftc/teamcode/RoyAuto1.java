/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.lang.Math;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Roy Auto IMU", group="Linear Opmode")
//@Disabled
public class RoyAuto1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private BNO055IMU imu = null;
    private int leftStartPos = 0;
    private int rightStartPos = 0;

    public void initIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = null; // new JustLogginngAccelerationIntegrator();

        imu.initialize(parameters);
        // Start the logging of measured acceleration
         imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        telemetry.addData("Gyro", "calibrating...");
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && !isStopRequested() && timer.seconds() < 12) {
            sleep(1100);
        }
        if (imu.isGyroCalibrated())
            telemetry.addData("Gyro", "IMU Ready");
        else
            telemetry.addData("Gyro", "Gyro IMU Calibration FAILED !!!!!!!!!!!!!!");

        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        telemetry.update();

        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
        RobotLog.d("IMU calib: %s", imu.getCalibrationStatus().toString());
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        // START

        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Position", imu.getPosition());
            telemetry.addData("Acceleration", imu.getAcceleration());
            telemetry.addData("Heading", getHeading());
            telemetry.update();

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // game Pad Actions
            // game Pad Actions
            // game Pad Actions
            // game Pad Actions
            // game Pad Actions

            if (gamepad1.y) {
                driveForward(2);
            }
            if (gamepad1.x) {
                driveForward(-2);
            }
            if (gamepad1.a) {
                turnRight(90);
            }
            if (gamepad1.b) {
                triangle();
            }
            if (gamepad1.left_bumper) {
                square();
            }


        }
    }


    // drive functions
    // drive functions
    // drive functions
    // drive functions
    // drive functions
    // drive functions
    // drive functions
    // drive functions

    public void drive(double meter) {
        ElapsedTime timer = new ElapsedTime();
        double k = 2500; // [meter pre sec]
        int timetometer = (int) (meter * k);
        while (timer.milliseconds() < Math.abs(timetometer)) {
            double power;
            if (meter > 0) {
                power = 0.5;
            } else {
                power = -0.5;
            }
            leftDrive.setPower(power);
            rightDrive.setPower(power);

        }
        stopPower();
    }



    public void resetPosition(){
        leftStartPos = leftDrive.getCurrentPosition();
        rightStartPos = rightDrive.getCurrentPosition();

    }


    public double getForwardDistance(){
        int polsMeter = 2500;
        int left_tick = leftDrive.getCurrentPosition() - leftStartPos;
        int right_tick = rightDrive.getCurrentPosition() - rightStartPos;
        double leftDist = left_tick/polsMeter;
        double rightDist = right_tick/polsMeter;
        double evgPols = (leftDist + rightDist) / 2;
        return evgPols;
    }

    public void setPower(double forward, double turn){
        double leftPower = Range.clip(forward + turn, -1.0, 1.0);
        double rightPower = Range.clip(forward - turn, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void driveForward(double meter) {
        double targetAngle = getHeading(); // zeroAngle
        driveForward(meter, targetAngle);
    }


    public void driveForward(double meter, double targetAngle) {
        //ElapsedTime timer = new ElapsedTime();
        // double k = 2000; // [meter pre sec]
        // Position startPosition = imu.getPosition();
        // double timetometer = (int) (meter * k);
        double power = 0.8;
        double s = (meter<0) ? -1 : 1;

        resetPosition();

        while ((getForwardDistance() * s < meter * s) && opModeIsActive()){
            double currentAngle = getHeading();
            double err = getDeltaHeading(targetAngle);
            double gain = 0.035;
            double correction = gain * err;

            setPower(power * s, correction);

            telemetry.addData("target", targetAngle);
            telemetry.addData("current", currentAngle);
            telemetry.addData("Error", err);
            telemetry.update();
        }
        stopPower();
    }

    // bonus functions
    // bonus functions
    // bonus functions
    // bonus functions
    // bonus functions

    public void square() {
            double heading = getHeading();
            driveForward(2, heading);
            turnRight(90);
            driveForward(2, heading + 90);
            turnRight(90);
            driveForward(2,heading + 180);
            turnRight(90);
            driveForward(2, heading -90);
            turnRight(90);
            driveForward(2, heading);


    }

    public void triangle() {
        double heading = getHeading();
        turnRight(60);
        driveForward(4, heading + 60);
        turnRight(120);
        driveForward(4,heading + 120);
        turnRight(60);

        driveForward(4, heading + 0);
    }

    public double getImuDistance(Position target){
        Position current = imu.getPosition();
        double dx = current.x - target.x;
        double dy = current.y - target.y;
        double sqrt = Math.pow(dy,2) + Math.pow(dx,2);
        double distance = Math.sqrt(sqrt);
        return distance;
    }


    public double getDeltaHeading(double target) {
        double currentAngle = getHeading();
        double delta = target - currentAngle ;

        if (delta < 180){
            delta = delta + 360;
        }
        if (delta > 180){
            delta = delta - 360;
        }

        return delta;
    }


    double getHeading() {
        Orientation orie = imu.getAngularOrientation();
        double angle = -orie.firstAngle;
        return angle;
    }


    public void stopPower() {

        setPower(0,0);
    }

    public void turnRight(double deg) {
        double targetPower = 0.7;
        double targetAngle = getHeading() + deg; // zeroAngle
        double s = (deg<0) ? -1 : 1;
        double delta = getDeltaHeading(targetAngle);
        while ((delta *s > 0 *s) && opModeIsActive()) {
            delta = getDeltaHeading(targetAngle);
            double gain = 0.1;
            double power = gain * delta * targetPower;
            if (Math.abs(power) < 0.2)
                power = 0.2 * Math.signum(power);

            setPower(0, power);

            telemetry.addData("target", targetAngle);
            telemetry.addData("current", getHeading());
            telemetry.addData("delta", delta);
            telemetry.addData("power", power);
            telemetry.update();

        }
        stopPower();
    }
 }