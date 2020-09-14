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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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

@TeleOp(name="Orri_Scorpion", group="Linear Opmode")
//@Disabled
public class OrriScorpion extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private BNO055IMU imu = null;
    private int leftBackStartPos = 0;
    private int rightBackStartPos = 0;
    private int leftFrontStartPos = 0;
    private int rightFrontStartPos = 0;

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

    public void resetPosition(){
        leftBackStartPos = leftBackDrive.getCurrentPosition();
        rightBackStartPos = rightBackDrive.getCurrentPosition();
        leftFrontStartPos = leftFrontDrive.getCurrentPosition();
        rightFrontStartPos = rightFrontDrive.getCurrentPosition();

    }

    public void strafe(double meter) {
        double targetAngle = getHeading(); // zeroAngle
        strafe(meter, targetAngle);
    }


    public void strafe(double meter, double targetAngle) {
        //ElapsedTime timer = new ElapsedTime();
        // double k = 2000; // [meter pre sec]
        // Position startPosition = imu.getPosition();
        // double timetometer = (int) (meter * k);
        double power = 0.8;
        double s = (meter<0) ? -1 : 1;

        resetPosition();

        while ((getStrafeDistance() * s < meter * s) && opModeIsActive()){
            double currentAngle = getHeading();
            double err = getDeltaHeading(targetAngle);
            double gain = 0.035;
            double correction = gain * err;

            setPower(0, correction, power * s);

            telemetry.addData("distance", getStrafeDistance());
            telemetry.addData("target", targetAngle);
            telemetry.addData("current", currentAngle);
            telemetry.addData("Error", err);
            telemetry.update();
        }
        stopPower();
    }

    public double getForwardDistance(){
        double polsMeter = 1150;
        int leftBack_tick = leftBackDrive.getCurrentPosition() - leftBackStartPos;
        int rightBack_tick = rightBackDrive.getCurrentPosition() - rightBackStartPos;
        int leftFront_tick = leftFrontDrive.getCurrentPosition() - leftFrontStartPos;
        int rightFront_tick = rightFrontDrive.getCurrentPosition() - rightFrontStartPos;
        double leftFrontDist = leftFront_tick/polsMeter;
        double rightFrontDist = rightFront_tick/polsMeter;
        double leftBackDist = leftBack_tick/polsMeter;
        double rightBackDist = rightBack_tick/polsMeter;
        double evgPols = (leftBackDist + rightBackDist + rightFrontDist + leftFrontDist) / 4;
        return evgPols;
    }
    public double getStrafeDistance(){
        double polsMeter = 1000;
        int leftBack_tick = leftBackDrive.getCurrentPosition() - leftBackStartPos;
        int rightBack_tick = rightBackDrive.getCurrentPosition() - rightBackStartPos;
        int leftFront_tick = leftFrontDrive.getCurrentPosition() - leftFrontStartPos;
        int rightFront_tick = rightFrontDrive.getCurrentPosition() - rightFrontStartPos;
        double leftFrontDist = leftFront_tick/polsMeter;
        double rightFrontDist = rightFront_tick/polsMeter;
        double leftBackDist = leftBack_tick/polsMeter;
        double rightBackDist = rightBack_tick/polsMeter;
        // double forwardDistance = (leftBackDist + rightBackDist + rightFrontDist + leftFrontDist) / 4;
        double strafeDistance = (leftBackDist + rightBackDist - rightFrontDist + leftFrontDist) / 4;
        return strafeDistance;
    }

    public void setPower(double forward, double turn, double strafe){
        leftFrontDrive.setPower(forward + turn + strafe);
        rightFrontDrive.setPower(forward - turn - strafe);

        leftBackDrive.setPower(forward + turn - strafe);
        rightBackDrive.setPower(forward - turn + strafe);
    }


    public void strafeSquare() {
        double heading = getHeading();
        driveForward(2, 0.7, heading);
        strafe(2, heading);
        driveForward(-2, 0.7, heading);
        strafe(-2, heading);
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
        setPower(0,0,0);
    }


    public void turn(double deg, double power) {
        double targetAngle = getHeading() + deg; // zeroAngle
        turnTo(targetAngle, power);
    }

    public void turnTo(double targetAngle, double targetPower) {
        double delta = getDeltaHeading(targetAngle);
        double s = (delta < 0) ? -1 : 1;
        while ((delta *s > 5 * s) && opModeIsActive()) {

            delta = getDeltaHeading(targetAngle);
            double gain = 0.04;
            double power = gain * delta * targetPower;
            if (Math.abs(power) < 0.1)
                power = 0.1 * Math.signum(power);

            setPower(0, power,0);

            telemetry.addData("target", targetAngle);
            telemetry.addData("current", getHeading());
            telemetry.addData("delta", delta);
            telemetry.addData("power", power);
            telemetry.update();

        }
        stopPower();
    }

    public void square() {
        double heading = getHeading();
        for (int i = 0; i<4; i++) {
            driveForward(1, 0.5, heading);
            heading += 90;
            turnTo(heading,0.5);
        }
    }

    public void driveForward(double meter, double power) {
        double targetAngle = getHeading(); // zeroAngle
        driveForward(meter, power, targetAngle);
    }


    public void driveForward(double meter, double power, double targetAngle) {
        //ElapsedTime timer = new ElapsedTime();
        // double k = 2000; // [meter pre sec]
        // Position startPosition = imu.getPosition();
        // double timetometer = (int) (meter * k);
        double s = (meter<0) ? -1 : 1;

        resetPosition();

        while ((getForwardDistance() * s < meter * s) && opModeIsActive()){
            double currentAngle = getHeading();
            double err = getDeltaHeading(targetAngle);
            double gain = 0.040;
            double correction = gain * err;
            double breakGain = 1;
            double delta = Math.abs(meter - getForwardDistance());
            double breakPower =     delta * breakGain;

            if (breakPower < power)
                power = breakPower;


            setPower(power * s, correction, 0);

            telemetry.addData("target", targetAngle);
            telemetry.addData("current", currentAngle);
            telemetry.addData("Error", err);
            telemetry.update();
        }
        stopPower();
    }


    public void triangle() {
        double heading = getHeading();
        driveForward(1, 0.5, heading );
        heading += 90;
        turnTo(heading, 0.5);
        driveForward(1, 0.5, heading);
        heading += 135;
        turnTo(heading, 0.5);
        driveForward(1.35, 0.5, heading);
    }



    // main functions
    // main functions
    // main functions
    // main functions
    // main functions

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive = hardwareMap.get(DcMotor.class, "bl_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "br_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fr_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fl_drive");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);

        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            double strafe = gamepad1.left_stick_x;
            double boost = gamepad1.left_trigger * 0.5 + 0.5;
            if (boost != 0)
                setPower(drive * boost, turn * boost, strafe * boost);
            else 
                setPower(drive, turn, strafe);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("StrafePos", getStrafeDistance());
            telemetry.addData("Position", getForwardDistance());
            telemetry.addData("Acceleration", imu.getAcceleration());
            telemetry.addData("Heading", getHeading());
            telemetry.update();

            // game Pad Actions
            // game Pad Actions
            // game Pad Actions
            // game Pad Actions
            // game Pad Actions

            if (gamepad1.right_bumper)
                triangle();

            if (gamepad1.a) {
                driveForward(4, 0.5);
            }
            if (gamepad1.b) {
                driveForward(-4, 0.5);
            }
            if (gamepad1.x) {
                square();
            }
            if (gamepad1.y) {
                strafe(1);
            }
            if (gamepad1.dpad_up) {
                strafeSquare();
            }

        }
    }


}