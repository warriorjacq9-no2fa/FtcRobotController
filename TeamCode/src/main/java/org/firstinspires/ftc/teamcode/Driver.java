package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.Command;

import java.util.ArrayDeque;
import java.util.Queue;

public class Driver {

    public static final boolean DEBUG = true;

    private static final double TOLERANCE_M = 0.01;
    private static final double TOLERANCE_RAD = 0.02;

    /* Counts per revolution, found on the product page for the motor */
    private static final double ENCODER_CPR = 384.5;
    private static final double WHEEL_RADIUS_M = 0.052;
    private static final double ROBOT_LENGTH_M = 0.285; /* Front-back from wheel centers */
    private static final double ROBOT_WIDTH_M = 0.415; /* left-right from wheel centers */

    /* PID constants */
    private static final double Kp = 10;
    private static final double Ki = 0.025;
    private static final double Kd = 0;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final IMU imu;
    private final Telemetry telemetry;
    private double oldFlEncoder;
    private double oldFrEncoder;
    private double oldBlEncoder;
    private double oldBrEncoder;
    private double heading;
    private double oldHeading;
    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastTime = 0;
    private double xIntegral = 0;
    private double yIntegral = 0;
    private double rxIntegral = 0;
    private double xDerivative = 0;
    private double yDerivative = 0;
    private double rxDerivative = 0;
    private double oldXError = 0;
    private double oldYError = 0;
    private double oldRxError = 0;

    public Driver(
            DcMotorEx frontLeft, DcMotorEx frontRight,
            DcMotorEx backLeft, DcMotorEx backRight,
            IMU imu,
            Telemetry telemetry
            ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
        this.telemetry = telemetry;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        heading = 0;
        oldHeading = 0;

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * RUN_USING_ENCODER uses the motor
         * controller's integrated hardware
         * PID control, which helps to control
         * wheel slip
         */
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        state = DriverState.GET_COMMAND;

        telemetry.addLine("Driver initialized");
        telemetry.update();
    }

    double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    double filter(double a, double b) {
        return 0.2 * a + (1 - 0.2) * b;
    }

    /**
     * Drive to a given relative pose in the
     * global frame
     * @param dPose Relative target pose
     * @param speed Speed, in distanceUnits/sec
     * @param angleUnits Angle units for pose
     * @param distanceUnits Distance units for
     *                      pose and speed
     * @return Value to pass back to this
     * function next iteration, or null if done.
     * Essentially dPose - poseMoved
     */
    public Pose drive(
            Pose dPose, double speed,
            AngleUnit angleUnits, DistanceUnit distanceUnits
    ) {
        heading = normalize(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        if(dPose == null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return null;
        }
        double x = distanceUnits.toMeters(dPose.x);
        double y = distanceUnits.toMeters(dPose.y);
        double rx = angleUnits.toRadians(dPose.heading);
        if(
                Math.abs(x) < TOLERANCE_M &&
                Math.abs(y) < TOLERANCE_M &&
                Math.abs(rx) < TOLERANCE_RAD
        ) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            return null;
        }

        double flEncoder = frontLeft.getCurrentPosition();
        double frEncoder = frontRight.getCurrentPosition();
        double blEncoder = backLeft.getCurrentPosition();
        double brEncoder = backRight.getCurrentPosition();

        double flRadians = ((flEncoder - oldFlEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double frRadians = ((frEncoder - oldFrEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double blRadians = ((blEncoder - oldBlEncoder) / ENCODER_CPR) * 2 * Math.PI;
        double brRadians = ((brEncoder - oldBrEncoder) / ENCODER_CPR) * 2 * Math.PI;

        double dx = (WHEEL_RADIUS_M / 4) * (flRadians + frRadians + blRadians + brRadians);
        double dy = (WHEEL_RADIUS_M / 4) * (flRadians - frRadians - blRadians + brRadians);

        double dHeading = normalize(heading - oldHeading);

        double g_dx = Math.cos(heading) * dx - Math.sin(heading) * dy;
        double g_dy = Math.sin(heading) * dx + Math.cos(heading) * dy;

        Pose pose = new Pose(
                dPose.x - distanceUnits.fromMeters(g_dx),
                dPose.y - distanceUnits.fromMeters(g_dy),
                dPose.heading - angleUnits.fromRadians(dHeading)
        );

        double xError = Math.cos(heading) * distanceUnits.toMeters(pose.x) +
                Math.sin(heading) * distanceUnits.toMeters(pose.y);
        double yError = -Math.sin(heading) * distanceUnits.toMeters(pose.x) +
                Math.cos(heading) * distanceUnits.toMeters(pose.y);
        double rxError = angleUnits.toRadians(pose.heading);

        double currentTime = loopTimer.seconds();
        double loopTime = currentTime - lastTime;
        lastTime = currentTime;

        xIntegral += xError * loopTime;
        yIntegral += yError * loopTime;
        rxIntegral += rxError * loopTime;

        xDerivative = filter((xError - oldXError) / loopTime, xDerivative);
        yDerivative = filter((yError - oldYError) / loopTime, yDerivative);
        rxDerivative = filter((rxError - oldRxError) / loopTime, rxDerivative);

        double xUt = Kp * xError + Ki * xIntegral + Kd * xDerivative;
        double yUt = Kp * yError + Ki * yIntegral + Kd * yDerivative;
        double rxUt = Kp * rxError + Ki * rxIntegral + Kd * rxDerivative;

        oldXError = xError;
        oldYError = yError;
        oldRxError = rxError;

        double wheelX = xUt;
        double wheelY = yUt;
        double wheelRx = rxUt;

        double flSpeed = wheelX + wheelY + wheelRx;
        double frSpeed = wheelX - wheelY - wheelRx;
        double blSpeed = wheelX - wheelY + wheelRx;
        double brSpeed = wheelX + wheelY - wheelRx;

        frontLeft.setVelocity(flSpeed, AngleUnit.RADIANS);
        frontRight.setVelocity(frSpeed, AngleUnit.RADIANS);
        backLeft.setVelocity(blSpeed, AngleUnit.RADIANS);
        backRight.setVelocity(brSpeed, AngleUnit.RADIANS);

        if(DEBUG) {
            telemetry.addData("Moved wheel", "%f %f %f %f",
                    flRadians, frRadians, blRadians, brRadians
            );
            telemetry.addData("Moved local", "%f %f %f", dx, dy, (oldHeading - heading));
            telemetry.addData("Moved global", "%f %f", g_dx, g_dy);
            telemetry.addData("Next global", "%f %f %f",
                    distanceUnits.toMeters(pose.x), distanceUnits.toMeters(pose.y),
                    angleUnits.toRadians(pose.heading)
            );
            telemetry.addData("Error", "%f %f %f", xError, yError, rxError);
            telemetry.addData("Wheel speeds", "%f %f %f %f",
                    flSpeed, frSpeed, blSpeed, brSpeed
            );
            telemetry.addData("PID out", "%f %f %f", xUt, yUt, rxUt);
            telemetry.addData("PID integrals", "%f %f %f",
                    xIntegral, yIntegral, rxIntegral
            );
            telemetry.addData("PID derivatives", "%f %f %f",
                    xDerivative, yDerivative, rxDerivative
            );
            telemetry.addData("Loop time", "%f", loopTime);
        }

        oldFlEncoder = flEncoder;
        oldFrEncoder = frEncoder;
        oldBlEncoder = blEncoder;
        oldBrEncoder = brEncoder;
        oldHeading = heading;

        return pose;
    }

    private enum DriverState {
        GET_COMMAND,
        RUN_COMMAND
    }

    private DriverState state;
    private Queue<Command> commands = new ArrayDeque<>();
    private Command currentCmd;

    /**
     * To be called during loop() in an OpMode
     */
    public void loop() {
        heading = normalize(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.addData("Heading", heading);
        switch(state) {
            case GET_COMMAND:
                Command cmd = commands.peek();
                if(cmd != null) {
                    currentCmd = cmd;
                    commands.remove();
                    state = DriverState.RUN_COMMAND;
                }
                break;

            case RUN_COMMAND:
                currentCmd.run();
                if(currentCmd.isDone())
                    state = DriverState.GET_COMMAND;
                break;
        }
        telemetry.addData("Driver state", state);
        telemetry.addData("currentCmd", currentCmd);
        telemetry.addData("Commands left", commands.size());
    }

    /**
     * Queue a command in the command queue to
     * be processed and executed
     *
     * @param cmd Command to be queued
     */
    public void doCommand(Command cmd) {
        commands.add(cmd);
    }

    /**
     * Check if current command is done
     *
     * @return True if done, false otherwise
     */
    public boolean isCurrentCommandDone() {
        if(currentCmd == null) return true;
        return currentCmd.isDone();
    }

    /**
     * Check if the command queue is empty
     *
     * @return True if empty, false otherwise
     */
    public boolean isCommandsEmpty() {
        return commands.isEmpty();
    }
}
