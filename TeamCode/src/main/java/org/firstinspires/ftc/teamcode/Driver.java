package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class Driver {

    public static final boolean DEBUG = true;

    private static final double TOLERANCE_M = 0.01;
    private static final double TOLERANCE_DEG = 0.5;

    /* Counts per revolution, found on the product page for the motor */
    private static final double ENCODER_CPR = 384.5;
    private static final double WHEEL_RADIUS_M = 0.052;
    /* TODO: measure chassis */
    private static final double CHASSIS_LENGTH_M = 0.300;
    private static final double CHASSIS_WIDTH_M = 0.300;


    /*
     * Since we are going to be using Limelight
     * cameras, we will also be using their
     * units in the autonomous code
     * Limelight uses meters for position and
     * degrees for rotation
     */
    public static class Pose {
        private final double x, y, heading;
        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getHeading() {
            return heading;
        }
    }
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final Telemetry telemetry;
    double oldFlEncoder;
    double oldFrEncoder;
    double oldBlEncoder;
    double oldBrEncoder;
    double rotated;

    public Driver(
            DcMotorEx frontLeft, DcMotorEx frontRight,
            DcMotorEx backLeft, DcMotorEx backRight,
            Telemetry telemetry
            ) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        telemetry.addLine("Driver initialized");
        telemetry.update();
    }

    /*
     * This method needs to be called in a loop
     * We don't use a loop inside the method since
     * the main OpMode loop cannot block (force
     * the controller to wait)
     */
    /**
     * Drive to a relative position
     *
     * @param dPose Relative target pose
     * @param speed Speed to move with, 0 to 1
     * @param reset Resets encoder and rotation
     *              counters
     * @return Remaining relative target pose to
     * pass back to this method, or null if done
     */
    public Pose driveTo(Pose dPose, double speed, boolean reset) {
        if(reset) {
            frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            oldFlEncoder = 0;
            oldFrEncoder = 0;
            oldBlEncoder = 0;
            oldBrEncoder = 0;
            rotated = 0;
        }
        if(dPose == null) return null;
        if(
                Math.abs(dPose.getX()) < TOLERANCE_M &&
                Math.abs(dPose.getY()) < TOLERANCE_M &&
                Math.abs(dPose.getHeading()) < TOLERANCE_DEG
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

        /*
         * To determine distance moved using the
         * encoders we can reverse the standard
         * mecanum movement equations,
         * for example for strafing (left and right):
         * [ dfl, dfr ] = [ y,-y ]
         * [ dbl, dbr ] = [-y, y ]
         * Plugging in the equation for wheel movement,
         * d=rΔθ
         * we get (after reversing):
         * [ y,-y ] = [ rΔθfl, rΔθfr ]
         * [-y, y ] = [ rΔθbl, rΔθbr ]
         * This simplifies to:
         * [ y, y ] = [ rΔθfl,-rΔθfr ]
         * [ y, y ] = [-rΔθbl, rΔθbr ]
         * Then calculate average of each wheel:
         * Δy = (rΔθfl + -rΔθfr + -rΔθbl + rΔθbr) / 4
         * And factor out r:
         * Δy = (r / 4)(Δθfl + -Δθfr + -Δθbl + Δθbr)
         *
         * To calculate forward and backward movement
         * (X axis) we add all wheel rotations since
         * moving forward spins all wheels forward.
         * Rotation is a little different since we need
         * to account for the chassis dimensions.
         * Given the equation:
         * [ dfl, dfr ] = [ ϕ,-ϕ ]
         * [ dbl, dbr ] = [ ϕ,-ϕ ]
         * We get (using the same principle as above):
         * Δϕ = (r / 4)(Δθfl + -Δθfr + Δθbl + -Δθbr)
         * Given L = distance from center to wheels
         * along the X axis and W = distance from
         * center to wheels along the Y axis, we
         * divide by L + W to get rotation in radians:
         * Δϕ = (r / 4(L + W))(Δθfl + -Δθfr + Δθbl + -Δθbr)
         */
        double dx = (WHEEL_RADIUS_M / 4) * (flRadians + frRadians + blRadians + brRadians);
        double dy = (WHEEL_RADIUS_M / 4) * (flRadians - frRadians - blRadians + brRadians);
        double drx = (WHEEL_RADIUS_M / (4 * (CHASSIS_LENGTH_M / 2 + CHASSIS_WIDTH_M / 2))) *
                (flRadians - frRadians - blRadians + brRadians);

        rotated += drx;
        if(rotated > 2 * Math.PI) rotated -= 2 * Math.PI;

        /*
         * Now we have the distance moved, but we need to
         * account for rotation by converting the
         * x and y distances using this calculation:
         * [ x' ] = [ cosϕ,-sinϕ ] [ x ]
         * [ y' ] = [ sinϕ, cosϕ ] [ y ]
         * Which simplifies to:
         * x' = cosϕ * x + -sinϕ * y
         * y' = sinϕ * x + cosϕ * y
         */
        double g_dx = Math.cos(rotated) * dx - Math.sin(rotated) * dy;
        double g_dy = Math.sin(rotated) * dx + Math.cos(rotated) * dy;

        Pose pose = new Pose(
                dPose.getX() - g_dx,
                dPose.getY() - g_dy,
                dPose.getHeading() - drx
        );

        double x = speed * Math.signum(pose.getX());
        double y = speed * Math.signum(pose.getY());
        double rx = speed * Math.signum(pose.getHeading());

        frontLeft.setVelocity(x + y + rx, AngleUnit.RADIANS);
        frontRight.setVelocity(x - y - rx, AngleUnit.RADIANS);
        backLeft.setVelocity(x - y + rx, AngleUnit.RADIANS);
        backRight.setVelocity(x + y - rx, AngleUnit.RADIANS);

        /*
         * These telemetry calls would dump a
         * massive amount of data, so we only log
         * if we enable DEBUG when compiling
         */
        if(DEBUG) {
            telemetry.addData("Moved",
                    String.format(Locale.ENGLISH, "%f,%f,%f", g_dx, g_dy, drx));
            telemetry.addData("Moved local",
                    String.format(Locale.ENGLISH, "%f,%f", dx, dy));
            telemetry.addData("Radians",
                    String.format(Locale.ENGLISH, "%f,%f,%f,%f",
                            flRadians, frRadians, blRadians, brRadians
                    ));
            telemetry.addData("Wrote",
                    String.format(Locale.ENGLISH, "%f,%f,%f", x, y, rx));
            telemetry.addData("Rotation", rotated);
            telemetry.update();
        }

        oldFlEncoder = flEncoder;
        oldFrEncoder = frEncoder;
        oldBlEncoder = blEncoder;
        oldBrEncoder = brEncoder;

        return pose;
    }
}
