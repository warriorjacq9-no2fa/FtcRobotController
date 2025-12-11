package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousOpMode", group="Bot")
public class AutonomousOpMode extends OpMode {

    private class Res3D {
        public double tx, ty, ta;
        public Res3D(double tx, double ty, double ta) {
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
        }
    }

    // All length/distance values are in inches

    // Parameters
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double TURN_RADIUS = 10;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Limelight3A limelight;

    private enum AutonomousState {
        TARGETING,
        DRIVING,
        ROTATING,
        LAUNCHING,
        DONE
    }

    // Set all motor modes at once to clean up code
    private void m_setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private Res3D llRead() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            return new Res3D(tx, ty, ta); 
        } else {
            return null;
        }
    }

    private boolean drive(double speed, double dx, double dy) {

        if(dx == 0 && dy == 0) return true;

        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Divide by cos(45) (or sqrt(2)) to account for movement loss
        // during strafing (side-to-side)
        int dxIn = (int)(dx * COUNTS_PER_INCH / Math.cos(45));
        int dyIn = (int)(dy * COUNTS_PER_INCH);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + dyIn + dxIn;
        newFrontRightTarget = frontRight.getCurrentPosition() + dyIn - dxIn;
        newBackLeftTarget = backLeft.getCurrentPosition() + dyIn - dxIn;
        newBackRightTarget = backRight.getCurrentPosition() + dyIn + dxIn;

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        m_setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        if(!(frontLeft.isBusy() && frontRight.isBusy()
                        && backLeft.isBusy() && backRight.isBusy())) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            m_setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return true;
        }

        return false;
    }

    private boolean rotate(double speed, double degrees) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        double dr = degrees * (TURN_RADIUS * (Math.PI / 180));

        //int drIn = (int)(dr * COUNTS_PER_INCH);
        int drIn = 5;

        newFrontLeftTarget = frontLeft.getCurrentPosition() + drIn;
        newFrontRightTarget = frontRight.getCurrentPosition() - drIn;
        newBackLeftTarget = backLeft.getCurrentPosition() + drIn;
        newBackRightTarget = backRight.getCurrentPosition() - drIn;

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        m_setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        if(!(frontLeft.isBusy() && frontRight.isBusy()
                        && backLeft.isBusy() && backRight.isBusy())) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            m_setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return true;
        }

        return false;
    }

    double posX, posY, posR;

    private AutonomousState state;

    @Override
    public void init() {
        posX = 0;
        posY = 0;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();*/

        m_setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Don't run autonomous until driver presses start
        state = AutonomousState.DONE;
    }

    @Override
    public void start() {
        rotate(1, 45);
        // Now we can start autonomous
        //state = AutonomousState.TARGETING;
    }

    double targetX = 0, targetY = 0, targetR = 0;
    @Override
    public void loop() {
        switch(state) {
            case TARGETING:
                Res3D target = llRead();
                targetR = target.tx;
                state = AutonomousState.DRIVING;
                break;
            case DRIVING:
                if(drive(DRIVE_SPEED, posX - targetX, posY - targetY)) {
                    posX = targetX;
                    posY = targetY;
                    state = AutonomousState.ROTATING;
                }
                break;
            case ROTATING:
                if(rotate(TURN_SPEED, posR - targetR)) {
                    posR = targetR;
                    state = AutonomousState.LAUNCHING;
                }
            case LAUNCHING:
                // TODO: launching
                state = AutonomousState.DONE;
                break;
        }
    }
}
