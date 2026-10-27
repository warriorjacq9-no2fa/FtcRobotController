package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name ="TestTele0p")
public class Teleop extends OpMode {

    private static final double SPEED = (750) / 52.0;
    private DcMotorEx FrontLeft = null;
    private DcMotorEx FrontRight = null;
    private DcMotorEx BackLeft = null;
    private DcMotorEx BackRight = null;

    public void init(){

        /* o-^ */ FrontLeft = hardwareMap.get(DcMotorEx.class,"frontLeft");
        /* o-v */ BackLeft = hardwareMap.get(DcMotorEx.class,"backLeft");
            /* ^-o */ FrontRight = hardwareMap.get(DcMotorEx.class,"frontRight");
            /* v-o */ BackRight = hardwareMap.get(DcMotorEx.class,"backRight");
        /* Wheel Variable = Irl Wheel from DcMotot.class */

        /* o-^ */ FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);  /* Set Left Normal Direction */
        /* o-v */ BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);   /* Set Left Normal Direction */
            /* ^-o */ FrontRight.setDirection(DcMotorSimple.Direction.REVERSE); /* Set Right Reversed */
            /* v-o */ BackRight.setDirection(DcMotorSimple.Direction.REVERSE);  /* Set Right Reversed */
        /* Set Right Wheels to Reverse, Set Left Wheels to normal direction (Forward) */

        /* o-^ */ FrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);  /* Speed 0 = Break */
        /* o-v */ BackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);   /* Speed 0 = Break */
         /* ^-o */ FrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); /* Speed 0 = Break */
            /* v-o */  BackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);  /* Speed 0 = Break */
        /* Set Wheel Behavior when Speed = 0 To break */

    }

    private void drive() {
        /* (^\v) () */  double forward = -gamepad1.left_stick_y;  /* Set Y-axis of Left Stick to Forward */
        /* () (<\>) */  double strafe = gamepad1.left_stick_x;    /* Set X-axis of Left Stick to Strafe */
        /* () (<\>) */  double turn = gamepad1.right_stick_x;     /* Set Y-axis of Right Stick to Turn */
        double FLPower = ( forward + strafe + turn );
        double FRPower = ( forward + strafe - turn );
        double BLPower = ( forward - strafe + turn );
        double BRPower = ( forward - strafe - turn );
        FrontLeft.setPower( FRPower );
        FrontRight.setPower( FLPower );
        BackLeft.setPower( BLPower );
        BackRight.setPower( BRPower );
        Display.(FLPower)
        public Void Display(W,P)
        String FL = "x";
        String BL = "x";
        String FR = "x";
        String BR = "x";

        if ( ( forward + strafe + turn ) <= 0 ) {
            FL = "^";
        } if ( ( forward + strafe + turn ) >= 0 ) {
            FL = "v";
        } else {
            FL = "o";
        }


        ( forward + strafe - turn );

        ( forward - strafe + turn );
        ( forward - strafe - turn );


        telemetry.addLine("n/a");
        telemetry.update();
    }


}
