// This file has been AI Generated.
package org.firstinspires.ftc.teamcode.driver;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.DRIVE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.STRAFE_KP;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KD;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KI;
import static org.firstinspires.ftc.teamcode.support.ConstantsPID.YAW_KP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.support.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.support.PIDController;

import java.util.Locale;

/**
 * Hold Position with active disturbance rejection.
 *
 * How interference (pushes / spins) is handled:
 *
 * 1. VELOCITY DAMPING - While holding, measured odometry velocity is fed back
 *    negatively on top of the position PID. A push or spin creates velocity
 *    BEFORE it creates much position error, so the robot resists instantly
 *    instead of waiting to drift off target. This is the primary defense.
 *
 * 2. DEDICATED HOLD CONTROLLERS - Zero tolerance/deadband so small persistent
 *    offsets (robot resting against another robot) are still corrected.
 *
 * 3. ADAPTIVE POWER - Normal holding uses a low, quiet power cap. When a
 *    disturbance is detected (error or velocity above threshold), the cap
 *    temporarily rises so the robot can fight back, then relaxes after
 *    CALM_DOWN_MS of calm. Prevents both "too weak to resist" and
 *    "always full power, oscillating".
 *
 * 4. ABANDON / RECAPTURE - If displaced farther than ABANDON_DISTANCE_IN
 *    (e.g. dragged across the field), the old target is abandoned and the
 *    hold re-anchors at the current pose instead of charging back.
 *
 * 5. INTEGRAL MANAGEMENT - Controllers reset when entering hold and when a
 *    disturbance clears, preventing windup from sustained pushes.
 *
 * Conventions:
 *   Pinpoint:   +X forward, +Y left, +heading counterclockwise.
 *   Drivetrain: +axial forward, +lateral right, +yaw clockwise.
 *
 * TUNING:
 *   - Robot gets pushed too easily -> raise VEL_DAMP_* gains.
 *   - Robot buzzes/jitters while holding -> lower VEL_DAMP_* gains.
 *   - Then adjust DISTURB_* thresholds and the power caps.
 */
@TeleOp(name = "Hold Position Mirrored V2", group = "StarterBot")
public class HoldPositionMirrored_V2 extends OpMode {

    // ------------------------------------------------------------------
    // Hardware
    // ------------------------------------------------------------------
    private DcMotor leftFrontDrive;
    private DcMotor leftRearDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightRearDrive;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private CRServo windmillServo;

    private GoBildaPinpointDriver odo;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dBoardTelemetry = dashboard.getTelemetry();

    private Pose2D robotPosition;
    private Pose2D targetPos;

    private final ElapsedTime lastLaunchMillis = new ElapsedTime();
    private final ElapsedTime calmTimer = new ElapsedTime();

    // ------------------------------------------------------------------
    // Driving / holding constants
    // ------------------------------------------------------------------
    private static final double STICK_DEADBAND = 0.10;
    private static final double STRAFE_COMPENSATION = 1.10;

    // Power caps: calm vs disturbed
    private static final double HOLD_POWER_LIMIT = 0.35;
    private static final double HOLD_YAW_POWER_LIMIT = 0.30;
    private static final double DISTURBED_POWER_LIMIT = 0.70;
    private static final double DISTURBED_YAW_POWER_LIMIT = 0.60;

    // Velocity damping gains (power per unit velocity).
    // axial/strafe: power per (in/sec). yaw: power per (deg/sec).
    private static final double VEL_DAMP_AXIAL = 0.012;
    private static final double VEL_DAMP_STRAFE = 0.012;
    private static final double VEL_DAMP_YAW = 0.0035;

    // Disturbance detection thresholds
    private static final double DISTURB_POS_ERROR_IN = 1.5;
    private static final double DISTURB_HEADING_ERROR_DEG = 5.0;
    private static final double DISTURB_VEL_IPS = 6.0;
    private static final double DISTURB_HEADING_VEL_DPS = 45.0;

    // If displaced farther than this, abandon the old target entirely
    // (robot was carried/dragged - charging back would be dangerous).
    private static final double ABANDON_DISTANCE_IN = 30.0;

    // Must stay calm this long before dropping back to low-power hold
    private static final double CALM_DOWN_MS = 300;

    // ------------------------------------------------------------------
    // Launcher
    // ------------------------------------------------------------------
    public int launcher_target_velocity = 1265;
    public final int LAUNCHER_MIN_VELOCITY = 50;
    public final int LAUNCHER_TIMEOUT = 5000;

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;

    private boolean field_centric_drive = true;
    private boolean holdingPosition = false;
    private boolean disturbed = false;
    private boolean reachedTarget = false;
    private int launches = 0;

    // Current field-frame velocities, refreshed every loop
    private double velXips = 0;        // in/sec, +forward
    private double velYips = 0;        // in/sec, +left
    private double velHeadingDps = 0;  // deg/sec, +CCW

    // ------------------------------------------------------------------
    // PID controllers
    // ------------------------------------------------------------------
    // Hold controllers: NO tolerance, NO deadband so small or sustained
    // disturbances are fully corrected. Max output is 1.0 here; the real
    // adaptive cap is applied in holdPosition().
    private final PIDController holdStrafeController = new PIDController(
            STRAFE_KP, STRAFE_KI, STRAFE_KD, 1.0,
            0.0, 0.0, false);

    private final PIDController holdYawController = new PIDController(
            YAW_KP, YAW_KI, YAW_KD, 1.0,
            0.0, 0.0, true);

    private final PIDController holdDriveController = new PIDController(
            DRIVE_KP, DRIVE_KI, DRIVE_KD, 1.0,
            0.0, 0.0, false);

    // ==================================================================
    // OpMode lifecycle
    // ==================================================================
    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftRearDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        leftRearDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        rightRearDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(40, 0, 0, 12.5));

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);
        launcher.setVelocity(0);

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        odo.update();
        robotPosition = odo.getPosition();

        telemetry.addData("Status",
                "Initialized - Verify axes: +X fwd, +Y left, CCW +heading");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updatePosition();
        displayTelemetry();
    }

    @Override
    public void start() {
        targetPos = null;
        holdingPosition = false;
        disturbed = false;
        launches = 0;
        lastLaunchMillis.reset();
        calmTimer.reset();
    }

    @Override
    public void loop() {
        updatePosition();
        controllerButtons();
        handleDrive();
        handleIntake();
        launch();
        displayTelemetry();
    }

    @Override
    public void stop() {
        setDrivePower(0, 0, 0, 0);
        setIntakePower(0);
        launcher.setVelocity(0);
        windmillServo.setPower(0);
    }

    // ==================================================================
    // Odometry
    // ==================================================================
    private void updatePosition() {
        odo.update();
        robotPosition = odo.getPosition();

        // Cache velocities in field frame with consistent units
        velXips = DistanceUnit.INCH.fromMm(odo.getVelX(DistanceUnit.MM));
        velYips = DistanceUnit.INCH.fromMm(odo.getVelY(DistanceUnit.MM));
        velHeadingDps = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        dBoardTelemetry.update();
    }

    private double getXPosition(DistanceUnit unit) {
        return robotPosition != null ? robotPosition.getX(unit) : 0;
    }

    private double getYPosition(DistanceUnit unit) {
        return robotPosition != null ? robotPosition.getY(unit) : 0;
    }

    private double getHeading(AngleUnit unit) {
        return robotPosition != null ? robotPosition.getHeading(unit) : 0;
    }

    private boolean hasValidPose() {
        if (robotPosition == null) return false;
        double x = robotPosition.getX(DistanceUnit.MM);
        double y = robotPosition.getY(DistanceUnit.MM);
        double h = robotPosition.getHeading(AngleUnit.DEGREES);
        return !Double.isNaN(x) && !Double.isInfinite(x)
                && !Double.isNaN(y) && !Double.isInfinite(y)
                && !Double.isNaN(h) && !Double.isInfinite(h);
    }

    // ==================================================================
    // Driving
    // ==================================================================
    private static double applyDeadband(double value) {
        if (Math.abs(value) < STICK_DEADBAND) return 0.0;
        double scaled = (Math.abs(value) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        return Range.clip(Math.copySign(scaled, value), -1.0, 1.0);
    }

    private static double wrapDegrees(double angle) {
        angle = angle % 360.0;
        if (angle >= 180.0) angle -= 360.0;
        else if (angle < -180.0) angle += 360.0;
        return angle;
    }

    private void handleDrive() {
        if (!hasValidPose()) {
            holdingPosition = false;
            targetPos = null;
            setDrivePower(0, 0, 0, 0);
            return;
        }

        double axial = applyDeadband(-(gamepad1.left_stick_y + gamepad2.left_stick_y));
        double lateral = applyDeadband(gamepad1.left_stick_x + gamepad2.left_stick_x);
        double yaw = applyDeadband(gamepad1.right_stick_x + gamepad2.right_stick_x);

        boolean manualInput = axial != 0 || lateral != 0 || yaw != 0;

        if (manualInput) {
            holdingPosition = false;
            disturbed = false;
            targetPos = null;

            if (field_centric_drive) {
                drive(axial, lateral, yaw);
            } else {
                driveRobotRelative(axial, lateral, yaw);
            }
            return;
        }

        if (!holdingPosition || targetPos == null) {
            captureHoldTarget();
        }

        holdPosition();
    }

    private void captureHoldTarget() {
        targetPos = new Pose2D(
                DistanceUnit.MM,
                robotPosition.getX(DistanceUnit.MM),
                robotPosition.getY(DistanceUnit.MM),
                AngleUnit.DEGREES,
                robotPosition.getHeading(AngleUnit.DEGREES)
        );

        // Reset ONCE when entering hold so integral/derivative start clean
        holdDriveController.reset(targetPos.getX(DistanceUnit.INCH), 1);
        holdStrafeController.reset(targetPos.getY(DistanceUnit.INCH), 1);
        holdYawController.reset(targetPos.getHeading(AngleUnit.DEGREES), 1);

        disturbed = false;
        holdingPosition = true;
    }

    /**
     * Active position hold with velocity damping and adaptive power.
     */
    private void holdPosition() {
        double xErrorIn = DistanceUnit.INCH.fromMm(targetPos.getX(DistanceUnit.MM))
                - getXPosition(DistanceUnit.INCH);
        double yErrorIn = DistanceUnit.INCH.fromMm(targetPos.getY(DistanceUnit.MM))
                - getYPosition(DistanceUnit.INCH);
        double headingErrorDeg = wrapDegrees(
                targetPos.getHeading(AngleUnit.DEGREES) - getHeading(AngleUnit.DEGREES));

        double posErrorIn = Math.hypot(xErrorIn, yErrorIn);

        // --- Abandon target if we were carried/dragged far away ---
        if (posErrorIn > ABANDON_DISTANCE_IN) {
            captureHoldTarget();
            setDrivePower(0, 0, 0, 0);
            return;
        }

        // --- Disturbance detection ---
        boolean disturbedNow =
                posErrorIn > DISTURB_POS_ERROR_IN
                        || Math.abs(headingErrorDeg) > DISTURB_HEADING_ERROR_DEG
                        || Math.hypot(velXips, velYips) > DISTURB_VEL_IPS
                        || Math.abs(velHeadingDps) > DISTURB_HEADING_VEL_DPS;

        if (disturbedNow) {
            calmTimer.reset();
            if (!disturbed) {
                // Just got hit: clear accumulated integral from calm holding
                // so it doesn't combine with the new correction and overshoot.
                holdDriveController.reset(targetPos.getX(DistanceUnit.INCH), 1);
                holdStrafeController.reset(targetPos.getY(DistanceUnit.INCH), 1);
                holdYawController.reset(targetPos.getHeading(AngleUnit.DEGREES), 1);
            }
            disturbed = true;
        } else if (disturbed && calmTimer.milliseconds() > CALM_DOWN_MS) {
            // Disturbance is over and we've been calm long enough
            disturbed = false;
        }

        // --- Position correction (field frame) ---
        double axialCmd = holdDriveController.getOutputFromError(xErrorIn);
        double strafeCmd = holdStrafeController.getOutputFromError(yErrorIn);
        double yawCmd = holdYawController.getOutputFromError(headingErrorDeg);

        // --- Velocity damping: resist ANY motion while holding ---
        // A push creates velocity before it creates position error, so this
        // fights the push immediately. Negative sign opposes the motion.
        axialCmd -= VEL_DAMP_AXIAL * velXips;
        strafeCmd -= VEL_DAMP_STRAFE * velYips;
        yawCmd -= VEL_DAMP_YAW * velHeadingDps;

        // --- Adaptive power caps ---
        double linCap = disturbed ? DISTURBED_POWER_LIMIT : HOLD_POWER_LIMIT;
        double yawCap = disturbed ? DISTURBED_YAW_POWER_LIMIT : HOLD_YAW_POWER_LIMIT;

        axialCmd = Range.clip(axialCmd, -linCap, linCap);
        strafeCmd = Range.clip(strafeCmd, -linCap, linCap);
        yawCmd = Range.clip(yawCmd, -yawCap, yawCap);

        // Convert left-positive / CCW-positive corrections into the
        // drivetrain convention (right-positive / clockwise-positive).
        drive(axialCmd, -strafeCmd, -yawCmd);
    }

    /** Field-centric mecanum drive. */
    public void drive(double axial, double lateral, double yaw) {
        double headingRad = getHeading(AngleUnit.RADIANS);

        double strafe = lateral * Math.cos(-headingRad) - axial * Math.sin(-headingRad);
        double forward = lateral * Math.sin(-headingRad) + axial * Math.cos(-headingRad);

        driveRobotRelative(forward, strafe, yaw);
    }

    /** Robot-relative mecanum drive. forward+, strafe+ = right, yaw+ = clockwise. */
    private void driveRobotRelative(double forward, double strafe, double yaw) {
        strafe *= STRAFE_COMPENSATION;

        // Normalize using the TRANSFORMED/SCALED values, not raw inputs,
        // so no wheel command gets clipped and distorts the motion.
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(yaw), 1.0);

        setDrivePower(
                (forward + strafe + yaw) / denominator,
                (forward - strafe - yaw) / denominator,
                (forward - strafe + yaw) / denominator,
                (forward + strafe - yaw) / denominator
        );
    }

    private void setDrivePower(double lf, double rf, double lr, double rr) {
        leftFrontPower = Range.clip(lf, -1.0, 1.0);
        rightFrontPower = Range.clip(rf, -1.0, 1.0);
        leftBackPower = Range.clip(lr, -1.0, 1.0);
        rightBackPower = Range.clip(rr, -1.0, 1.0);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftBackPower);
        rightRearDrive.setPower(rightBackPower);
    }

    // ==================================================================
    // Gamepad buttons
    // ==================================================================
    public void controllerButtons() {
        if (gamepad1.squareWasPressed() || gamepad2.squareWasPressed()) {
            field_centric_drive = true;
        } else if (gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed()) {
            field_centric_drive = false;
        }

        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            odo.resetPosAndIMU();
            targetPos = null;
            holdingPosition = false;
            disturbed = false;
        }

        if (gamepad1.crossWasPressed() || gamepad2.crossWasPressed()) {
            launches = 0;
            reachedTarget = false;
        }

        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            launcher_target_velocity += 5;
        } else if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            launcher_target_velocity -= 5;
        }

        launcher_target_velocity = Range.clip(launcher_target_velocity,
                LAUNCHER_MIN_VELOCITY + 10, 3000);
    }

    // ==================================================================
    // Intake
    // ==================================================================
    public void handleIntake() {
        intakePower = Range.clip(
                (gamepad1.right_trigger + gamepad2.right_trigger)
                        - (gamepad1.left_trigger + gamepad2.left_trigger),
                -1.0, 1.0);

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);
    }

    // ==================================================================
    // Launcher
    // ==================================================================
    void launch() {
        if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
            launches++;
            lastLaunchMillis.reset();
            if (launches == 1) reachedTarget = false;
        }

        double actualVelocity = launcher.getVelocity();
        double minVelocity = launcher_target_velocity - LAUNCHER_MIN_VELOCITY;

        if (launches > 0) {
            launcher.setVelocity(launcher_target_velocity);

            if (actualVelocity >= minVelocity) {
                reachedTarget = true;
            } else if (reachedTarget && actualVelocity < minVelocity) {
                // Speed dropped after being up to speed -> a ball launched
                launches = Math.max(0, launches - 1);
                reachedTarget = false;
                lastLaunchMillis.reset();
            }
        } else {
            launcher.setVelocity(0);
            reachedTarget = false;
        }

        // Timeout: nothing launched within the limit
        if (lastLaunchMillis.milliseconds() > LAUNCHER_TIMEOUT && launches > 0) {
            launches = 0;
            reachedTarget = false;
        }

        boolean autoFeed = launches > 0 && actualVelocity >= minVelocity;
        boolean manualWindmill = gamepad1.left_bumper || gamepad2.left_bumper;

        double windmillPower = 0.0;
        if (autoFeed || manualWindmill) windmillPower = 1.0;

        if (gamepad1.dpad_left || gamepad2.dpad_left) windmillPower = -1.0;
        if (gamepad1.dpad_right || gamepad2.dpad_right) windmillPower = 1.0;

        windmillServo.setPower(windmillPower);

        // While auto-feeding, also run intake gently to dislodge stuck balls.
        // NOTE: set after handleIntake() so it adds on top of driver control.
        if (autoFeed) {
            setIntakePower(intakePower + 0.4);
        }
    }

    private void setIntakePower(double power) {
        power = Range.clip(power, -1.0, 1.0);
        intake.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    // ==================================================================
    // Telemetry
    // ==================================================================
    public void displayTelemetry() {
        Pose2D pos = odo.getPosition();

        telemetry.addData("Odometry Status", odo.getDeviceStatus());
        telemetry.addData("Drive Mode",
                field_centric_drive ? "Field Centric" : "Robot Centric");
        telemetry.addData("State", holdingPosition
                ? (disturbed ? "HOLDING (DISTURBED)" : "HOLDING (calm)")
                : "Manual Driving");

        if (pos != null) {
            telemetry.addData("Position", String.format(Locale.US,
                    "X: %.1f mm  Y: %.1f mm  H: %.1f deg",
                    pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM),
                    pos.getHeading(AngleUnit.DEGREES)));
        }

        if (targetPos != null && holdingPosition) {
            double xErr = DistanceUnit.INCH.fromMm(targetPos.getX(DistanceUnit.MM))
                    - getXPosition(DistanceUnit.INCH);
            double yErr = DistanceUnit.INCH.fromMm(targetPos.getY(DistanceUnit.MM))
                    - getYPosition(DistanceUnit.INCH);
            telemetry.addData("Hold Error", String.format(Locale.US,
                    "X: %.2f in  Y: %.2f in  H: %.1f deg",
                    xErr, yErr,
                    wrapDegrees(targetPos.getHeading(AngleUnit.DEGREES)
                            - getHeading(AngleUnit.DEGREES))));
        }

        telemetry.addData("Velocity", String.format(Locale.US,
                "X: %.1f in/s  Y: %.1f in/s  H: %.1f deg/s",
                velXips, velYips, velHeadingDps));

        telemetry.addData("Queued Launches", launches);
        telemetry.addData("Target Launch Speed", launcher_target_velocity);
        telemetry.addData("Actual Launcher Speed", launcher.getVelocity());

        telemetry.addData("Drive Powers", String.format(Locale.US,
                "LF: %.2f  RF: %.2f  LR: %.2f  RR: %.2f",
                leftFrontPower, rightFrontPower, leftBackPower, rightBackPower));

        telemetry.update();
        dBoardTelemetry.update();
    }
}
