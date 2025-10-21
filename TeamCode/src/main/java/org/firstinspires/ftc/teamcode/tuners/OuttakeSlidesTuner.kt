package org.firstinspires.ftc.teamcode.tuners

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.LINKAGE_IN
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.SLIDES_kD
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.SLIDES_kD_DOWN
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.SLIDES_kD_HANG
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.SLIDES_kP
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.SLIDES_kP_DOWN
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.SLIDES_kP_HANG
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.outtakePosition
import org.firstinspires.ftc.teamcode.util.MotorGroup.getCurrents
import org.firstinspires.ftc.teamcode.util.MotorGroup.getPowers
import org.firstinspires.ftc.teamcode.util.MotorGroup.resetEncoders
import org.firstinspires.ftc.teamcode.util.MotorGroup.setPower
import org.firstinspires.ftc.teamcode.util.TeleOpBase

@Config("Outtake Slides Tuner")
@TeleOp(name = "Outtake Slides Tuner", group = "Tuners")
class OuttakeSlidesTuner : TeleOpBase(DriveSubsystem, IntakeSubsystem) {
    companion object {
        @JvmField
        var TARGET = 0.0

        @JvmField
        var HANGING = false

        @JvmField
        var TOLERANCE = 300.0
    }

    private lateinit var linkage: Servo
    private lateinit var gearbox: Servo

    private lateinit var slidesMotors: List<MotorEx>
    private val slidesController = PIDFController(
        kP = SLIDES_kP,
        kD = SLIDES_kD,
        setPointTolerance = TOLERANCE
    )
    private var slidesLowering = false

    override fun initialize() {
        linkage = hardwareMap.get(Servo::class.java, "out_link")

        slidesMotors = arrayOf("1", "2", "3")
            .map { id -> MotorEx("lift${id}") }
        slidesMotors[0].direction = DcMotorSimple.Direction.FORWARD
        slidesMotors[1].direction = DcMotorSimple.Direction.REVERSE
        slidesMotors[2].direction = DcMotorSimple.Direction.REVERSE
        slidesMotors.resetEncoders()

        gearbox = OpModeData.hardwareMap.get(Servo::class.java, "gearbox")
    }

    override fun onStartButtonPressed() {
        IntakeSubsystem.initializeSystem()

        linkage.position = LINKAGE_IN
    }

    override fun update() {
        gearbox.position =
            if (HANGING) OuttakeSubsystem.GEARBOX_HANG else OuttakeSubsystem.GEARBOX_NORMAL

        if (HANGING) {
            slidesController.kP = SLIDES_kP_HANG
            slidesController.kD = SLIDES_kD_HANG
        } else {
            if (slidesLowering) {
                slidesController.kP = SLIDES_kP_DOWN
                slidesController.kD = SLIDES_kD_DOWN
            } else {
                slidesController.kP = SLIDES_kP
                slidesController.kD = SLIDES_kD
            }
        }

        slidesController.setPointTolerance = TOLERANCE

        slidesMotors.setPower(
            slidesController.calculate(
                slidesMotors[0].currentPosition.outtakePosition
            )
        )

        if (TARGET != slidesController.target) {
            slidesLowering = TARGET < slidesController.target

            slidesController.target = TARGET
            slidesController.reset()
        }

        telemetry.addData(
            "Current Slides State",
            if (HANGING) "Hanging"
            else if (slidesLowering) "Lowering"
            else "Normal"
        )

        telemetry.addData(
            "Outtake Slides Position",
            slidesMotors[0].currentPosition.outtakePosition
        )
        telemetry.addData("Outtake Slides Target", slidesController.target)
        telemetry.addLine()
        telemetry.addData(
            "Outtake Slides At Target",
            if (slidesController.atTarget(slidesMotors[0].currentPosition.outtakePosition)) "Yes" else "No"
        )
        telemetry.addLine()
        telemetry.addData(
            "Outtake Slides Currents",
            slidesMotors.getCurrents().joinToString { "%.2f".format(it) })
        telemetry.addLine()
        telemetry.addData(
            "Outtake Slides Coefficients",
            "kP: ${slidesController.kP} kD: ${slidesController.kD}"
        )
        telemetry.addLine()
        telemetry.addData("Outtake Slides Power", slidesMotors.getPowers().average())
        telemetry.addLine()
    }
}