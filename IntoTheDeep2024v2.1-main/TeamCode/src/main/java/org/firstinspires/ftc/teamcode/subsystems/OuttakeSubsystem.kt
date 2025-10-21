package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.OpModeData.hardwareMap
import com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import org.firstinspires.ftc.teamcode.util.InterpolatedServo
import org.firstinspires.ftc.teamcode.util.MotorGroup.getPowers
import org.firstinspires.ftc.teamcode.util.MotorGroup.resetEncoders
import org.firstinspires.ftc.teamcode.util.MotorGroup.setPower

@Config("Outtake Constants")
object OuttakeSubsystem : Subsystem() {
    @JvmField
    var CLAW_OPENED = .5

    @JvmField
    var CLAW_CLOSED = .18

    @JvmField
    var ARM_STRAIGHT = 180.0

    @JvmField
    var ARM_TRANSFER = 171.0

    @JvmField
    var ARM_BASKET = 22.0

    @JvmField
    var ARM_SPECIMEN = -35.0

    @JvmField
    var LINKAGE_IN = .62

    @JvmField
    var LINKAGE_OUT = .2

    @JvmField
    var GEARBOX_NORMAL = .95

    @JvmField
    var GEARBOX_HANG = .28

    @JvmField
    var SLIDES_DOWN = 0.0

    @JvmField
    var SLIDES_HANG_2 = 85.0

    @JvmField
    var SLIDES_SPECIMEN = 450.0

    @JvmField
    var SLIDES_LOW_BASKET = 900.0

    @JvmField
    var SLIDES_HIGH_BASKET = 2900.0

    @JvmField
    var SLIDES_HANG_PREPARE = 2000.0

    @JvmField
    var SLIDES_kP = 0.004

    @JvmField
    var SLIDES_kP_DOWN = 0.00045

    @JvmField
    var SLIDES_kP_HANG = 0.015

    @JvmField
    var SLIDES_kD = 0.00009

    @JvmField
    var SLIDES_kD_DOWN = 0.00002

    @JvmField
    var SLIDES_kD_HANG = 0.00015

    val Double.outtakePosition: Double
        get() {
            return (this / 8192.0 * 360).coerceAtLeast(0.0)
        }

    private lateinit var clawServo: Servo

    private lateinit var armLeft: InterpolatedServo
    private lateinit var armRight: InterpolatedServo

    private lateinit var linkageServo: Servo

    private lateinit var slidesMotors: List<MotorEx>
    private val slidesController = PIDFController(
        kP = SLIDES_kP,
        kD = SLIDES_kD,
        setPointTolerance = 100.0
    )
    private var slidesLowering = false
    private var hanging = false

    private lateinit var gearbox: Servo

    var clawState: ClawState? = null
    private var armState: ArmState? = null
    private var linkageState: LinkageState? = null
    private var gearboxState: GearboxState? = null
    var slidesState: SlidesState? = null

    val armLeftPositions: Array<Pair<Double, Double>> = arrayOf(
        0.0 to 0.187,
        90.0 to 0.465,
        180.0 to 0.735,
        220.0 to 0.87
    )

    val armRightPositions: Array<Pair<Double, Double>> = arrayOf(
        0.0 to 0.187,
        90.0 to 0.465,
        180.0 to 0.735,
        220.0 to 0.87,
    )

    override fun initialize() {
        clawServo = hardwareMap.get(Servo::class.java, "out_claw")

        armLeft = InterpolatedServo(hardwareMap, "arm_left")
        armLeft.setInverted(false)
        armLeft.generatePositions(*armLeftPositions)

        armRight = InterpolatedServo(hardwareMap, "arm_right")
        armRight.setInverted(true)
        armRight.generatePositions(*armRightPositions)

        linkageServo = hardwareMap.get(Servo::class.java, "out_link")

        slidesMotors = arrayOf("1", "2", "3")
            .map { id -> MotorEx("lift${id}") }
        slidesMotors[0].direction = DcMotorSimple.Direction.FORWARD
        slidesMotors[1].direction = DcMotorSimple.Direction.REVERSE
        slidesMotors[2].direction = DcMotorSimple.Direction.REVERSE

        gearbox = hardwareMap.get(Servo::class.java, "gearbox")

        slidesController.target = 0.0
        slidesController.reset()
    }

    override fun periodic() {
        if (hanging) {
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

        slidesMotors.setPower(slidesController.calculate(slidesMotors[0].currentPosition.outtakePosition))

        telemetry.addData(
            "Current Slides State",
            if (hanging) "Hanging"
            else if (slidesLowering) "Lowering"
            else "Normal"
        )
        telemetry.addLine()
        telemetry.addData(
            "Outtake Slides Position",
            slidesMotors[0].currentPosition.outtakePosition
        )
        telemetry.addData("Outtake Slides Target", slidesController.target)
        telemetry.addData(
            "Outtake Slides At Target",
            if (slidesController.atTarget(slidesMotors[0].currentPosition.outtakePosition)) "Yes" else "No"
        )
        telemetry.addData(
            "Outtake Slides Coefficients",
            "kP: ${slidesController.kP} kD: ${slidesController.kD}"
        )
        telemetry.addLine()
        telemetry.addData("Outtake Slides Power", slidesMotors.getPowers().average())
        telemetry.addLine()
    }

    val initializeSystem
        get() = ParallelGroup(
            setClawState(ClawState.OPENED),
            setArmState(ArmState.STRAIGHT),
            setLinkageState(LinkageState.IN),
            setGearboxState(GearboxState.NORMAL),
            setSlidesState(SlidesState.DOWN),
        )

    // - Slides Commands

    val resetSlidesEncoder
        get() = InstantCommand { slidesMotors.resetEncoders() }

    fun adjustSlides(by: Double) =
        InstantCommand {
            if (slidesController.target + by <= 3000)
                setSlidesPosition(slidesController.target + by)()
        }

    fun setSlidesPosition(pos: Double) =
        LambdaCommand()
            .setStart {
                slidesLowering = pos < slidesController.target

                slidesController.target = pos
                slidesController.reset()
            }
            .setIsDone { slidesController.atTarget(slidesMotors[0].currentPosition.outtakePosition) }

    fun setSlidesState(state: SlidesState) =
        SequentialGroup(
            InstantCommand { slidesState = state },
            when (state) {
                SlidesState.DOWN -> setSlidesPosition(SLIDES_DOWN)
                SlidesState.HANG_2 -> setSlidesPosition(SLIDES_HANG_2)
                SlidesState.SPECIMEN -> setSlidesPosition(SLIDES_SPECIMEN)
                SlidesState.LOW_BASKET -> setSlidesPosition(SLIDES_LOW_BASKET)
                SlidesState.HIGH_BASKET -> setSlidesPosition(SLIDES_HIGH_BASKET)
                SlidesState.HANG_PREPARE -> setSlidesPosition(SLIDES_HANG_PREPARE)
            }
        )

    // - Arm Commands

    fun setArmPosition(pos: Double) = InstantCommand {
        armLeft.position = pos
        armRight.position = pos
    }

    fun setArmState(state: ArmState) = SequentialGroup(
        InstantCommand { armState = state },
        when (state) {
            ArmState.SPECIMEN -> setArmPosition(ARM_SPECIMEN)
            ArmState.BASKET -> setArmPosition(ARM_BASKET)
            ArmState.TRANSFER -> setArmPosition(ARM_TRANSFER)
            ArmState.STRAIGHT -> setArmPosition(ARM_STRAIGHT)
        }
    )

    // - Linkage Commands

    fun setLinkageState(state: LinkageState) = when (state) {
        LinkageState.IN -> ServoToPosition(linkageServo, LINKAGE_IN, this)
        LinkageState.OUT -> ServoToPosition(linkageServo, LINKAGE_OUT, this)
    }

    private val toggleLinkage
        get() = when (linkageState!!) {
            LinkageState.IN -> setLinkageState(LinkageState.OUT)
            LinkageState.OUT -> setLinkageState(LinkageState.IN)
        }

    // - Gearbox Commands

    fun setGearboxState(state: GearboxState) =
        SequentialGroup(
            InstantCommand {
                gearboxState = state
                hanging = state == GearboxState.HANG
            },
            when (state) {
                GearboxState.NORMAL -> ServoToPosition(gearbox, GEARBOX_NORMAL, this)
                GearboxState.HANG -> ServoToPosition(gearbox, GEARBOX_HANG, this)
            }
        )

    private val toggleGearbox
        get() = when (gearboxState!!) {
            GearboxState.NORMAL -> setGearboxState(GearboxState.HANG)
            GearboxState.HANG -> setGearboxState(GearboxState.NORMAL)
        }

    // - Claw State

    fun setClawState(state: ClawState) =
        SequentialGroup(
            InstantCommand { clawState = state },
            when (state) {
                ClawState.CLOSED -> ServoToPosition(clawServo, CLAW_CLOSED, this)
                ClawState.OPENED -> ServoToPosition(clawServo, CLAW_OPENED, this)
            }
        )

    private val toggleClaw
        get() = when (clawState!!) {
            ClawState.CLOSED -> setClawState(ClawState.OPENED)
            ClawState.OPENED -> setClawState(ClawState.CLOSED)
        }

    enum class SlidesState {
        DOWN,
        HANG_2,
        SPECIMEN,
        LOW_BASKET,
        HIGH_BASKET,
        HANG_PREPARE
    }

    enum class ArmState {
        SPECIMEN,
        BASKET,
        TRANSFER,
        STRAIGHT,
    }

    enum class LinkageState {
        IN,
        OUT
    }

    enum class GearboxState {
        NORMAL,
        HANG
    }

    enum class ClawState {
        OPENED,
        CLOSED
    }
}