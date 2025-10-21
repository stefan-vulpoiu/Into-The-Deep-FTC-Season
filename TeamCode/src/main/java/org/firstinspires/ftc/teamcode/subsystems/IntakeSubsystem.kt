package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.util.SampleSensor
import java.lang.Double.isNaN

@Config("Intake Constants")
object IntakeSubsystem : Subsystem() {
    @JvmField
    var LIFT_UP = .17

    @JvmField
    var LIFT_EXTENDING = .6

    @JvmField
    var LIFT_ATTACK = .72

    @JvmField
    var LIFT_DOWN = 0.82

    @JvmField
    var ARM_BEHIND = 0.0

    @JvmField
    var ARM_LEFT = .154

    @JvmField
    var ARM_CENTER = .37

    @JvmField
    var ARM_RIGHT = .56

    @JvmField
    var CLAW_OPENED = .55

    @JvmField
    var CLAW_TRANSFER = .17

    @JvmField
    var CLAW_CLOSED = .12

    @JvmField
    var EXTENDO_kP = 0.015

    @JvmField
    var EXTENDO_kD = 0.0002

    @JvmField
    var EXTENDO_IN = 0.0

    @JvmField
    var EXTENDO_TRANSFER = 450.0

    @JvmField
    var EXTENDO_OUT = 1300.0

    val Double.extendoPosition: Double
        get() {
            return (this / 8192.0 * 360)
        }

    private lateinit var extendoMotor: MotorEx

    private lateinit var diffLeft: Servo
    private lateinit var diffRight: Servo

    private lateinit var liftServo: Servo
    private lateinit var armServo: Servo

    private lateinit var clawServo: Servo

    lateinit var sampleSensor: SampleSensor

    var armState: ArmState? = null
    var liftState: LiftState? = null
    private var pivotState: PivotState? = null
    var rotationState: RotationState? = null
    var clawState: ClawState? = null
    var extendoState: ExtendoState? = null

    val extendoController = PIDFController(
        kP = EXTENDO_kP,
        kD = EXTENDO_kD,
        setPointTolerance = 10.0
    )

    private val stuckTime = ElapsedTime()

    override fun initialize() {
        extendoMotor = MotorEx("extendo")
        extendoMotor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        extendoMotor.motor.setCurrentAlert(4.5, CurrentUnit.AMPS)
        extendoMotor.direction = DcMotorSimple.Direction.FORWARD

        diffLeft = hardwareMap.get(Servo::class.java, "int_left")
        diffLeft.direction = Servo.Direction.REVERSE

        diffRight = hardwareMap.get(Servo::class.java, "int_right")
        diffRight.direction = Servo.Direction.FORWARD

        liftServo = hardwareMap.get(Servo::class.java, "int_lift")
        liftServo.direction = Servo.Direction.REVERSE

        armServo = hardwareMap.get(Servo::class.java, "int_rotate")

        clawServo = hardwareMap.get(Servo::class.java, "int_claw")

        sampleSensor = SampleSensor(hardwareMap)

        extendoController.target = 0.0
        extendoController.reset()

        stuckTime.reset()
    }

    override fun periodic() {
        extendoController.kP = EXTENDO_kP
        extendoController.kD = EXTENDO_kD

        extendoMotor.power =
            extendoController.calculate(extendoMotor.currentPosition.extendoPosition)

        telemetry.addData("Intake Extendo Position", extendoMotor.currentPosition.extendoPosition)
        telemetry.addData("Intake Extendo Target", extendoController.target)
        telemetry.addData(
            "Intake Extendo At Target",
            extendoController.atTarget(extendoMotor.currentPosition.extendoPosition)
        )
        telemetry.addLine()
    }

    val initializeSystem
        get() = ParallelGroup(
            setArmState(ArmState.CENTER),
            setLiftState(LiftState.ATTACK),
            setPivotState(PivotState.DOWN),
            setRotation(RotationState.HORIZONTAL),
            setClawState(ClawState.OPENED),
            setExtendoState(ExtendoState.IN),
        )

    // - Extendo Commands

    val resetExtendoEncoder
        get() = InstantCommand { extendoMotor.resetEncoder() }

    fun adjustExtendoPosition(by: Double) =
        InstantCommand {
            if (extendoController.target + by <= EXTENDO_OUT)
                setExtendoPosition(extendoController.target + by)()
        }

    fun setExtendoPosition(pos: Double) =
        LambdaCommand()
            .setStart {
                extendoController.target = pos
                extendoController.reset()

                stuckTime.reset()
            }
            .setIsDone {
                extendoController.atTarget(extendoMotor.currentPosition.extendoPosition) || stuckTime.seconds() > 1.35
            }

    fun setExtendoState(state: ExtendoState) =
        SequentialGroup(
            InstantCommand { extendoState = state },
            when (state) {
                ExtendoState.IN -> setExtendoPosition(EXTENDO_IN)
                ExtendoState.OUT -> setExtendoPosition(EXTENDO_OUT)
                ExtendoState.TRANSFER -> setExtendoPosition(EXTENDO_TRANSFER)
            }
        )

    val toggleExtendo
        get() = when (extendoState!!) {
            ExtendoState.IN -> setExtendoState(ExtendoState.OUT)
            ExtendoState.TRANSFER -> setExtendoState(ExtendoState.IN)
            ExtendoState.OUT -> setExtendoState(ExtendoState.IN)
        }

    // - Claw Commands

    fun setClawState(state: ClawState) =
        SequentialGroup(
            InstantCommand { clawState = state },
            when (state) {
                ClawState.CLOSED -> ServoToPosition(clawServo, CLAW_CLOSED, this)
                ClawState.TRANSFER -> ServoToPosition(clawServo, CLAW_TRANSFER, this)
                ClawState.OPENED -> ServoToPosition(clawServo, CLAW_OPENED, this)
            }
        )

    val toggleClaw
        get() = when (clawState!!) {
            ClawState.CLOSED -> setClawState(ClawState.OPENED)
            ClawState.TRANSFER -> setClawState(ClawState.OPENED)
            ClawState.OPENED -> setClawState(ClawState.CLOSED)
        }

    // - Lift Commands

    fun setLiftPosition(pos: Double) = ServoToPosition(liftServo, pos, this)

    fun setLiftState(state: LiftState) = SequentialGroup(
        InstantCommand { liftState = state },
        when (state) {
            LiftState.UP -> setLiftPosition(LIFT_UP)
            LiftState.DOWN -> setLiftPosition(LIFT_DOWN)
            LiftState.ATTACK -> setLiftPosition(LIFT_ATTACK)
            LiftState.EXTENDING -> setLiftPosition(LIFT_EXTENDING)
        }
    )
    // - Arm Commands

    fun setArmPosition(pos: Double) =
        ServoToPosition(armServo, if (isNaN(pos)) ARM_CENTER else pos, this)

    fun setArmState(state: ArmState) = SequentialGroup(
        InstantCommand { armState = state },
        when (state) {
            ArmState.BEHIND -> setArmPosition(ARM_BEHIND)
            ArmState.LEFT -> setArmPosition(ARM_LEFT)
            ArmState.CENTER -> setArmPosition(ARM_CENTER)
            ArmState.RIGHT -> setArmPosition(ARM_RIGHT)
        }
    )

    val previousArm
        get() = when (armState!!) {
            ArmState.BEHIND -> setArmState(ArmState.CENTER)
            ArmState.LEFT -> setArmState(ArmState.RIGHT)
            ArmState.CENTER -> setArmState(ArmState.LEFT)
            ArmState.RIGHT -> setArmState(ArmState.CENTER)
        }

    val nextArm
        get() = when (armState!!) {
            ArmState.BEHIND -> setArmState(ArmState.CENTER)
            ArmState.LEFT -> setArmState(ArmState.CENTER)
            ArmState.CENTER -> setArmState(ArmState.RIGHT)
            ArmState.RIGHT -> setArmState(ArmState.LEFT)
        }

    // - Differential Commands

    fun setPivotState(state: PivotState) = InstantCommand {
        pivotState = state
        Differential.update()
    }

    fun setRotation(state: RotationState) = InstantCommand {
        rotationState = state
        Differential.update()
    }

    val previousRotation
        get() = when (rotationState!!) {
            RotationState.LEFT -> setRotation(RotationState.VERTICAL)
            RotationState.RIGHT -> setRotation(RotationState.HORIZONTAL)
            RotationState.VERTICAL -> setRotation(RotationState.RIGHT)
            RotationState.HORIZONTAL -> setRotation(RotationState.LEFT)
        }

    val nextRotation
        get() = when (rotationState!!) {
            RotationState.LEFT -> setRotation(RotationState.HORIZONTAL)
            RotationState.RIGHT -> setRotation(RotationState.VERTICAL)
            RotationState.VERTICAL -> setRotation(RotationState.LEFT)
            RotationState.HORIZONTAL -> setRotation(RotationState.RIGHT)
        }

    enum class ArmState {
        BEHIND,
        LEFT,
        CENTER,
        RIGHT
    }

    enum class LiftState {
        UP,
        ATTACK,
        EXTENDING,
        DOWN,
    }

    enum class PivotState {
        UP,
        BEHIND,
        DOWN
    }

    enum class RotationState {
        LEFT,
        RIGHT,
        VERTICAL,
        HORIZONTAL
    }

    enum class ClawState {
        OPENED,
        TRANSFER,
        CLOSED
    }

    enum class ExtendoState {
        IN,
        TRANSFER,
        OUT
    }

    private object Differential {
        private data class Position(val leftPos: Double, val rightPos: Double)
        private data class State(val pivot: PivotState?, val rotation: RotationState?)

        private val positions = mapOf(
            State(PivotState.UP, RotationState.VERTICAL)
                    to Position(0.25, 0.21),

            State(PivotState.UP, RotationState.HORIZONTAL)
                    to Position(0.40, 0.36),

            State(PivotState.DOWN, RotationState.LEFT)
                    to Position(0.82, 0.63),
            State(PivotState.DOWN, RotationState.HORIZONTAL)
                    to Position(0.745, 0.705),
            State(PivotState.DOWN, RotationState.RIGHT)
                    to Position(0.67, 0.77),
            State(PivotState.DOWN, RotationState.VERTICAL)
                    to Position(0.60, 0.84),

            State(PivotState.BEHIND, RotationState.VERTICAL)
                    to Position(0.57, 0.88)
        )

        private fun get(pivot: PivotState?, rotation: RotationState?): Position {
            if (pivot == PivotState.BEHIND)
                return positions[State(pivot, RotationState.VERTICAL)]!!

            return positions[State(pivot, rotation)]
                ?: positions[State(
                    PivotState.DOWN,
                    RotationState.HORIZONTAL
                )]!!
        }

        fun update() {
            val positions = get(pivotState, rotationState)

            diffLeft.position = positions.leftPos
            diffRight.position = positions.rightPos
        }
    }
}