package org.firstinspires.ftc.teamcode.teleop.duo

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.CommandManager
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.TeleOpBase
import org.firstinspires.ftc.teamcode.util.UtilityCommands.enterCollect
import org.firstinspires.ftc.teamcode.util.UtilityCommands.enterCollectOuttake
import org.firstinspires.ftc.teamcode.util.commands.ConditionalCommand

@Config
@TeleOp(name = "Luca + Miron Duo", group = "TeleOp Duo")
class LucaMironDuo : TeleOpBase(DriveSubsystem, IntakeSubsystem, OuttakeSubsystem) {
    private var transferCommand: Command? = null

    private var prevRotation: IntakeSubsystem.RotationState? = null
    private var prevExtendo: IntakeSubsystem.ExtendoState? = null
    private var prevArm: IntakeSubsystem.ArmState? = null

    private var opState: State = State.COLLECTING

    private var sensorEnabled = true

    enum class State {
        COLLECTING,
        COLLECTED,
        DEPOSITING,
    }

    private val toggleSensor
        get() = InstantCommand {
            sensorEnabled = !sensorEnabled
        }

    private val transfer
        get() = SequentialGroup(
            InstantCommand {
                prevRotation = IntakeSubsystem.rotationState
                prevArm = IntakeSubsystem.armState
                prevExtendo = IntakeSubsystem.extendoState
            },

            DriveSubsystem.setMaxSpeed(1.0),

            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),

            if (IntakeSubsystem.extendoState == IntakeSubsystem.ExtendoState.IN)
                IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.TRANSFER)
                    .endAfter(0.3)
            else NullCommand(),

            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),

            Delay(0.05),

            ConditionalCommand(
                { !sensorEnabled || IntakeSubsystem.sampleSensor.getDistance(DistanceUnit.CM) <= 1.8 },
                {
                    SequentialGroup(
                        IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),

                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

                        IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
                        Delay(0.14),

                        OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
                        Delay(0.05),
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        Delay(0.075),

                        OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                            .and(
                                SequentialGroup(
                                    Delay(.15),
                                    OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                                    Delay(.1),
                                    OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                                )
                            ),

                        InstantCommand { opState = State.DEPOSITING },
                        InstantCommand { transferCommand = null },

                        enterCollect,
                    )
                },
                {
                    SequentialGroup(
                        enterCollectOuttake,

                        IntakeSubsystem.setExtendoState(
                            prevExtendo ?: IntakeSubsystem.ExtendoState.IN
                        )
                            .endAfter(0.0),
                        IntakeSubsystem.setArmState(prevArm ?: IntakeSubsystem.ArmState.CENTER),
                        IntakeSubsystem.setRotation(
                            prevRotation ?: IntakeSubsystem.RotationState.HORIZONTAL
                        ),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),

                        InstantCommand { opState = State.COLLECTING },
                        InstantCommand { transferCommand = null },
                    )
                }
            ),
        )

    private val actionCommand
        get() = when (opState) {
            State.COLLECTING ->
                if (IntakeSubsystem.clawState == IntakeSubsystem.ClawState.OPENED)
                    if (IntakeSubsystem.liftState == IntakeSubsystem.LiftState.ATTACK)
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN)
                    else
                        SequentialGroup(
                            IntakeSubsystem.toggleClaw,
                            Delay(.125),
                            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                            InstantCommand { opState = State.COLLECTED },
                            run {
                                transferCommand = transfer
                                transferCommand!!
                            },
                        )
                else IntakeSubsystem.toggleClaw

            State.DEPOSITING ->
                if (OuttakeSubsystem.clawState == OuttakeSubsystem.ClawState.CLOSED)
                    OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED)
                else
                    SequentialGroup(
                        enterCollectOuttake.endAfter(0),

                        DriveSubsystem.setMaxSpeed(1.0),

                        InstantCommand { opState = State.COLLECTING }
                    )

            State.COLLECTED -> NullCommand()
        }

    private val cancelCommand
        get() = when (opState) {
            State.COLLECTING ->
                if (IntakeSubsystem.liftState == IntakeSubsystem.LiftState.DOWN)
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK)
                else NullCommand()

            State.COLLECTED ->
                if (transferCommand != null)
                    SequentialGroup(
                        InstantCommand {
                            CommandManager.cancelCommand(transferCommand!!)
                            transferCommand = null
                        },

                        IntakeSubsystem.setExtendoState(
                            prevExtendo ?: IntakeSubsystem.ExtendoState.IN
                        ).endAfter(0.0),
                        IntakeSubsystem.setArmState(prevArm ?: IntakeSubsystem.ArmState.CENTER),
                        IntakeSubsystem.setRotation(
                            prevRotation ?: IntakeSubsystem.RotationState.HORIZONTAL
                        ),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),

                        enterCollectOuttake,

                        InstantCommand { opState = State.COLLECTING }
                    )
                else
                    if (IntakeSubsystem.clawState == IntakeSubsystem.ClawState.CLOSED)
                        SequentialGroup(
                            IntakeSubsystem.toggleClaw,
                            InstantCommand { opState = State.COLLECTING }
                        )
                    else NullCommand()

            State.DEPOSITING ->
                SequentialGroup(
                    OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
                    Delay(.25),
                    actionCommand
                )
        }

    private val pickCommand
        get() =
            if (IntakeSubsystem.clawState == IntakeSubsystem.ClawState.OPENED)
                if (IntakeSubsystem.liftState == IntakeSubsystem.LiftState.EXTENDING
                    || IntakeSubsystem.liftState == IntakeSubsystem.LiftState.ATTACK
                )
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN)
                else
                    SequentialGroup(
                        IntakeSubsystem.toggleClaw,
                        Delay(.125),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                    )
            else SequentialGroup(
                IntakeSubsystem.toggleClaw,
                Delay(0.1),
                IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK)
            )

    private var hang2 = false
    private var hang3 = false
    private val hangCommand
        get() = if (!hang2)
            if (OuttakeSubsystem.slidesState == OuttakeSubsystem.SlidesState.HANG_PREPARE)
                SequentialGroup(
                    OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HANG_2),
                    InstantCommand { hang2 = true }
                )
            else SequentialGroup(
                OuttakeSubsystem.setGearboxState(OuttakeSubsystem.GearboxState.HANG),
                Delay(0.6),
                OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HANG_PREPARE),
            )
        else
            if (!hang3)
                SequentialGroup(
                    OuttakeSubsystem.setSlidesPosition(2600.0),

                    IntakeSubsystem.setLiftPosition(0.1),
                    IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.VERTICAL),
                    IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                    IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                    OuttakeSubsystem.setArmPosition(0.0),

                    InstantCommand { hang3 = true }
                )
            else
                OuttakeSubsystem.setSlidesPosition(-75.0)


    override fun initialize() {
        driver1.leftBumper.pressedCommand = {
            if (OuttakeSubsystem.slidesState == OuttakeSubsystem.SlidesState.HIGH_BASKET)
                DriveSubsystem.setMaxSpeed(0.3)
            else DriveSubsystem.setMaxSpeed(0.4)
        }
        driver1.leftBumper.releasedCommand = { DriveSubsystem.setMaxSpeed(1.0) }

        driver1.rightTrigger.pressedCommand = {
            if (transferCommand == null)
                SequentialGroup(
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                    Delay(.1),
                    IntakeSubsystem.toggleExtendo,
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                )
            else NullCommand()
        }

        driver1.square.pressedCommand = {
            if (transferCommand == null) IntakeSubsystem.previousArm else NullCommand()
        }
        driver1.circle.pressedCommand = {
            if (transferCommand == null) IntakeSubsystem.nextArm else NullCommand()
        }

        driver1.triangle.pressedCommand = {
            if (transferCommand == null) hangCommand else NullCommand()
        }

        driver1.rightBumper.pressedCommand = {
            if (transferCommand == null && hang3) SequentialGroup(
                IntakeSubsystem.setLiftPosition(1.0),
                IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
                IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),
                IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                OuttakeSubsystem.setArmPosition(180.0),

                Delay(0.5),

                IntakeSubsystem.setLiftPosition(0.1),
                IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.VERTICAL),
                IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
                IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
                OuttakeSubsystem.setArmPosition(0.0),
            ) else NullCommand()
        }

        driver1.dpadLeft.pressedCommand = {
            if (transferCommand == null) OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.LOW_BASKET) else NullCommand()
        }

        driver1.dpadUp.pressedCommand = { IntakeSubsystem.adjustExtendoPosition(50.0) }
        driver1.dpadDown.pressedCommand = { IntakeSubsystem.adjustExtendoPosition(-50.0) }

        driver2.dpadUp.pressedCommand = { OuttakeSubsystem.adjustSlides(15.0) }
        driver2.dpadDown.pressedCommand = { OuttakeSubsystem.adjustSlides(-15.0) }

        driver2.leftStick.button.pressedCommand = {
            if (transferCommand == null) IntakeSubsystem.previousRotation else NullCommand()
        }
        driver2.rightStick.button.pressedCommand = {
            if (transferCommand == null) IntakeSubsystem.nextRotation else NullCommand()
        }

        driver2.cross.pressedCommand =
            { if (transferCommand == null) actionCommand else NullCommand() }
        driver2.circle.pressedCommand = { cancelCommand }

        driver2.square.pressedCommand = { pickCommand }

        driver2.guide.pressedCommand = { toggleSensor }

        driver2.back.pressedCommand = { OuttakeSubsystem.resetSlidesEncoder }

        driver1.back.pressedCommand = { IntakeSubsystem.resetExtendoEncoder }
    }

    override fun onStartButtonPressed() {
        IntakeSubsystem.initializeSystem()
        OuttakeSubsystem.initializeSystem()
    }

    override fun update() {
        telemetry.addData("State", opState)
        telemetry.addData("Sensor Enabled", sensorEnabled)
        telemetry.addData("Is Transferring", transferCommand != null)
    }
}