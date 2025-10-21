package org.firstinspires.ftc.teamcode.teleop.solo

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.CommandManager
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.util.SampleSensor
import org.firstinspires.ftc.teamcode.util.TeleOpBase
import org.firstinspires.ftc.teamcode.util.UtilityCommands.enterCollect
import org.firstinspires.ftc.teamcode.util.UtilityCommands.sampleTransfer
import org.firstinspires.ftc.teamcode.util.UtilityCommands.specimenTransfer

@Config
@TeleOp(name = "Petrica Solo", group = "TeleOp Solo")
class PetricaSolo : TeleOpBase(DriveSubsystem, IntakeSubsystem, OuttakeSubsystem) {
    companion object {
        @JvmField
        var SAMPLE_INIT = true

        @JvmField
        var RED_INIT = true
    }

    enum class State {
        COLLECTING,
        COLLECTED,
        DEPOSITING,
    }

    private val toggleMode
        get() = InstantCommand {
            isSampleMode = !isSampleMode
        }

    private val toggleAlliance
        get() = InstantCommand {
            isRed = !isRed
        }

    private var isSampleMode = SAMPLE_INIT
    private var isTransferring = false
    private var opState: State = State.COLLECTING

    private var isRed = RED_INIT

    private val actionCommand: Command
        get() = when (opState) {
            State.COLLECTING ->
                if (IntakeSubsystem.clawState == IntakeSubsystem.ClawState.OPENED)
                    if (IntakeSubsystem.liftState == IntakeSubsystem.LiftState.ATTACK)
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN)
                    else SequentialGroup(
                        IntakeSubsystem.toggleClaw,
                        Delay(.125),
                        IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
                        InstantCommand { opState = State.COLLECTED },
                        BlockingConditionalCommand(
                            { IntakeSubsystem.sampleSensor.sampleType != SampleSensor.SampleType.Yellow },
                            { cancelCommand }
                        )
                    )
                else IntakeSubsystem.toggleClaw

            State.COLLECTED ->
                if (!isTransferring)
                    SequentialGroup(
                        InstantCommand { isTransferring = true },
                        if (isSampleMode) sampleTransfer
                        else specimenTransfer,
                        InstantCommand { isTransferring = false },
                        InstantCommand { opState = State.DEPOSITING }
                    )
                else NullCommand()

            State.DEPOSITING ->
                if (isSampleMode)
                    if (OuttakeSubsystem.clawState == OuttakeSubsystem.ClawState.CLOSED)
                        OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED)
                    else
                        SequentialGroup(
                            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
                            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
                            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
                            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                                .withDeadline(Delay(.1)),

                            DriveSubsystem.setMaxSpeed(1.0),

                            InstantCommand { opState = State.COLLECTING }
                        )
                else
                    SequentialGroup(
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
                        Delay(.150),
                        enterCollect,
                        InstantCommand { opState = State.COLLECTING }
                    )
        }


    private val cancelCommand: Command
        get() = when (opState) {
            State.COLLECTING ->
                if (IntakeSubsystem.liftState == IntakeSubsystem.LiftState.DOWN)
                    IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK)
                else NullCommand()

            State.COLLECTED ->
                if (isTransferring)
                    SequentialGroup(
                        InstantCommand {
                            if (isSampleMode) {
                                CommandManager.cancelCommand(sampleTransfer)
                            } else {
                                CommandManager.cancelCommand(specimenTransfer)
                            }
                        },
                        enterCollect,
                        IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
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
                if (isSampleMode)
                    SequentialGroup(
                        OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
                        Delay(.25),
                        actionCommand
                    )
                else actionCommand
        }

    override fun initialize() {
        gamepad1.setLedColor(64.0, 224.0, 208.0, -1)

        driver1.rightTrigger.pressedCommand = {
            SequentialGroup(
                IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.EXTENDING),
                Delay(.1),
                if (IntakeSubsystem.extendoState == IntakeSubsystem.ExtendoState.OUT)
                    IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN)
                        .then(DriveSubsystem.setMaxSpeed(1.0))
                else
                    IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.OUT)
                        .then(DriveSubsystem.setMaxSpeed(0.7)),
                IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK)
            )
        }

        driver1.leftBumper.pressedCommand = {
            when (opState) {
                State.COLLECTING -> IntakeSubsystem.previousArm
                State.DEPOSITING -> OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.LOW_BASKET)
                    .and(DriveSubsystem.setMaxSpeed(0.4))

                State.COLLECTED -> NullCommand()
            }
        }
        driver1.rightBumper.pressedCommand = {
            when (opState) {
                State.COLLECTING -> IntakeSubsystem.nextArm
                State.DEPOSITING -> OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                    .and(DriveSubsystem.setMaxSpeed(0.4))

                State.COLLECTED -> NullCommand()
            }
        }

        driver1.dpadUp.pressedCommand = {
            OuttakeSubsystem.adjustSlides(100.0)
        }

        driver1.dpadDown.pressedCommand = {
            OuttakeSubsystem.adjustSlides(-100.0)
        }

        driver1.leftStick.button.pressedCommand = { IntakeSubsystem.previousRotation }
        driver1.rightStick.button.pressedCommand = { IntakeSubsystem.nextRotation }

        driver1.cross.pressedCommand = { actionCommand }
        driver1.circle.pressedCommand = { cancelCommand }

        driver1.guide.pressedCommand = { toggleAlliance }

        driver1.back.pressedCommand = {
            SequentialGroup(
                IntakeSubsystem.resetExtendoEncoder,
                OuttakeSubsystem.resetSlidesEncoder,
            )
        }
    }

    override fun onStartButtonPressed() {
        IntakeSubsystem.initializeSystem()
        OuttakeSubsystem.initializeSystem()
    }

    override fun update() {
        telemetry.addData("Mode", if (isSampleMode) "Sample" else "Specimen")
        telemetry.addData("Operation State", opState)
        telemetry.addData("Is Transferring", isTransferring)
    }
}