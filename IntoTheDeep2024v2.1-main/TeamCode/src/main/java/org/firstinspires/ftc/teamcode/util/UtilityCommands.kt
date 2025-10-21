package org.firstinspires.ftc.teamcode.util

import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

object UtilityCommands {
    val enterCollect
        get() = SequentialGroup(
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
        )

    val enterCollectOuttake
        get() = SequentialGroup(
            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN)
                .endAfter(0.0),
        )

    val collectSample
        get() = SequentialGroup(
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.DOWN),
            Delay(0.15),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),
            Delay(0.15),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK)
        )

    val enterSpecimenCollect
        get() = SequentialGroup(
            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.SPECIMEN),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN)
        )

    val sampleTransfer
        get() = SequentialGroup(
            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.TRANSFER),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),

            if (IntakeSubsystem.extendoState == IntakeSubsystem.ExtendoState.IN)
                IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.TRANSFER)
                    .endAfter(0.0).thenWait(0.1)
            else NullCommand(),

            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.TRANSFER),

            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.UP),
            Delay(0.12),

            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.UP),
            Delay(0.15),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.CLOSED),

            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
            Delay(0.05),

            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.CLOSED),
            Delay(0.1),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
            Delay(0.075),

            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)
                .and(
                    SequentialGroup(
                        Delay(.1),
                        OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.BASKET),
                        Delay(.05),
                        OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.OUT),
                    )
                ),

            enterCollect,
        )

    val specimenTransfer
        get() = SequentialGroup(
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.VERTICAL),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.BEHIND),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN),
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.BEHIND),
        )

    val resetIntake
        get() = SequentialGroup(
            IntakeSubsystem.setArmState(IntakeSubsystem.ArmState.CENTER),
            IntakeSubsystem.setLiftState(IntakeSubsystem.LiftState.ATTACK),
            IntakeSubsystem.setPivotState(IntakeSubsystem.PivotState.DOWN),
            IntakeSubsystem.setRotation(IntakeSubsystem.RotationState.HORIZONTAL),
            IntakeSubsystem.setClawState(IntakeSubsystem.ClawState.OPENED),
            IntakeSubsystem.setExtendoState(IntakeSubsystem.ExtendoState.IN)
        )

    val resetOuttake
        get() = SequentialGroup(
            OuttakeSubsystem.setGearboxState(OuttakeSubsystem.GearboxState.NORMAL),
            OuttakeSubsystem.setArmState(OuttakeSubsystem.ArmState.STRAIGHT),
            OuttakeSubsystem.setLinkageState(OuttakeSubsystem.LinkageState.IN),
            OuttakeSubsystem.setSlidesState(OuttakeSubsystem.SlidesState.DOWN),
            OuttakeSubsystem.setClawState(OuttakeSubsystem.ClawState.OPENED)
        )
}