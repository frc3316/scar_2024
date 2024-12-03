// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.JoysticksConstants;
import frc.robot.subsystems.Arm;
import frc.robot.humanIO.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Arm m_arm = new Arm();
    private final CommandPS5Controller m_Controller = new CommandPS5Controller(JoysticksConstants.driverPort);
  public RobotContainer() {
    
  }


  private void configureBindings() {
    m_Controller.square().onTrue(new InstantCommand(() -> m_arm.drawSquare()));
    m_Controller.circle().onTrue(new InstantCommand(() -> m_arm.drawCircle(100)));
    m_Controller.triangle().onTrue(new InstantCommand(() -> m_arm.getCompetativeDrow(100)));
    m_Controller.cross().onTrue(new InstantCommand(()-> m_arm.resetPosotion()));
  }



  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }*/
}
