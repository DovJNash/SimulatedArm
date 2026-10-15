package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Questionnaire {
    private enum SecondCycle {
        YES(),
        NO();

        SecondCycle() {
        }

        public boolean getValue() {
            return this == YES;
        }
    }

    public Questionnaire() {
    }

    public Command getCommand() {
        return Commands.none();
    }

    public String getSelected() {
        return "Placeholder";
    }

    private <T extends Enum<T>> LoggedDashboardChooser<T> createQuestion(String questionName, Class<T> enumClass) {
        final LoggedDashboardChooser<T> question = new LoggedDashboardChooser<>(questionName);

        for (final T option : enumClass.getEnumConstants()) {
            question.addOption(option.name(), option);
        }

        if (enumClass.getEnumConstants().length > 0) {
            question.addDefaultOption(enumClass.getEnumConstants()[0].name(), enumClass.getEnumConstants()[0]);
        }

        return question;
    }
}