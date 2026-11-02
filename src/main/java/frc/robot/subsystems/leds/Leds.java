package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDStrip;
import frc.lib.util.flippable.Flippable;

import static frc.robot.utilities.PortsConstants.LEDSTRIP_PORT_PWM;

public class Leds extends SubsystemBase {
    private static final int FRONT_LENGTH = 42;
    private static final int BACK_LENGTH = 26;
    private static final int TOTAL_LENGTH = FRONT_LENGTH + BACK_LENGTH;

    private static final int[] RED_ALLIANCE_COLOURS_FLIPPED = {0x0000FF, 0xFF0000};//{0x8B0000, 0xFF0000, 0x8B0000, 0xFF8C00};
    private static final int[] RED_ALLIANCE_COLOURS = {0x8B0000, 0xFF0000, 0x8B0000, 0xFF8C00}; //for police: {0xFF0000, 0x0000FF};//
    private static final int[] BLUE_ALLIANCE_COLOURS = {0x87CEEB, 0x6495ED, 0x0000FF, 0xADD8E6, 0xC0C0C0};

    // Modes ordered lowest → highest priority. LAST active entry wins.
    public enum LEDMode {
        DEFAULT {
            @Override
            public void apply(Leds leds) {
                int[] c = Flippable.isRedAlliance() ? RED_ALLIANCE_COLOURS : BLUE_ALLIANCE_COLOURS;
//                leds.front.flashing(RED_ALLIANCE_COLOURS);
//                leds.back.flashing(RED_ALLIANCE_COLOURS_FLIPPED); //POLICIA!
                leds.front.interpolated(c);
                leds.back.interpolated(c);
            }
        },
        AUTO_START {
            @Override
            public void apply(Leds leds) {
                leds.front.interpolated(0xFFFFFF, 0x00FFFF);
                leds.back.interpolated(0xFFFFFF, 0x00FFFF);
            }
        },

        // ── Critical (always visible) ─────────────────────────────────────
        BATTERY_LOW {
            @Override
            public void apply(Leds leds) {
                leds.front.pulse(0xFF00FF, 10);
                leds.back.pulse(0xFF00FF, 10);
            }
        };

        public abstract void apply(Leds leds);
    }

    // Cached to avoid per-periodic array allocation from LEDMode.values().
    private static final LEDMode[] MODES = LEDMode.values();

    private final AddressableLED ledstrip = new AddressableLED(LEDSTRIP_PORT_PWM);
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(TOTAL_LENGTH);

    final LEDStrip front = new LEDStrip(FRONT_LENGTH);
    final LEDStrip back = new LEDStrip(BACK_LENGTH);

    // boolean[] indexed by ordinal — avoids EnumMap boxing/unboxing overhead.
    private final boolean[] activeRequests = new boolean[MODES.length];

    public Leds() {
        ledstrip.setLength(TOTAL_LENGTH);
        ledstrip.setData(ledBuffer);
        ledstrip.start();
    }

    @Override
    public void periodic() {
        LEDMode toDisplay = LEDMode.DEFAULT;

        for (int i = MODES.length - 1; i >= 0; i--) {
            if (activeRequests[i]) {
                toDisplay = MODES[i];
                break;
            }
        }

        toDisplay.apply(this);

        front.writeToBuffer(ledBuffer, 0);
        back.writeToBuffer(ledBuffer, FRONT_LENGTH);

        ledstrip.setData(ledBuffer);
    }

    public Command showFor(LEDMode mode, double durationSeconds) {
        return show(mode).withTimeout(durationSeconds);
    }

    public Command show(LEDMode mode) {
        int ord = mode.ordinal();
        return Commands.startEnd(
                () -> activeRequests[ord] = true,
                () -> activeRequests[ord] = false,
                this
        );
    }
}