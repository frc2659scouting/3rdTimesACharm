package frc.lib;

public final class SdsModuleConfigurations {
    public static final ModuleConfiguration MK3_STANDARD = new ModuleConfiguration(
            0.1016,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (48/16*6), 
            true
    );
    public static final ModuleConfiguration MK3_FAST = new ModuleConfiguration(
            0.1016,
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 60.0),
            true,
            (48/16*6), 
            true
    );

    public static final ModuleConfiguration MK4_L1 = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            true,
            (48/16*6), 
            true
    );
    public static final ModuleConfiguration MK4_L2 = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (48/16*6), 
            true
    );
    public static final ModuleConfiguration MK4_L3 = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0),
            true,
            (48/16*6), 
            true
    );
    public static final ModuleConfiguration MK4_L4 = new ModuleConfiguration(
            0.1016, //wheel diameter
            (16.0 / 48.0) * (28.0 / 16.0) * (15.0 / 45.0), //drive
            true,
            1.0 / (48.0 / 16.0 * 6.0), // FIX ME //turn
            false
    );

    private SdsModuleConfigurations() {
    }
}
