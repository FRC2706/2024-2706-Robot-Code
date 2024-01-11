package frc.lib.lib2706;

import java.util.function.Consumer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.PubSubOption;


public class UpdateSimpleFeedforward {
    private final double UPDATE_RATE_MS = 200; // in ms
    private Consumer<SimpleMotorFeedforward> m_setFeedforward;
    private DoubleEntry entryKS, entryKV, entryKA;
    private double prevKS, prevKV, prevKA;

    /**
     * Create a UpdateFeedforward to update a {@link SimpleMotorFeedforward} object from networktables.
     * 
     * @param setFeedforward A consumer to recieve an updated SimpleMotorFeedforward. Recommend: {@snippet (ff) -> m_topFF = ff}
     * @param tableName The table name to put the 3 entries in for kS, kV and kA
     * @param kS The default value for kS
     * @param kV The default value for kV
     * @param kA The default value for kA
     */
    public UpdateSimpleFeedforward(Consumer<SimpleMotorFeedforward> setFeedforward, NetworkTable table, double kS, double kV, double kA) {
        m_setFeedforward = setFeedforward;
        prevKS = kS;
        prevKV = kV;
        prevKA = kA;
        
        NetworkTable ffTable = table.getSubTable("TuneSimpleFF");
        entryKS = ffTable.getDoubleTopic("kS").getEntry(kS, PubSubOption.periodic(UPDATE_RATE_MS));
        entryKV = ffTable.getDoubleTopic("kV").getEntry(kV, PubSubOption.periodic(UPDATE_RATE_MS));
        entryKA = ffTable.getDoubleTopic("kA").getEntry(kA, PubSubOption.periodic(UPDATE_RATE_MS));

        // Publish an initial value to the entry so there's something there.
        if (!entryKS.exists()) {
            entryKS.setDefault(kS);
            entryKV.setDefault(kV);
            entryKA.setDefault(kA);
        }

        // Set the initial feedforward to the defaults or to previously set values on networktables
        m_setFeedforward.accept(new SimpleMotorFeedforward(entryKS.get(), entryKV.get(), entryKA.get()));
    }

    /**
     * Check if there have been any changes since the last call to this method.
     */
    public void checkForUpdates() {
        boolean changes = false;

        if (entryKS.get() != prevKS) {
            prevKS = entryKS.get();
            changes = true;
        }

        if (entryKV.get() != prevKV) {
            prevKV = entryKV.get();
            changes = true;
        }

        if (entryKA.get() != prevKA) {
            prevKA = entryKA.get();
            changes = true;
        }

        if (changes) {
            m_setFeedforward.accept(new SimpleMotorFeedforward(prevKS, prevKV, prevKA));
        }
    }
}
