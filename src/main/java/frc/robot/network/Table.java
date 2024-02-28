package frc.robot.network;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class Table {
    public final String name;
    // The networktable for the limelight
    private final NetworkTable table;
    
    // Cache the NetworkTableEntries so we don't have to keep accessing it
    private final Map<String, NetworkTableEntry> entryCache = new HashMap<>();

    protected Table(String name, NetworkTable table){
        this.name = name;
        this.table = table;
    }

    public String getName() {
        return name;
        
    }

    public NetworkTableEntry getEntry(String key){
        if(entryCache.containsKey(key))
            return entryCache.get(key);
        var entry = table.getEntry(key);
        entryCache.put(key, entry);
        return entry;
    }

    interface foo<T> {
        T apply(NetworkTableEntry entry, T defaultValue);
    }

    private <T> T getEntry(String key, foo<T> func, T defaultValue){
        return func.apply(getEntry(key), defaultValue);
    }

    public boolean getBoolean(String key){
        return getEntry(key, NetworkTableEntry::getBoolean, false);
    }

    public double getDouble(String key){
        return getEntry(key, NetworkTableEntry::getDouble, 0.0);
    }
    
    public float getFloat(String key){
        return getEntry(key).getFloat(0.0f);
    }

    public int getInteger(String key){
        return (int) getEntry(key).getInteger(0);
    }

    public String getString(String key){
        return getEntry(key).getString("");
    }
    public double[] getDoubleArray(String key) {
        return getEntry(key).getDoubleArray(new double[]{});
    }

    public Map<String, String> getAllEntries(){
        var keys = table.getKeys();
        Map<String, String> res = new HashMap<>();
        for(var key : keys){
            var entry = getEntry(key);
            var str = switch (entry.getType()) {
                case kBoolean -> Boolean.toString(entry.getBoolean(false));
                case kDouble -> Double.toString(entry.getDouble(0));
                case kFloat -> Float.toString(entry.getFloat(0));
                case kString -> entry.getString("UNKNOWN STRING");
                case kDoubleArray -> Arrays.toString(entry.getDoubleArray(new double[]{}));
                default -> "UNKNOWN VALUE TYPE: " + entry.getType().toString();
            };
            res.put(key, str);
        }

        return res;
    }
}