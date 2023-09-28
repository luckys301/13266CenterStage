package org.firstinspires.ftc.teamcode.subsystems.arm;

public class ArmValue {
    protected enum PivotEnum {
        TRANSFER(0.0),

        LOW(0.0),
        MID(0.0),
        HIGH(0.0),
        MANUAL(0.0) // To use when the subsystem is going Manually - No Need to make PivotValue,
        //^^ ??
        ;
        public final double value;
        PivotEnum(double value) {
            this.value = value;
        }
    }
    public PivotEnum pivotEnum;
    public volatile double pivotPosition;
    public volatile boolean shouldSensorWork;


    protected ArmValue(PivotEnum pivotEnum, double pivotPosition, boolean shouldSensorWork) {
        this.pivotEnum = pivotEnum;
        this.pivotPosition = pivotPosition;
        this.shouldSensorWork = shouldSensorWork;
    }
    protected static ArmValue make(PivotEnum pivotEnum, double pivotPosition, boolean shouldSensorWork) {
        return new ArmValue(pivotEnum, pivotPosition, shouldSensorWork);
    }

//    protected double getPivotPosition(){
//        return pivotPosition;
//    }
//    protected boolean getShouldSensorWork(){
//        return shouldSensorWork;
//    }
//    protected PivotEnum getEnum(){
//        return pivotEnum;
//    }
    public final boolean equals(Object other) {
        return this==other;
    }
}