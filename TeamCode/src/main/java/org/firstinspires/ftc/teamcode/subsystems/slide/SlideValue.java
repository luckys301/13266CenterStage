package org.firstinspires.ftc.teamcode.subsystems.slide;

public class SlideValue {
    public enum SlideEnum {
        TRANSFER(0.0),

        LOW(0.0),
        MID(0.0),
        HIGH(0.0),

        MANUAL(0.0);
        public final double value;
        SlideEnum(double value) {
            this.value = value;
        }
    }
    public SlideEnum slideEnum;
    public volatile double slidePosition;
    public volatile boolean shouldSlideDrop;

    protected SlideValue(SlideEnum slideEnum, double slidePosition, boolean shouldSlideDrop) {
        this.slideEnum = slideEnum;
        this.slidePosition = slidePosition;
        this.shouldSlideDrop = shouldSlideDrop;
    }
    protected static SlideValue make(SlideEnum slideEnum, double pivotPosition, boolean shouldSlideDrop) {
        return new SlideValue(slideEnum, pivotPosition, shouldSlideDrop);
    }
    protected static SlideValue make(SlideEnum slideEnum, boolean shouldSlideDrop) {
        return new SlideValue(slideEnum, slideEnum.value, shouldSlideDrop);
    }

//    protected double getSlidePosition(){
//        return slidePosition;
//    }
//    protected boolean getShouldSlideDrop(){
//        return shouldSlideDrop;
//    }
//    protected SlideEnum getEnum(){
//        return slideEnum;
//    }
    public final boolean equals(Object other) {
        return this==other;
    }

}
