// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;


/** Add your docs here. */
public class LookupTables {
    private static InterpolatingTreeMap<Double, Double> shooterAngleTable = new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static InterpolatingTreeMap<Double, Double> shooterSpeedTable = new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    
    // private static InterpolatingTreeMap<Double, Double> shooterAngleTable = new InterpolatingTreeMap<Double, Double>(
    // (up, q, down) -> {
    //     double upperToLower = up.doubleValue() - down.doubleValue();
    //     if(upperToLower <= 0.0){
    //         return 0.0;
    //     }
    //     double queryToLower = q.doubleValue() - down.doubleValue();
    //     if(queryToLower <= 0.0){
    //         return 0.0;
    //     }
    //     return queryToLower/upperToLower;
    // },
    // (val1, val2, d) -> {
    //     double dydx = val2.doubleValue() - val1.doubleValue();
    //     return dydx * d + val1.doubleValue();
    // });
    // private static InterpolatingTreeMap<Double, Double> shooterSpeedTable = new InterpolatingTreeMap<Double, Double>(
    // (up, q, down) -> {
    //     double upperToLower = up.doubleValue() - down.doubleValue();
    //     if(upperToLower <= 0.0){
    //         return 0.0;
    //     }
    //     double queryToLower = q.doubleValue() - down.doubleValue();
    //     if(queryToLower <= 0.0){
    //         return 0.0;
    //     }
    //     return queryToLower/upperToLower;
    // },
    // (val1, val2, d) -> {
    //     double dydx = val2.doubleValue() - val1.doubleValue();
    //     return dydx * d + val1.doubleValue();
    // });



    /**
     * <pre>
     *Inits Values for the Interpolating Tables
     *Key is Distance from Speaker,
     *Value is RPM or Angle Value at that distance
     * </pre>
     */
    public static void InitValues() {
        //                    DISTANCE     ENCODER VALUE
        shooterAngleTable.put(1.5303, .2929); // .3005 then 2905
        shooterAngleTable.put(1.9748, .2991); // .3063 then 2963
        shooterAngleTable.put(2.1844, .3173); // .3249 then 3149
        shooterAngleTable.put(2.4384, .3211); // .3287 then 3187
        shooterAngleTable.put(2.6797, .330); // .3376 then 3276
        shooterAngleTable.put(2.9337, .3319); // .3395 then 3295
        shooterAngleTable.put(3.0607, .3358); // .3434 then 3334
        shooterAngleTable.put(3.3655, .3407); // .3483 then 3383
        shooterAngleTable.put(3.683, .3505); // .3581 then 3481
        shooterAngleTable.put(3.9243, .3525); // .3601 then 3501
        shooterAngleTable.put(4.4323, .3563); // .3639 then 3539
        shooterAngleTable.put(4.699, .3583); // .3659 then 3559
        shooterAngleTable.put(4.9403, .3621); // .3697 then 3597
        shooterAngleTable.put(5.1943, .3623); // .3699 then 3599
        shooterAngleTable.put(5.4483, .3631); // .3707 then 3607


        //shooterAngleTable.put(1.905, .75563); // .74663
        //shooterAngleTable.put(2.2987, .766); // .757
        //shooterAngleTable.put(2.809875, .7748); // .7658
        //shooterAngleTable.put(3.39725, .7904); // .7814
        //shooterAngleTable.put(3.9497, .792); // .783
        //shooterAngleTable.put(4.5085, .8099); // .8009
        //shooterAngleTable.put(5.0927, .812); // .803
        //shooterAngleTable.put(5.7531, .813); // .804

        // new values
       
    
    }

    public static InterpolatingTreeMap<Double, Double> getAngleTable(){
        InitValues(); 
        return shooterAngleTable;
    }

    public static InterpolatingTreeMap<Double, Double> getSpeedTable(){
        InitValues();
        return shooterAngleTable;
    }

    public static double getSpeedValueAtDistance(Double distMeterFromSpeaker) {
        InitValues();
        return (double) shooterSpeedTable.get(distMeterFromSpeaker);
    }

    public static double getAngleValueAtDistance(Double distMeterFromSpeaker) {
        InitValues();
        //System.out.println(shooterAngleTable.get(distMeterFromSpeaker));
        return (double) shooterAngleTable.get(distMeterFromSpeaker);  
    }
    
    
}
