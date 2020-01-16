/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Add your docs here.
 */
public class Dashboard238 {

    private HashMap<String, NetworkTableEntry> dashboardEntries = new HashMap<>();
    ShuffleboardTab diagnosticTab;
    
    public void init() {
        diagnosticTab = Shuffleboard.getTab("DiagnosticTab");

        // create widgets
        buildElement(diagnosticTab, "LEncoder", 0, 1, 1, 1, 0);
        buildElement(diagnosticTab, "REncoder", 0, 1, 1, 1, 1);
        // put on tabs
        // create get and set method

    }

    private void buildElement(String elementName, Boolean value, int sizeX, int sizeY, int posX, int posY) {
        SimpleWidget theWidget = diagnosticTab.add(elementName, value);
        dashboardEntries.put(elementName, theWidget.getEntry());
        theWidget.withSize(sizeX, sizeY).withPosition(posX, posY);
    }

    private void buildElement(String elementName, Double value, int sizeX, int sizeY, int posX, int posY) {
        SimpleWidget theWidget = diagnosticTab.add(elementName, value);
        dashboardEntries.put(elementName, theWidget.getEntry());
        theWidget.withSize(sizeX, sizeY).withPosition(posX, posY);
    }

    private void buildElement(String elementName, int value, int sizeX, int sizeY, int posX, int posY) {
        buildElement(diagnosticTab, elementName, value, sizeX, sizeY, posX, posY);
    }

    private void buildElement(ShuffleboardTab tab, String elementName, int value, int sizeX, int sizeY, int posX,
            int posY) {
        SimpleWidget theWidget = tab.add(elementName, value);
        dashboardEntries.put(elementName, theWidget.getEntry());
        theWidget.withSize(sizeX, sizeY).withPosition(posX, posY);
    }

}
