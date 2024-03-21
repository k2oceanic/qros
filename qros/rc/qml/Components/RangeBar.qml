//import QtQuick 2.15
//import QtQuick.Controls 2.15

//Item {
//    id: root
//    width: 100
//    height: 300

//    property int maxValue: 100 // Maximum value for the bar
//    property int minValue: -100
//    property int warningValue: 70 // Starting value of the warning section
//    property int dangerUpper: 85 // Starting value of the limit section
//    property int currentValue: 0 // Current value, dynamically updated

//    // Function to calculate the pointer position based on the current value
//    function calculatePointerY(value) {
//        var deltaValue = maxValue - minValue
//        var normalizedValue = value/deltaValue
//        var normalizedMinValue = minValue/Value
//        var screenSpaceValue = normalizedValue
//        return root.height - (value / deltaValue * root.height);
//    }

//    // Background of the bar
//    Rectangle {
//        id: barBackground
//        anchors.fill: parent
//        color: "#e0e0e0"
//    }

//    // Green section
//    Rectangle {
//        id: greenSection
//        width: parent.width
//        height: parent.height * (warningValue / maxValue)
//        color: "#47a159"
//        anchors.bottom: parent.bottom
//    }

//    // Yellow section
//    Rectangle {
//        id: yellowSection
//        width: parent.width
//        height: parent.height * ((dangerUpper - warningValue) / maxValue)
//        color: "#ffff4f"
//        anchors.bottom: greenSection.top
//    }

//    // Red section
//    Rectangle {
//        id: redSection
//        width: parent.width
//        height: parent.height * ((maxValue - dangerUpper) / maxValue)
//        color: "#fe5151"
//        anchors.bottom: yellowSection.top
//    }

//    // Pointer for the current value
//    Rectangle {
//        id: pointer
//        width: parent.width
//        height: 5
//        color: "#80ffffff"
//        radius: 2
//        y: calculatePointerY(parent.currentValue) - height / 2 // Adjust pointer position
//        anchors.horizontalCenter: parent.horizontalCenter
//    }
//}

import QtQuick 2.15

Item {
    id: root
    width: 100
    height: 300

    property double maxValue: 100 // Maximum value for the bar
    property double minValue: -100 // Minimum value for the bar, can be negative


    property double greenUpper: 50 // Upper end value of the green section
    property double greenLower: -50 // Lower start value of the green section

    property double yellowUpper: 70 // Upper end value of the yellow section
    property double yellowLower: -70 // Lower start value of the yellow section

    property double currentValue: 0 // Current value, dynamically updated

    // Function to scale a value to the bar's height
    function scaleToHeight(value) {
        return ((value - minValue) / (maxValue - minValue)) * root.height;
    }

    // Red section dynamically fills the space based on other sections
    Rectangle {
        id: redSection
        width: parent.width
        height: parent.height
        color: "#fe5151"
    }

    // Yellow section
    Rectangle {
        id: yellowSection
        width: parent.width
        height: scaleToHeight(yellowUpper) - scaleToHeight(yellowLower)
        color: "#ffff4f"
        y: root.height - scaleToHeight(yellowUpper)
    }


    // Green section
    Rectangle {
        id: greenSection
        width: parent.width
        height: scaleToHeight(greenUpper) - scaleToHeight(greenLower)
        color: "#47a159"
        y: root.height - scaleToHeight(greenUpper)
    }


    // Pointer for the current value
    Rectangle {
        id: pointer
        width: parent.width
        height: 6
        color: "#d2ffffff"
        radius: 3
        border.width: 1
        y: root.height - scaleToHeight(currentValue) - height / 2
        anchors.horizontalCenter: parent.horizontalCenter
    }



}

