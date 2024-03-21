import QtQuick 2.15
import "../Components"
import OdysseusUi 1.0

Item {
    id: root
    property var subscriber: undefined
    property double maxValue: 80 // Maximum value for the bar
    property double minValue: 0 // Minimum value for the bar, can be negative

    property double greenUpper: 50 // Upper end value of the green section
    property double greenLower: 0 // Lower start value of the green section

    property double yellowUpper: 70 // Upper end value of the yellow section
    property double yellowLower: 0 // Lower start value of the yellow section

    property string label: "Card Temp"

    width: 200
    height: 100

    NumericalDisplay{
        id: display
        minValue: root.minValue
        maxValue: root.maxValue

        greenUpper: root.greenUpper
        greenLower: root.greenLower

        yellowLower: root.yellowLower
        yellowUpper: root.yellowUpper
        unit: "c"
        label: root.label

        anchors.fill: parent
    }

    function mySlot() {

        if (subscriber === undefined) {
            console.log("subscriber not defined");
        } else {
            display.currentValue = root.subscriber.getTemp()
        }
    }

    Connections {
        target: root.subscriber
        function onMsgReceived(){mySlot()}
    }

}
