import QtQuick 2.15

Item {
    id: root
    width: 300
    height: 100

    property real currentValue: 0
    property string unit: ""
    property int decimalPlaces: 1

    // Example current value, bind this to your actual data source

    property double maxValue: 100 // Maximum value for the bar
    property double minValue: -100 // Minimum value for the bar, can be negative

    property double greenUpper: 50 // Upper end value of the green section
    property double greenLower: -50 // Lower start value of the green section

    property double yellowUpper: 70 // Upper end value of the yellow section
    property double yellowLower: -70 // Lower start value of the yellow section

    property string label: "Value"

    function textColor(){
        var textColor =  "#47a159"
        if(currentValue > greenLower && currentValue < greenUpper){
            textColor = "#47a159"
        }else if(currentValue > yellowLower && currentValue < yellowUpper){
            textColor = "#ffff4f"
        }else{
            textColor = "#fe5151"
        }
        return textColor
    }
    // Background rounded rectangle
    Rectangle {
        id: backgroundRect
        anchors.fill: parent
        anchors.rightMargin: 0
        anchors.bottomMargin: 0
        anchors.leftMargin: 0
        anchors.topMargin: 0
        radius: 20
        border.width: 0
        color: Qt.rgba(0, 0, 0, 0.25)


        RangeBar {
            id: rangeBar
            height: parent.height
            visible: true
            width: parent.width * 0.1
            anchors.right: parent.right
            currentValue: root.currentValue

            minValue: root.minValue
            maxValue: root.maxValue

            greenUpper: root.greenUpper
            greenLower: root.greenLower

            yellowLower: root.yellowLower
            yellowUpper: root.yellowUpper

        }
    }

//    // Line graph
//    Canvas {
//        id: canvas
//        anchors.fill: parent
//        anchors.rightMargin: 0
//        anchors.bottomMargin: 0
//        anchors.leftMargin: 0
//        anchors.topMargin: 0
//        onPaint: {
//            var ctx = getContext("2d");
//            ctx.beginPath();
//            ctx.strokeStyle = "rgba(0, 0, 255, 0.5)"; // Semi-transparent line color
//            ctx.lineWidth = 5;
//            // Example data points, replace with your actual data
//            var points = [[10, 50], [50, 100], [100, 30], [150, 80], [200, 60], [250, 120]];
//            for (var i = 0; i < points.length; i++) {
//                var point = points[i];
//                if (i === 0) ctx.moveTo(point[0], point[1]);
//                else ctx.lineTo(point[0], point[1]);
//            }
//            ctx.stroke();
//        }
//    }

    // Current value display
    SIPrefixDisplay {
        id: valueDisplay
        color: textColor() //"#47a159"
        //text: currentValue.toFixed(1)
        numericValue: root.currentValue
        unit: root.unit
        decimalPlaces: root.decimalPlaces
        font.pixelSize: .75 * root.height
        anchors.verticalCenterOffset: parent.height * -0.1
        anchors.horizontalCenterOffset: 0
        anchors.centerIn: parent
    }

    Text {
        id: text1
        color: "#ffffff"
        text: label
        anchors.bottom: parent.bottom
        font.pixelSize: 22
        anchors.horizontalCenterOffset: 0
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottomMargin: 8
    }






    // Update the canvas whenever the component size changes or data updates
    // Component.onCompleted: canvas.requestPaint()
    // onWidthChanged: canvas.requestPaint()
    // onHeightChanged: canvas.requestPaint()
}
