import QtQuick 2.0

Item {
    id: root
    property bool isOn: true // Expose isOn as a property
    property string label: "0" // Expose lightId as a property
    property string activeColor: "red" // Default color, can be overridden
    property bool animated: true

    width: 20 // Default width
    height: 20 // Default height

    Rectangle {
        id: light
        width: parent.width
        height: parent.height
        color: isOn ? activeColor : "grey"
        radius: width / 3
        anchors.centerIn: parent
        opacity: isOn ? 1.0 : 0.5 // Start fully visible if on

        // Pulsate effect when 'on'
        SequentialAnimation on opacity {
            loops: Animation.Infinite
            running: isOn && animated // Only run when the light is 'on'

            NumberAnimation {
                from: 0.5
                to: 1.0
                duration: 500
            }
            NumberAnimation {
                from: 1.0
                to: 0.5
                duration: 500
            }
        }
    }

    Text {
        text: label
        font.pointSize: parent.height/3
        anchors.centerIn: parent
        color: "white"
    }
}
