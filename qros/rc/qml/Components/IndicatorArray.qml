import QtQuick 2.15
import QtQuick.Layouts 1.15
import QtQuick.Controls 2.15 // For ToolTip

Rectangle {
    color: "black"
    width: 500
    height: 50
    property int size: 20

    ListModel {
        id: lightStates
        Component.onCompleted: {
            for (var i = 0; i < size; i++) {
                append({"isOn": Math.random() > 0.5, "id": i})
            }
        }
    }

    RowLayout {
        anchors.centerIn: parent
        spacing: 3

        Repeater {
            model: lightStates

            delegate: Indicator {
                isOn: model.isOn
                label: model.id
                activeColor: "red" // You can customize this for each instance if needed
                MouseArea {
                    anchors.fill: parent
                    onClicked: {
                        lightStates.set(index, {"isOn": !model.isOn});
                    }
                }
            }
        }
    }

    function setState(index, state) {
        if (index >= 0 && index < lightStates.count) {
            lightStates.set(index, {"isOn": state});
        }
    }
}
