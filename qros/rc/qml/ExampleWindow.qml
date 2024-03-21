import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
import QtQuick.Controls.Material 2.15
import "Components"
import "DataDisplays"
import QRos 1.0

ApplicationWindow {
    id: ioWindow
    visible: true
    width: 1920
    height: 1080

    QRosStringSubscriber{
        id: stringSub
        node: applicationNode
        Component.onCompleted:{
            subscribe("/topic")
        }
    }

    Label{
        id: myLabel
        anchors.centerIn: parent
        text: stringSub.data
    }
}
