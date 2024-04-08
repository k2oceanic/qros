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
            topic="/topic"
        }
    }

    QRosStringPublisher{
        id: stringPub
        node: applicationNode
        // Component.onCompleted:{
        //     topic="/from_qml"
        // }

    }

    Label{
        id: myLabel
        anchors.centerIn: parent
        text: stringSub.data
        onTextChanged: {
            stringPub.data = text
            stringPub.publish(text)
        }
    }

    TextField{
        id: textField
        text: "/from_qml"
        onTextChanged: {
            stringPub.setTopic(textField.text)
        }
    }

    QRosRawPacketPublisher{
        id: rawPub
        node: applicationNode
    }

    Button {
        id: myButton
        anchors.left: myLabel.right
        text: "Publish Raw Packet"
        onClicked: {
            rawPub.setTopic("/from_qml/raw_packet")
            rawPub.setData("Hello World")  // Convert QString to QByteArray
            rawPub.publish()
        }
    }
}
