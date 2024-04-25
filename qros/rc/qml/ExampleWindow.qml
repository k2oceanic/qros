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

    Component.onCompleted: {
        // let intList = [1, 2, 3, 4]
        applicationNode.declareParameter("cmd.ip", "0.0.0.0");
        applicationNode.declareParameter("cmd.port", 0);
        applicationNode.declareParameter("vector", [1, 2, 3, 4, 5]);
        for (var key in applicationNode.parameters) {
           console.log("Key:", key, "Value:", applicationNode.parameters[key]);
       }
    }

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
    Column{
        Label{
            id: myLabel
           // anchors.centerIn: parent
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

        Label{
            id: paramlabel
            text: applicationNode.parameters["cmd.ip"]
        }
        Row{
            spacing:  10
            TextField{
                id: nodeName
                text: "/parameter_listener_node"
            }
            TextField{
                id: paramName
                text: "my_parameter"
            }
            TextField{
                id: paramValue
                text: "hello"
            }
            Button{
                id: sendParam
                text: "send"
                onClicked: {
                    applicationNode.setParameterAsync(nodeName.text,paramName.text,paramValue.text)
                }
            }
        }
    }

    QRosRawPacketPublisher{
        id: rawPub
    }
        id: myButton
    Button {

        node: applicationNode
        onClicked: {
        text: "Publish Raw Packet"
        anchors.left: myLabel.right
            rawPub.setTopic("/from_qml/raw_packet")
            rawPub.setData("Hello World")  // Convert QString to QByteArray
            rawPub.publish()
        }
    }
}
