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
                    applicationNode.setExternalParameterAsync(nodeName.text,paramName.text,paramValue.text)
                }
            }
            Button{
                id: getParam
                text: "get"
                onClicked: {
                    applicationNode.getExternalParametersAsync(nodeName.text,[paramName.text, paramName2.text])
                }
            }

            Label {
                id: resultLabel
                text: "Parameter result will appear here."
            }

            Component.onCompleted: {
                applicationNode.parametersGetResult.connect(handleParametersGetResult);
            }

            function handleParametersGetResult(success, nodeName, params, error) {
                if (success) {
                    var resultText = "Parameters from " + nodeName + ":\n";
                    for (var key in params) {
                        resultText += key + ": " + params[key] + "\n";
                    }
                    resultLabel.text = resultText;
                } else {
                    console.log("Failed to get parameters from " + nodeName + ": " + error);
                    resultLabel.text = "Failed to get parameters: " + error;
                }
            }
        }

        Row {
            TextField{
                id: paramName2
                text: "my_parameter2"
            }
        }
    }
}
