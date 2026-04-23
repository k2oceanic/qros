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
        applicationNode.declareParameter("cmd.ip", "0.0.0.0")
        applicationNode.declareParameter("cmd.port", 0)
        applicationNode.declareParameter("vector", [1, 2, 3, 4, 5])
        for (var key in applicationNode.parameters) {
           console.log("Key:", key, "Value:", applicationNode.parameters[key])
        }
    }

    // --- existing ROS demo bits ---
    QRosStringSubscriber{
        id: stringSub
        node: applicationNode
        topic: "/topic"
    }

    QRosStringPublisher{ id: stringPub; node: applicationNode }

    QRosStringPublisher{
        id: syncStringPub
        node: applicationNode
        topic: "/input_topic1"
    }

    QRosStringSubscriber{
        id: syncStringSub
        node: applicationNode
        latched: true
        Component.onCompleted: topic="/latched_topic1"
    }

    Column{
        spacing: 12

        Label{
            id: myLabel
            text: stringSub.data
            onTextChanged: {
                stringPub.data = text
                stringPub.publish(text)
            }
        }

        TextField{
            id: textField
            text: "/from_qml"
            onTextChanged: stringPub.setTopic(textField.text)
        }

        Label{
            id: paramlabel
            text: applicationNode.parameters["cmd.ip"]
        }

        Row{
            spacing:  10
            TextField{ id: nodeName;  text: "/parameter_listener_node" }
            TextField{ id: paramName; text: "my_parameter" }
            TextField{ id: paramValue; text: "hello" }
            Button{
                id: sendParam
                text: "send"
                onClicked: applicationNode.setExternalParameterAsync(nodeName.text, paramName.text, paramValue.text)
            }
            Button{
                id: getParam
                text: "get"
                onClicked: applicationNode.getExternalParametersAsync(nodeName.text,[paramName.text, paramName2.text])
            }
            Label { id: resultLabel; text: "Parameter result will appear here." }

            Component.onCompleted: applicationNode.parametersGetResult.connect(handleParametersGetResult)
            function handleParametersGetResult(success, nodeName, params, error) {
                if (success) {
                    var resultText = "Parameters from " + nodeName + ":\n"
                    for (var key in params) resultText += key + ": " + params[key] + "\n"
                    resultLabel.text = resultText
                } else {
                    console.log("Failed to get parameters from " + nodeName + ": " + error)
                    resultLabel.text = "Failed to get parameters: " + error
                }
            }
        }

        Row {
            spacing: 10
            TextField{ id: paramName2; text: "my_parameter2" }
            Button {
                text: "List All Parameters"
                onClicked: applicationNode.listExternalParametersAsync(nodeName.text, 1000)
            }
            Label { id: listLabel; text: "Parameter list will appear here."; wrapMode: Text.WordWrap }
            Connections {
                target: applicationNode
                onParametersListResult: {
                    listLabel.text = success
                        ? "Parameters available in " + nodeName.text + ":\n" + param_names.join("\n")
                        : "Failed to list parameters from " + nodeName.text + ": " + error
                }
            }
        }

        // Joint State Publisher
        QRosJointStatePublisher {
            id: jointStatePublisher
            node: applicationNode
            Component.onCompleted: topic = "/pt25_roll_cmd"
            jointNames: ["joint1"]
            positions: [0.0]
            velocities: [0.0]
            efforts: [0.0]
        }

        // Joint State Subscriber
        QRosJointStateSubscriber {
            id: jointStateSubscriber
            node: applicationNode
            Component.onCompleted: topic = "/pt25_roll"
            onJointStateChanged: {
                console.log("Joint state updated")
                console.log("Joint names:", jointStateSubscriber.jointNames)
                console.log("Positions:", jointStateSubscriber.positions)
                console.log("Velocities:", jointStateSubscriber.velocities)
                console.log("Efforts:", jointStateSubscriber.efforts)
            }
        }

        // UI Elements to display joint states
        Label { text: "Joint Names: " + jointStateSubscriber.jointNames.join(", ") }
        Label { text: "Positions: " + jointStateSubscriber.positions.join(", ") }
        Label { text: "Velocities: " + jointStateSubscriber.velocities.join(", ") }
        Label { text: "Efforts: " + jointStateSubscriber.efforts.join(", ") }

        Row {
            TextField {
                id: positionField1
                placeholderText: "Position"
                onAccepted: jointStatePublisher.positions[0] = parseFloat(positionField1.text)
            }
        }
        Button {
            text: "Update Joint States"
            onClicked: {
                jointStatePublisher.positions = [parseFloat(positionField1.text)]
                jointStatePublisher.publish()
            }
        }

        TextField{
            id: syncLabel
            property string textState: syncStringSub.data
            onEditingFinished: {
                syncStringPub.data = text
                syncStringPub.publish()
            }
            onTextStateChanged: syncLabel.text = textState
        }

        Button { onClicked: trigService.callService() }
        Label { text: trigService.respMessage }

        QRosTriggerServiceClient{
            id: trigService
            node: applicationNode
            Component.onCompleted: serviceName= "/odysseus4k/nav/reset_map_frame"
        }

        // =======================
        // TF DEMO PANEL (new component)
        // =======================
        TFDemoPanel {
            id: tfPanel
            node: applicationNode
            // Optionally set defaults:
            // targetFrame: "map"
            // sourceFrame: "base_link"
        }
        // ===== end TF demo =====
    }
}
