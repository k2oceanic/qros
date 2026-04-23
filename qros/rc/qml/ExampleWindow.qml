import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import "Components"
import QRos 1.0

ApplicationWindow {
    visible: true
    width: 1000
    height: 800
    title: "qros Example"

    // Topics assigned directly in the object literal — no Component.onCompleted needed
    QRosStringPublisher  { id: strPub;   node: applicationNode; topic: "/qml/echo" }
    QRosStringSubscriber { id: strSub;   node: applicationNode; topic: "/qml/echo";    staleTimeout: 3.0 }

    QRosDoublePublisher  { id: dblPub;   node: applicationNode; topic: "/qml/value" }
    QRosDoubleSubscriber { id: dblSub;   node: applicationNode; topic: "/qml/value";   staleTimeout: 2.0 }

    QRosBoolPublisher    { id: boolPub;  node: applicationNode; topic: "/qml/flag" }
    QRosBoolSubscriber   { id: boolSub;  node: applicationNode; topic: "/qml/flag";    staleTimeout: 5.0 }

    // latched: true — late subscribers receive the last published value on connect
    QRosStringPublisher  { id: latchPub; node: applicationNode; topic: "/qml/latched"; latched: true }
    QRosStringSubscriber { id: latchSub; node: applicationNode; topic: "/qml/latched"; latched: true }

    QRosTriggerServiceClient {
        id: trigClient
        node: applicationNode
        Component.onCompleted: serviceName = "/reset"
    }

    ScrollView {
        anchors.fill: parent
        clip: true

        Column {
            x: 12; y: 12
            width: parent.width - 24
            spacing: 10

            GroupBox {
                width: parent.width
                title: "String — /qml/echo"
                Column { spacing: 6
                    Row { spacing: 8
                        TextField { id: strInput; placeholderText: "message…"; width: 300 }
                        Button { text: "Publish"; onClicked: { strPub.data = strInput.text; strPub.publish() } }
                    }
                    Label { text: "received: " + (strSub.data || "—") + (strSub.isStale ? "  [stale]" : "") }
                }
            }

            GroupBox {
                width: parent.width
                title: "Float64 — /qml/value  (staleTimeout: 2 s)"
                Column { spacing: 6
                    Row { spacing: 8
                        Slider { id: dblSlider; from: -100; to: 100; width: 300
                            onValueChanged: { dblPub.data = value; dblPub.publish() }
                        }
                        Label { text: dblSlider.value.toFixed(2); anchors.verticalCenter: parent.verticalCenter }
                    }
                    Label { text: "received: " + dblSub.data.toFixed(3) + (dblSub.isStale ? "  [stale]" : "") }
                }
            }

            GroupBox {
                width: parent.width
                title: "Bool — /qml/flag  (staleTimeout: 5 s)"
                Row { spacing: 12
                    Switch { id: boolSwitch; text: "publish"
                        onCheckedChanged: { boolPub.data = checked; boolPub.publish() }
                    }
                    Label { text: "received: " + boolSub.data + (boolSub.isStale ? "  [stale]" : "");
                            anchors.verticalCenter: parent.verticalCenter }
                }
            }

            GroupBox {
                width: parent.width
                title: "Latched — /qml/latched  (subscriber receives last value on connect)"
                Column { spacing: 6
                    Row { spacing: 8
                        TextField { id: latchInput; placeholderText: "value to hold…"; width: 300 }
                        Button { text: "Publish Latched"; onClicked: { latchPub.data = latchInput.text; latchPub.publish() } }
                    }
                    Label { text: "held value: " + (latchSub.data || "—") }
                }
            }

            GroupBox {
                width: parent.width
                title: "Trigger Service Client"
                Column { spacing: 6
                    Row { spacing: 8
                        TextField { id: svcField; text: trigClient.serviceName; width: 300
                            onEditingFinished: trigClient.serviceName = text }
                        Button { text: "Call"; onClicked: trigClient.callService() }
                    }
                    Label { text: "response: " + (trigClient.respMessage || "—") }
                }
            }

            GroupBox {
                width: parent.width
                title: "Parameters"
                Column { spacing: 6
                    Component.onCompleted: {
                        applicationNode.declareParameter("demo.value", 42.0)
                        applicationNode.declareParameter("demo.label", "hello qros")
                    }
                    Label { text: "demo.value: " + (applicationNode.parameters["demo.value"] ?? "—") +
                                  "   demo.label: " + (applicationNode.parameters["demo.label"] ?? "—") }
                    Row { spacing: 8
                        TextField { id: extNode;  text: "/listener_node"; width: 160 }
                        TextField { id: extParam; text: "my_param";       width: 120 }
                        TextField { id: extValue; text: "hello";           width: 120 }
                        Button { text: "Set"; onClicked: applicationNode.setExternalParameterAsync(extNode.text, extParam.text, extValue.text) }
                        Button { text: "Get"; onClicked: applicationNode.getExternalParametersAsync(extNode.text, [extParam.text]) }
                    }
                    Connections {
                        target: applicationNode
                        function onParametersGetResult(success, nodeName, params, error) {
                            paramResult.text = success
                                ? Object.entries(params).map(function(e) { return e[0] + ": " + e[1] }).join(", ")
                                : "error: " + error
                        }
                    }
                    Label { id: paramResult; text: "—" }
                }
            }

            TFDemoPanel { width: parent.width; node: applicationNode }
        }
    }
}
