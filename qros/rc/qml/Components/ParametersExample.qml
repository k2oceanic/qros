import QtQuick 2.15
import QtQuick.Controls 2.15
import QRos 1.0

// Demonstrates local node parameters and getting/setting parameters on external nodes.
// Local parameters are declared on startup and read back via the parameters map.
// External parameters can be get/set on any running node by name.
GroupBox {
    id: root
    title: "Node Parameters"
    property var node

    Component.onCompleted: {
        node.declareParameter("demo.value", 42.0)
        node.declareParameter("demo.label", "hello qros")
    }

    Column {
        spacing: 6

        Label {
            text: "demo.value: " + (root.node.parameters["demo.value"] ?? "—") +
                  "   demo.label: " + (root.node.parameters["demo.label"] ?? "—")
        }

        Row {
            spacing: 8
            TextField { id: extNode;  text: "/listener_node"; width: 160 }
            TextField { id: extParam; text: "my_param";       width: 120 }
            TextField { id: extValue; text: "hello";           width: 120 }
            Button {
                text: "Set"
                onClicked: root.node.setExternalParameterAsync(extNode.text, extParam.text, extValue.text)
            }
            Button {
                text: "Get"
                onClicked: root.node.getExternalParametersAsync(extNode.text, [extParam.text])
            }
        }

        Connections {
            target: root.node
            function onParametersGetResult(success, nodeName, params, error) {
                result.text = success
                    ? Object.entries(params).map(function(e) { return e[0] + ": " + e[1] }).join(", ")
                    : "error: " + error
            }
        }

        Label { id: result; text: "—" }
    }
}
