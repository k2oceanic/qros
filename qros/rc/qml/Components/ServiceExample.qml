import QtQuick 2.15
import QtQuick.Controls 2.15
import QRos 1.0

// Calls a std_srvs/Trigger service and displays the response message.
// Edit the service name field to point at any Trigger service on the network.
GroupBox {
    id: root
    title: "Trigger Service Client"
    property var node

    QRosTriggerServiceClient {
        id: client
        node: root.node
        Component.onCompleted: serviceName = "/reset"
    }

    Column {
        spacing: 6
        Row {
            spacing: 8
            TextField {
                id: svcField
                text: client.serviceName
                width: 300
                onEditingFinished: client.serviceName = text
            }
            Button { text: "Call"; onClicked: client.callService() }
        }
        Label { text: "response: " + (client.respMessage || "—") }
    }
}
