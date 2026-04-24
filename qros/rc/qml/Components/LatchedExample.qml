import QtQuick 2.15
import QtQuick.Controls 2.15
import QRos 1.0

// Demonstrates latched (transient_local) QoS.
// Publish a value, then restart the subscriber — it will receive the last
// value immediately on connect without the publisher having to re-publish.
GroupBox {
    id: root
    title: "Latched Topic — /qml/latched"
    property var node

    QRosStringPublisher  { id: pub; node: root.node; topic: "/qml/latched"; latched: true }
    QRosStringSubscriber { id: sub; node: root.node; topic: "/qml/latched"; latched: true }

    Column {
        spacing: 6
        Row {
            spacing: 8
            TextField { id: input; placeholderText: "value to hold…"; width: 300 }
            Button {
                text: "Publish Latched"
                onClicked: { pub.data = input.text; pub.publish() }
            }
        }
        Label { text: "held value: " + (sub.data || "—") }
    }
}
