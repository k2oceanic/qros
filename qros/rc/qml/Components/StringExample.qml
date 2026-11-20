import QtQuick 2.15
import QtQuick.Controls 2.15
import QRos 1.0

// Publishes and subscribes to a std_msgs/String on the same topic.
// The subscriber shows stale detection: if no message arrives within
// staleTimeout seconds, isStale becomes true.
GroupBox {
    id: root
    title: "String Pub/Sub — /qml/echo"
    property var node

    QRosStringPublisher  { id: pub; node: root.node; topic: "/qml/echo" }
    QRosStringSubscriber { id: sub; node: root.node; topic: "/qml/echo"; staleTimeout: 3.0 }

    Column {
        spacing: 6
        Row {
            spacing: 8
            TextField { id: input; placeholderText: "message…"; width: 300 }
            Button {
                text: "Publish"
                onClicked: { pub.data = input.text; pub.publish() }
            }
        }
        Label { text: "received: " + (sub.data || "—") + (sub.isStale ? "  [stale]" : "") }
    }
}
