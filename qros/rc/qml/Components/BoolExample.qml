import QtQuick 2.15
import QtQuick.Controls 2.15
import QRos 1.0

// Publishes std_msgs/Bool via a switch, subscribes on the same topic.
GroupBox {
    id: root
    title: "Bool Pub/Sub — /qml/flag"
    property var node

    QRosBoolPublisher  { id: pub; node: root.node; topic: "/qml/flag" }
    QRosBoolSubscriber { id: sub; node: root.node; topic: "/qml/flag"; staleTimeout: 5.0 }

    Row {
        spacing: 12
        Switch {
            text: "publish"
            onCheckedChanged: { pub.data = checked; pub.publish() }
        }
        Label {
            text: "received: " + sub.data + (sub.isStale ? "  [stale]" : "")
            anchors.verticalCenter: parent.verticalCenter
        }
    }
}
