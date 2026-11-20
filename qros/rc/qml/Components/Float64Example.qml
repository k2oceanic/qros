import QtQuick 2.15
import QtQuick.Controls 2.15
import QRos 1.0

// Publishes std_msgs/Float64 via a slider, subscribes on the same topic.
// Move the slider then leave it alone — isStale trips after 2 seconds of silence.
GroupBox {
    id: root
    title: "Float64 Pub/Sub — /qml/value  (staleTimeout: 2 s)"
    property var node

    QRosDoublePublisher  { id: pub; node: root.node; topic: "/qml/value" }
    QRosDoubleSubscriber { id: sub; node: root.node; topic: "/qml/value"; staleTimeout: 2.0 }

    Column {
        spacing: 6
        Row {
            spacing: 8
            Slider {
                id: slider
                from: -100; to: 100; width: 300
                onValueChanged: { pub.data = value; pub.publish() }
            }
            Label { text: slider.value.toFixed(2); anchors.verticalCenter: parent.verticalCenter }
        }
        Label { text: "received: " + sub.data.toFixed(3) + (sub.isStale ? "  [stale]" : "") }
    }
}
