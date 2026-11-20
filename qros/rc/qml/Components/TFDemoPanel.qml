import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QRos 1.0

GroupBox {
    id: root
    title: "TF Demo (QRosTfBuffer)"

    // Hook up your QRosNode from outside
    property alias node: tf.node

    // Expose current transform to parents
    property QRosTransformStamped currentTF: QRosTransformStamped { }

    // Convenience props (bind or read from parent)
    property alias targetFrame: targetBox.currentText
    property alias sourceFrame: sourceBox.currentText
    property alias frames: tf.frames

    readonly property real rad2deg: 180 / Math.PI

    // Quaternion → RPY (radians)
    function rpyFromQuat(q) {
        // q: QQuaternion (x,y,z,scalar[w])
        var x = q.x, y = q.y, z = q.z, w = q.scalar
        // roll (x-axis)
        var sinr_cosp = 2 * (w * x + y * z)
        var cosr_cosp = 1 - 2 * (x * x + y * y)
        var roll = Math.atan2(sinr_cosp, cosr_cosp)
        // pitch (y-axis)
        var sinp = 2 * (w * y - z * x)
        var pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp)
        // yaw (z-axis)
        var siny_cosp = 2 * (w * z + x * y)
        var cosy_cosp = 1 - 2 * (y * y + z * z)
        var yaw = Math.atan2(siny_cosp, cosy_cosp)
        return { roll: roll, pitch: pitch, yaw: yaw }
    }

    // Internal TF buffer (no timeouts, no internal spinning)
    QRosTfBuffer {
        id: tf
        // node is set via alias above
        Component.onCompleted: refreshFrames()
    }

    ColumnLayout {
        anchors.margins: 8
        anchors.fill: parent
        spacing: 8

        RowLayout {
            spacing: 8
            Layout.fillWidth: true

            Label { text: "Target:"; Layout.alignment: Qt.AlignVCenter }
            ComboBox {
                id: targetBox
                Layout.preferredWidth: 200
                editable: true
                model: tf.frames
                Component.onCompleted: currentIndex = Math.max(0, model.indexOf("map"))
            }

            Label { text: "Source:"; Layout.alignment: Qt.AlignVCenter }
            ComboBox {
                id: sourceBox
                Layout.preferredWidth: 200
                editable: true
                model: tf.frames
                Component.onCompleted: {
                    var idx = model.indexOf("base_link")
                    currentIndex = idx >= 0 ? idx : 0
                }
            }

            Button {
                text: "Refresh Frames"
                onClicked: tf.refreshFrames()
            }

            CheckBox {
                id: autoPoll
                text: "Auto-poll (10 Hz)"
                checked: true
            }

            Button {
                text: "Lookup Now"
                onClicked: {
                    if (tf.canTransform(targetBox.currentText, sourceBox.currentText)) {
                        const T = tf.lookupTransform(targetBox.currentText, sourceBox.currentText, 0.0)
                        if (T && T.valid) root.currentTF = T
                    }
                }
            }
        }

        // Auto polling (non-blocking, zero timeout)
        Timer {
            interval: 100
            running: autoPoll.checked
            repeat: true
            onTriggered: {
                if (tf.canTransform(targetBox.currentText, sourceBox.currentText)) {
                    const T = tf.lookupTransform(targetBox.currentText, sourceBox.currentText, 0.0)
                    if (T && T.valid) root.currentTF = T
                }
            }
        }

        // Status row
        RowLayout {
            spacing: 12
            Label {
                text: tf.frames.length > 0 ? "Frames: " + tf.frames.length : "Frames: —"
                color: tf.frames.length > 0 ? "green" : "tomato"
            }
            Label {
                text: (tf.canTransform(targetBox.currentText, sourceBox.currentText))
                      ? "✓ canTransform" : "✗ no transform"
                color: (tf.canTransform(targetBox.currentText, sourceBox.currentText))
                       ? "green" : "tomato"
            }
        }

        // Transform readout
        GridLayout {
            columns: 2
            rowSpacing: 4
            columnSpacing: 10
            Layout.fillWidth: true

            Label { text: "frame_id:" }
            Label { text: (root.currentTF && root.currentTF.valid) ? root.currentTF.frameId : "-" }

            Label { text: "child_frame_id:" }
            Label { text: (root.currentTF && root.currentTF.valid) ? root.currentTF.childFrameId : "-" }

            Label { text: "stamp (sec):" }
            Label { text: (root.currentTF && root.currentTF.valid) ? root.currentTF.stampSec.toFixed(3) : "-" }

            Label { text: "translation (x,y,z):" }
            Label {
                text: (root.currentTF && root.currentTF.valid)
                      ? root.currentTF.translation.x.toFixed(3) + ", "
                        + root.currentTF.translation.y.toFixed(3) + ", "
                        + root.currentTF.translation.z.toFixed(3)
                      : "-"
            }

            Label { text: "rotation (x,y,z,w):" }
            Label {
                text: (root.currentTF && root.currentTF.valid)
                      ? root.currentTF.rotation.x.toFixed(3) + ", "
                        + root.currentTF.rotation.y.toFixed(3) + ", "
                        + root.currentTF.rotation.z.toFixed(3) + ", "
                        + root.currentTF.rotation.scalar.toFixed(3)
                      : "-"
            }

            // NEW: Roll, Pitch, Yaw (deg)
            Label { text: "r,p,y (deg):" }
            Label {
                readonly property var rpy: (root.currentTF && root.currentTF.valid)
                    ? rpyFromQuat(root.currentTF.rotation) : null
                text: rpy
                      ? (rpy.roll  * root.rad2deg).toFixed(1) + ", "
                        + (rpy.pitch * root.rad2deg).toFixed(1) + ", "
                        + (rpy.yaw   * root.rad2deg).toFixed(1)
                      : "-"
            }
        }
    }
}
