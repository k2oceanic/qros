import QtQuick 2.15
import QtQuick.Controls 2.15

Slider {
    id: slider
    width: 300
    height: 50
    // from: 0
    // to: 100
    value: 50
    property string text: "text"

    onPressedChanged: {
        if (pressed) {
            handleOpacityAnimation.stop()
            handleOpacityAnimation.duration = 50
            handleOpacityAnimation.to = 1; // Target opacity when pressed
            handleOpacityAnimation.start();
        } else {
            // Delay the start of the fade-out animation until the handle is released
            handleOpacityAnimation.stop()
            handleOpacityAnimation.duration = 1000
            handleOpacityAnimation.to = 0.75; // Target opacity when released
            handleOpacityAnimation.start();
        }
    }

    // Increase the touch area
    handle: Rectangle {
        id: handleRect
        x: slider.position * (slider.width - width)
        y: (slider.height - height) / 2
        width: parent.height  // Larger width for easier touch
        height: parent.height  // Larger height for easier touch
        color: "#2196f3"
        radius: 15
        border.width: 0
        opacity: 0.75

        Label {
            text: slider.value.toFixed(1)
            font.pointSize: handleRect.width * .33
            anchors.centerIn: parent
            color: "#ffffff"
            visible: slider.pressed
        }
    }

    // Customizing the slider track
    background: Rectangle {
        id: rectangle
        x: 0
        y: slider.topPadding + (slider.availableHeight - height) / 2
        width: parent.width
        height: parent.height
        radius: 15
        color: Qt.rgba(0, 0, 0, 0.25)
        Text {
            id: labelText
            color: "#ffffff"
            text: slider.text
            anchors.verticalCenter: parent.verticalCenter
            //font.styleName: "Bold"
            font.pointSize: parent.height / 3
            anchors.horizontalCenter: parent.horizontalCenter
        }
    }

    // Opacity animation for the handle
    NumberAnimation {
        id: handleOpacityAnimation
        target: handleRect // Target the handle rectangle for animation
        property: "opacity" // Animate the opacity property
        duration: 1000 // Duration of the animation in milliseconds, set to 1 second
        easing.type: Easing.InOutQuad // Easing function for a smooth transition
    }
}
