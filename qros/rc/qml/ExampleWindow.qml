import QtQuick 2.15
import QtQuick.Controls 2.15
import "Components"

ApplicationWindow {
    visible: true
    width: 1000
    height: 800
    title: "qros Example"

    ScrollView {
        anchors.fill: parent
        clip: true

        Column {
            x: 12; y: 12
            width: parent.width - 24
            spacing: 10

            StringExample    { width: parent.width; node: applicationNode }
            Float64Example   { width: parent.width; node: applicationNode }
            BoolExample      { width: parent.width; node: applicationNode }
            LatchedExample   { width: parent.width; node: applicationNode }
            ServiceExample   { width: parent.width; node: applicationNode }
            ParametersExample { width: parent.width; node: applicationNode }
            TFDemoPanel      { width: parent.width; node: applicationNode }
        }
    }
}
