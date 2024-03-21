import QtQuick 2.15
import "../Components"
import OdysseusUi 1.0

Item {

    function mySlot() {

        if (subscriber === undefined) {
            console.log("subscriber not defined");
        } else {
            display.currentValue = root.subscriber.getPressure() * 1.450377e-4;
        }
    }

    Connections {
        target: root.subscriber
        onMsgReceived: mySlot()
    }

}
