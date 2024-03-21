import QtQuick 2.15
import "../Components"
import OdysseusUi 1.0

Item {
    id: root
    property var subscriber: undefined
    property string label: "Card Temp"

    width: 200
    height: 100

    IndicatorArray{
        id: indicators
    }

    function rosUpdate() {

        if (subscriber === undefined) {
            console.log("subscriber not defined");
        } else {
            display.currentValue = root.subscriber.getTemp()
        }
    }

    Connections {
        target: root.subscriber
        function onMsgReceived(){mySlot()}
    }

}
