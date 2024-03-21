import QtQuick 2.15
import QtQuick.Controls 2.15

Item {
    width: 400
    height: 400
    property real pitch: 10
    // Degrees
    property real roll: 10



    // Degrees

    Canvas {
        id: canvas
        anchors.fill: parent
        onPaint: {
            var ctx = getContext("2d")
            var width = canvas.width
            var height = canvas.height

            // Clear the canvas
            ctx.reset()
            ctx.clearRect(0, 0, width, height)

            // Calculate the horizon line based on pitch
            var horizonY = height / 2 + (pitch * 2); // Simple pitch representation

            // Draw sky
            ctx.fillStyle = "skyblue";
            ctx.fillRect(0, 0, width, horizonY);

            // Draw ground
            ctx.fillStyle = "sienna";
            ctx.fillRect(0, horizonY, width, height - horizonY);

            // Draw horizon line
            ctx.save();
            ctx.translate(width / 2, height / 2);
            ctx.rotate(roll * Math.PI / 180); // Convert degrees to radians
            ctx.strokeStyle = "white";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(-width / 2, 0);
            ctx.lineTo(width / 2, 0);
            ctx.stroke();
            ctx.restore();
        }
    }
}
