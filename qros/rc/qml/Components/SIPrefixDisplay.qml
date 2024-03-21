import QtQuick 2.0

Text {
    property real numericValue: 0.0 // The numeric value to be displayed
    property string unit: "" // The unit of the numeric value
    property int decimalPlaces: 1

    // Function to determine the appropriate SI prefix
    function calculateSIPrefix(value) {
        const prefixes = [
            {threshold: 1e18, prefix: "E"}, // Exa
            {threshold: 1e15, prefix: "P"}, // Peta
            {threshold: 1e12, prefix: "T"}, // Tera
            {threshold: 1e9, prefix: "G"},  // Giga
            {threshold: 1e6, prefix: "M"},  // Mega
            {threshold: 1e3, prefix: "k"},  // Kilo
            {threshold: 1, prefix: ""},     // No prefix
            {threshold: 1e-3, prefix: "m"}, // Milli
            {threshold: 1e-6, prefix: "μ"}, // Micro
            {threshold: 1e-9, prefix: "n"}, // Nano
            {threshold: 1e-12, prefix: "p"},// Pico
            {threshold: 1e-15, prefix: "f"},// Femto
            {threshold: 1e-18, prefix: "a"} // Atto
        ];

        for (let i = 0; i < prefixes.length; i++) {
            if (Math.abs(value) >= prefixes[i].threshold) {
                let adjustedValue = value / prefixes[i].threshold;
                return adjustedValue.toFixed(decimalPlaces) + " " + prefixes[i].prefix + unit;
            }
        }
        return value.toString() + " " + unit; // Fallback for very small numbers
    }

    // Update the text display whenever the value or unit changes
    Component.onCompleted: updateText()
    onNumericValueChanged: updateText()
    onUnitChanged: updateText()

    function updateText() {
        text = calculateSIPrefix(numericValue)
    }
}
