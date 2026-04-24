# qros

A Qt/QML bridge library for ROS 2. Exposes ROS publishers, subscribers, service clients, TF2, parameters, and diagnostics as native QML types so you can build reactive ROS GUIs entirely in QML with no per-widget C++ boilerplate.

---

## Table of Contents

- [Overview](#overview)
- [Dependencies](#dependencies)
- [Integration](#integration)
- [Quick Start](#quick-start)
- [Core Types](#core-types)
  - [QRosNode](#qrosnode)
  - [QRosParameterEvent](#qrosparameterevent)
  - [QRosParameterClient](#qrosparameterclient)
- [Publishers & Subscribers](#publishers--subscribers)
  - [Base Properties](#base-properties)
  - [std\_msgs](#std_msgs)
  - [sensor\_msgs](#sensor_msgs)
  - [geometry\_msgs](#geometry_msgs)
  - [nav\_msgs](#nav_msgs)
  - [diagnostic\_msgs](#diagnostic_msgs)
  - [Custom: Propulsion & Hydraulics](#custom-propulsion--hydraulics)
  - [Custom: I/O](#custom-io)
- [Service Clients](#service-clients)
- [TF2](#tf2)
- [Diagnostics Updater](#diagnostics-updater)
- [Dynamic Variants](#dynamic-variants)
- [Utilities](#utilities)
- [Standalone Nodes](#standalone-nodes)
- [Design Patterns](#design-patterns)

---

## Overview

qros wraps `rclcpp` publisher, subscriber, service client, TF2, and parameter primitives as `QObject` subclasses registered with the QML type system under the `QRos 1.0` import. The result is that a ROS subscriber looks like this in QML:

```qml
import QRos 1.0

QRosDoubleSubscriber {
    node:  applicationNode
    topic: "/sensors/depth"
    onMsgReceived: depthLabel.text = value.toFixed(2) + " m"
}
```

No C++ required per widget. All message data is exposed as typed QML properties with `NOTIFY` signals so bindings and `onXChanged` handlers work naturally.

---

## Dependencies

| Category | Packages |
|---|---|
| ROS 2 | `rclcpp`, `rcl_interfaces`, `std_srvs`, `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `tf2_sensor_msgs` |
| ROS msgs | `std_msgs`, `sensor_msgs`, `diagnostic_msgs`, `nav_msgs`, `geometry_msgs`, `geographic_msgs` |
| Custom msgs | `qros_interfaces`, `hydraulic_interfaces`, `io_interfaces`, `propulsion_interfaces` |
| Qt 6 | `Qt6Core`, `Qt6Gui`, `Qt6Qml`, `Qt6Quick` |

---

## Integration

### CMake

```cmake
find_package(qros REQUIRED)
target_link_libraries(my_target qros::qros)
```

### C++ Setup

Register all QML types once before loading your QML engine, then expose the node as a context property:

```cpp
#include "qros/qros.h"

qros::registerQmlTypes();   // registers all 46+ types as "QRos 1.0"

QQmlApplicationEngine engine;

auto ros_node = std::make_shared<rclcpp::Node>("my_app");
QRosNode applicationNode;
applicationNode.setNodePtr(ros_node);
applicationNode.spinRosWithTimer(10);   // 10 ms Qt timer drives rclcpp::spin_some

engine.rootContext()->setContextProperty("applicationNode", &applicationNode);
engine.load(url);
```

### QML Import

```qml
import QRos 1.0
```

All types are available under this single import. No per-type imports needed.

---

## Quick Start

```qml
import QtQuick 2.15
import QRos 1.0

Item {
    // Subscribe to a topic
    QRosDoubleSubscriber {
        node:  applicationNode
        topic: "/vehicle/speed"
        onMsgReceived: speedText.text = value.toFixed(1)
    }

    // Publish a value
    QRosBoolPublisher {
        id:    lightPub
        node:  applicationNode
        topic: "/lights/enable"
    }

    // Watch an external parameter
    QRosParameterClient {
        node:         applicationNode
        watchedNode:  "/thruster_driver"
        watchedParam: "max_rpm"
        onValueChanged: console.log("max_rpm =", value)
    }

    // Call a service
    QRosTriggerServiceClient {
        id:          resetSrv
        node:        applicationNode
        serviceName: "/system/reset"
        onResponseReceived: console.log("Reset:", respMessage)
    }

    Text  { id: speedText }
    Button { text: "Lights On";  onClicked: { lightPub.value = true;  lightPub.publish() } }
    Button { text: "Reset";      onClicked: resetSrv.callService() }
}
```

---

## Core Types

### QRosNode

The application's ROS node wrapper. Always created in C++ and passed to QML as the `applicationNode` context property. Every publisher, subscriber, and service client takes a `node: applicationNode` property.

**C++ methods (call before loading QML):**

| Method | Description |
|---|---|
| `setNodePtr(rclcpp::Node::SharedPtr)` | Attach an existing rclcpp node |
| `spinRosWithTimer(int ms = 10)` | Integrate ROS spinning into Qt event loop |

**QML-callable slots:**

| Slot | Returns | Description |
|---|---|---|
| `getTopics()` | `QStringList` | All active topics |
| `getTopicsOfType(type)` | `QStringList` | Topics matching a message type string |
| `getServices()` | `QStringList` | All active services |
| `getNodeNames()` | `QStringList` | All active node names |
| `getName()` | `QString` | This node's name |
| `getNamespace()` | `QString` | This node's namespace |
| `declareParameter(name, default)` | — | Declare a node parameter |
| `getParameters()` | `QVariantMap` | All current parameter values |
| `setParameter(name, value)` | — | Set a local parameter |
| `setExternalParameterAsync(node, param, value, wait_ms)` | — | Set a parameter on another node |
| `getExternalParametersAsync(node, params, wait_ms)` | — | Fetch parameters from another node |
| `countSubscribers(topic)` | `int` | Number of subscribers on a topic |
| `countPublishers(topic)` | `int` | Number of publishers on a topic |

**Properties:**

| Property | Type | Description |
|---|---|---|
| `parameters` | `QVariantMap` | Current node parameter map (read-only) |
| `parameterEvent` | `QRosParameterEvent*` | Live parameter event stream |

**Signals:** `parametersChanged()`, `nodeChanged()`, `parameterSetResult(bool, node, param)`, `parametersGetResult(bool, node, params, error)`

---

### QRosParameterEvent

Listens to `/parameter_events` and fires Qt signals for any parameter change across all nodes. Accessed via `applicationNode.parameterEvent`.

```qml
Connections {
    target: applicationNode.parameterEvent
    function onEvent(nodeName, paramName, value) {
        console.log(nodeName, paramName, "=", value)
    }
}
```

**Signals:**

| Signal | Arguments | Description |
|---|---|---|
| `event` | `node, param, value` | Any parameter changed |
| `newParam` | `node, param, value` | New parameter declared |
| `deleted` | `node, param` | Parameter removed |

---

### QRosParameterClient

Watches and optionally syncs a single named parameter on an external node. Useful for mirroring driver configuration into the UI and writing it back when the driver restarts.

```qml
QRosParameterClient {
    node:         applicationNode
    watchedNode:  "/ixys_driver"
    watchedParam: "node_1.trip_level_amps"
    override:     true        // re-push cached value when node restarts
    onValueChanged: tripSlider.value = value
}
```

**Properties:**

| Property | Type | Description |
|---|---|---|
| `watchedNode` | `QString` | Fully-qualified node name to watch |
| `watchedParam` | `QString` | Parameter name on that node |
| `value` | `QVariant` | Current parameter value (read-only) |
| `available` | `bool` | Whether the node is reachable |
| `override` | `bool` | Push cached value back on node restart (default `true`) |

**Slots:** `set(value)` — write a new value to the external node.

**Signals:** `valueChanged()`, `availableChanged()`, `newParam(value)`

---

## Publishers & Subscribers

### Base Properties

All publishers and subscribers share these properties from their base classes.

**Publishers** (`QRosPublisher`):

| Property | Type | Default | Description |
|---|---|---|---|
| `node` | `QRosNode*` | — | The application node (required) |
| `topic` | `QString` | `""` | Topic to publish on |
| `queueSize` | `int` | `10` | QoS history depth |
| `latched` | `bool` | `false` | Transient-local (latched) QoS |

**Slot:** `publish()` — send the current buffered message.

**Subscribers** (`QRosSubscriber`):

| Property | Type | Default | Description |
|---|---|---|---|
| `node` | `QRosNode*` | — | The application node (required) |
| `topic` | `QString` | `""` | Topic to subscribe to |
| `queueSize` | `int` | `1` | QoS history depth |
| `latched` | `bool` | `false` | Transient-local (latched) QoS |
| `staleTimeout` | `double` | `0` | Seconds before `isStale` becomes true (0 = disabled) |
| `isStale` | `bool` | `false` | No message received within `staleTimeout` |

**Signal:** `msgReceived()` — fires on every new message. Use `on`-handler in QML.

---

### std_msgs

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosStringPublisher` / `QRosStringSubscriber` | `std_msgs/String` | `value` (QString) |
| `QRosBoolPublisher` / `QRosBoolSubscriber` | `std_msgs/Bool` | `value` (bool) |
| `QRosIntPublisher` / `QRosIntSubscriber` | `std_msgs/Int32` | `value` (int) |
| `QRosInt64Publisher` / `QRosInt64Subscriber` | `std_msgs/Int64` | `value` (int) |
| `QRosDoublePublisher` / `QRosDoubleSubscriber` | `std_msgs/Float64` | `value` (double) |
| `QRosFloat32Publisher` / `QRosFloat32Subscriber` | `std_msgs/Float32` | `value` (real) |
| `QRosFloat32MultiArrayPublisher` / `QRosFloat32MultiArraySubscriber` | `std_msgs/Float32MultiArray` | `values` (QList\<double\>) |

```qml
QRosDoubleSubscriber {
    node: applicationNode
    topic: "/depth"
    onMsgReceived: depthGauge.value = value
}
```

---

### sensor_msgs

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosTemperatureSubscriber` | `sensor_msgs/Temperature` | `temperature` (double), `variance` (double) |
| `QRosFluidPressureSubscriber` | `sensor_msgs/FluidPressure` | `fluidPressure` (double), `variance` (double) |
| `QRosJoyPublisher` / `QRosJoySubscriber` | `sensor_msgs/Joy` | `axes` (QList\<double\>), `buttons` (QList\<int\>) |
| `QRosJointStatePublisher` / `QRosJointStateSubscriber` | `sensor_msgs/JointState` | `names` (QStringList), `positions`, `velocities`, `efforts` (QList\<double\>) |
| `QRosImuPublisher` / `QRosImuSubscriber` | `sensor_msgs/Imu` | `orientation` (QQuaternion), `angularVelocity`, `linearAcceleration` (QVector3D) |
| `QRosNavSatFixPublisher` / `QRosNavSatFixSubscriber` | `sensor_msgs/NavSatFix` | `latitude`, `longitude`, `altitude` (double), `status` (int), `positionCovariance` (QList\<double\>) |
| `QRosRangePublisher` / `QRosRangeSubscriber` | `sensor_msgs/Range` | `range`, `fieldOfView`, `minRange`, `maxRange` (double) |

```qml
QRosImuSubscriber {
    node: applicationNode
    topic: "/imu/data"
    onMsgReceived: {
        headingText.text = orientation.toEulerAngles().z.toFixed(1) + "°"
    }
}
```

---

### geometry_msgs

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosPoseStampedPublisher` / `QRosPoseStampedSubscriber` | `geometry_msgs/PoseStamped` | `position` (QVector3D), `orientation` (QQuaternion), `frameId` (QString) |
| `QRosTwistStampedPublisher` / `QRosTwistStampedSubscriber` | `geometry_msgs/TwistStamped` | `linear`, `angular` (QVector3D), `frameId` (QString) |
| `QRosPointStampedPublisher` / `QRosPointStampedSubscriber` | `geometry_msgs/PointStamped` | `position` (QVector3D), `frameId` (QString) |
| `QRosWrenchStampedPublisher` / `QRosWrenchStampedSubscriber` | `geometry_msgs/WrenchStamped` | `force`, `torque` (QVector3D), `frameId` (QString) |
| `QRosGeoPointPublisher` / `QRosGeoPointSubscriber` | `geographic_msgs/GeoPoint` | `latitude`, `longitude`, `altitude` (double) |

---

### nav_msgs

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosOdometryPublisher` / `QRosOdometrySubscriber` | `nav_msgs/Odometry` | `position` (QVector3D), `orientation` (QQuaternion), `linearVelocity`, `angularVelocity` (QVector3D), `frameId`, `childFrameId` (QString) |

---

### diagnostic_msgs

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosDiagnosticStatusPublisher` / `QRosDiagnosticStatusSubscriber` | `diagnostic_msgs/DiagnosticStatus` | `level` (int), `name`, `message`, `hardwareId` (QString), `values` (QVariantMap) |
| `QRosDiagnosticArraySubscriber` | `diagnostic_msgs/DiagnosticArray` | `status` (QVariantList of maps) |

For publishing live diagnostic status from QML components, prefer the [Diagnostics Updater](#diagnostics-updater) pattern below over using `QRosDiagnosticStatusPublisher` directly.

---

### Custom: Propulsion & Hydraulics

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosThrustStampedPublisher` / `QRosThrustStampedSubscriber` | `propulsion_interfaces/ThrustStamped` | `scale`, `proportionalValue` (double) |
| `QRosThrustArrayPublisher` / `QRosThrustArraySubscriber` | `propulsion_interfaces/ThrustArray` | `scales`, `proportionalValues` (QList\<double\>) |
| `QRosValvePublisher` | `hydraulic_interfaces/Valve` | `valveId` (int), `setPoint` (double) |
| `QRosValveStampedPublisher` | `hydraulic_interfaces/ValveStamped` | `valveId` (int), `setPoint` (double) |
| `QRosValvePackPublisher` / `QRosValvePackSubscriber` | `hydraulic_interfaces/ValvePack` | Array of valve ID / setpoint pairs |

---

### Custom: I/O

| Type | ROS Message | Key Properties |
|---|---|---|
| `QRosRawPacketPublisher` / `QRosRawPacketSubscriber` | `io_interfaces/RawPacket` | Raw byte payload, viewable as ASCII or hex string |
| `QRosRawAnalogSubscriber` | `io_interfaces/RawAnalogArray` | `channelIds` (QList\<int\>), `scales`, `proportionalValues`, `scaledValues` (QList\<double\>) |
| `QRosRawDigitalArrayPublisher` / `QRosRawDigitalArraySubscriber` | `io_interfaces/RawDigitalArray` | `channelIds` (QList\<int\>), `states` (QList\<bool\>) |

```qml
QRosRawAnalogSubscriber {
    id:    analogSub
    node:  applicationNode
    topic: "/io_card/analog"
    onAnalogChanged: (channelId, scale, proportionalValue, voltage, type) => {
        if (channelId === 3) pressureDisplay.value = voltage * scale
    }
}
```

---

## Service Clients

| Type | ROS Service | Notes |
|---|---|---|
| `QRosTriggerServiceClient` | `std_srvs/Trigger` | `respSuccess` (bool), `respMessage` (QString) on `responseReceived` |
| `QRosChannelTriggerClient` | `io_interfaces/ChannelTrigger` | `callChannelTrigger(channel_id)` slot |

```qml
QRosTriggerServiceClient {
    node:        applicationNode
    serviceName: "/arm_system"
    onResponseReceived: statusText.text = respSuccess ? "Armed" : "Failed: " + respMessage
}
```

All service clients call synchronously with a 1-second timeout. The call is non-blocking from QML — `responseReceived` fires when complete.

---

## TF2

### QRosTfBuffer

Listens to the TF2 tree and provides frame lookup from QML.

```qml
QRosTfBuffer {
    id: tfBuf
    node: applicationNode
    Component.onCompleted: refreshFrames()
}

Button {
    text: "Get vehicle pose"
    onClicked: {
        var tf = tfBuf.lookupTransform("map", "base_link")
        if (tf.valid) {
            posLabel.text = "x=%1 y=%2".arg(tf.translation.x.toFixed(2))
                                        .arg(tf.translation.y.toFixed(2))
        }
    }
}
```

**Properties:**

| Property | Type | Description |
|---|---|---|
| `frames` | `QStringList` | All known TF frame names (call `refreshFrames()` to update) |

**Methods:**

| Method | Returns | Description |
|---|---|---|
| `canTransform(target, source)` | `bool` | Check if transform is available |
| `lookupTransform(target, source, time_sec=0)` | `QRosTransformStamped*` | Get transform at time (0 = latest) |
| `lookupTransformFull(target, t_time, source, s_time, fixed)` | `QRosTransformStamped*` | Full extrapolation lookup |
| `frameExists(frame)` | `bool` | Check if a specific frame is known |
| `refreshFrames()` | — | Update the `frames` list |

### QRosTransformStamped

Returned by `lookupTransform`. All properties have `NOTIFY` signals.

| Property | Type | Description |
|---|---|---|
| `valid` | `bool` | Transform was found and is valid |
| `frameId`, `childFrameId` | `QString` | Frame names |
| `stampSec` | `double` | Timestamp as seconds since epoch |
| `stamp` | `QDateTime` | Timestamp as Qt date-time |
| `translation` | `QVector3D` | Position |
| `rotation` | `QQuaternion` | Orientation |
| `matrix` | `QMatrix4x4` | Combined 4×4 transformation matrix |

---

## Diagnostics Updater

A pattern for aggregating health status from many QML components into a single `/diagnostics` stream without any per-component C++ code.

### Setup (C++)

```cpp
#include "qros/qros_diagnostics_updater.h"

qros::QRosDiagnosticsUpdater* diagUpdater = new qros::QRosDiagnosticsUpdater(&engine);
diagUpdater->setup(ros_node);   // hardwareId defaults to node's fully-qualified name
engine.rootContext()->setContextProperty("diagnosticsUpdater", diagUpdater);
```

**`QRosDiagnosticsUpdater` properties:**

| Property | Type | Default | Description |
|---|---|---|---|
| `hardwareId` | `QString` | node FQDN | Hardware ID stamped on all status messages |
| `period` | `double` | `1.0` | Publish interval in seconds |

### Usage (QML)

Place a `QRosDiagnosticTask` anywhere in the component tree. It automatically registers with `diagnosticsUpdater` when created and unregisters when destroyed — no wiring needed.

```qml
QRosDiagnosticTask {
    name:    "Hydraulic Pressure"
    level:   pressure < 50 ? QRosDiagnosticTask.ERROR
           : pressure < 80 ? QRosDiagnosticTask.WARN
           :                  QRosDiagnosticTask.OK
    message: "%1 bar".arg(pressure.toFixed(1))
    values:  ({ "raw_v": rawVoltage, "scaled_bar": pressure })
}
```

**`QRosDiagnosticTask` properties:**

| Property | Type | Description |
|---|---|---|
| `name` | `QString` | Task name shown in diagnostic viewer |
| `level` | `int` (enum) | `OK` (0), `WARN` (1), `ERROR` (2), `STALE` (3) |
| `message` | `QString` | Human-readable status string |
| `values` | `QVariantMap` | Key-value pairs published as `diagnostic_msgs/KeyValue[]` |

Tasks are driven entirely by QML property bindings — `level` and `message` update reactively as the values they depend on change. The updater reads all registered tasks at each publish interval.

---

## Dynamic Variants

For passing arbitrary structured data between ROS nodes using QVariant serialization.

| Type | ROS Message | Description |
|---|---|---|
| `QRosQVariantPublisher` / `QRosQVariantSubscriber` | `qros_interfaces/QVariant` | Single QVariant (any type) serialized as binary |
| `QRosQVariantMapPublisher` / `QRosQVariantMapSubscriber` | `qros_interfaces/QVariantMap` | Key-value map of QVariants serialized as binary |

Useful for passing configuration objects between UI nodes without defining custom message types.

---

## Utilities

### QRosSettings

Thin wrapper around `QSettings` for persisting UI configuration:

```cpp
// C++ construction
auto* settings = new QRosSettings("MyOrg", "MyApp", parent);

// Or register as context property and use from QML
engine.rootContext()->setContextProperty("appSettings", settings);
```

```qml
// QML usage
Component.onCompleted: mySlider.value = appSettings.getValue("gain", 1.0)
onValueChanged: appSettings.setValue("gain", mySlider.value)
```

**Methods:** `setValue(key, value)`, `getValue(key, default)` → QVariant

Stores to the platform's standard config location (e.g. `~/.config/MyOrg/MyApp.conf`).

---

## Standalone Nodes

### `hello_qros`

Minimal example showing how to bootstrap a qros QML application:

```bash
ros2 run qros hello_qros
```

Creates a node named `qml_example`, registers `applicationNode`, and loads `ExampleWindow.qml` from the package resources.

### `message_buffer`

Republishes topics with transient-local (latched) QoS. Useful for making a non-latched topic available to late-joining subscribers.

**Parameters:**

| Parameter | Type | Description |
|---|---|---|
| `input_topics` | `string[]` | Topics to subscribe to |
| `output_topics` | `string[]` | Topics to republish on |
| `message_types` | `string[]` | Message type strings (one per pair) |

Uses generic serialized message passing — no type-specific code, handles any message type.

---

## Design Patterns

### Node Injection

Every QML type takes a `node` property referencing the application's `QRosNode`. This enables multiple nodes in a single application and makes dependencies explicit.

### Template Specialization

All message types are implemented as thin template specializations over `QRosTypedPublisher<msg_T>` / `QRosTypedSubscriber<msg_T>`. Adding a new message type requires about 30 lines — a header that specializes the template and declares Qt properties for the fields you want to expose.

### QoS Adaptation

Subscribers automatically select QoS based on the `latched` property: best-effort volatile for streaming data, reliable transient-local for latched configuration topics. Publishers respect the same flag.

### Stale Detection

Any subscriber can set `staleTimeout` (seconds). If no message arrives within that window, `isStale` becomes `true` and `isStaleChanged()` fires. Set to `0` (default) to disable.

### Auto-Registration (Diagnostics)

`QRosDiagnosticTask` implements `QQmlParserStatus`. In `componentComplete()` it walks the QML context to find the `diagnosticsUpdater` context property and registers itself. In its destructor it unregisters. This means tasks created dynamically (e.g. inside a `Repeater`) appear and disappear from the diagnostics stream automatically with no manual connect/disconnect calls.

### Adding a New Message Type

1. Create `include/qros/qros_my_msg.h`
2. Specialize `QRosTypedPublisher<my_pkg::msg::MyMsg>` and/or `QRosTypedSubscriber<...>` as `QRosMy MsgPublisher` / `QRosMyMsgSubscriber`
3. Declare `Q_PROPERTY` for each field you want in QML
4. Override `onMsgReceived()` in the subscriber to emit your signals
5. Add `REGISTER_QML_TYPE(QRosMyMsgPublisher)` to `qros.cpp`
6. Include the header in `qros.h`
