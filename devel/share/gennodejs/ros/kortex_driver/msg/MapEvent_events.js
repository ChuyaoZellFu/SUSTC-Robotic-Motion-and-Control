// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyEvent = require('./SafetyEvent.js');
let GpioEvent = require('./GpioEvent.js');
let ControllerEvent = require('./ControllerEvent.js');

//-----------------------------------------------------------

class MapEvent_events {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.safety_event = null;
      this.gpio_event = null;
      this.controller_event = null;
    }
    else {
      if (initObj.hasOwnProperty('safety_event')) {
        this.safety_event = initObj.safety_event
      }
      else {
        this.safety_event = [];
      }
      if (initObj.hasOwnProperty('gpio_event')) {
        this.gpio_event = initObj.gpio_event
      }
      else {
        this.gpio_event = [];
      }
      if (initObj.hasOwnProperty('controller_event')) {
        this.controller_event = initObj.controller_event
      }
      else {
        this.controller_event = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MapEvent_events
    // Serialize message field [safety_event]
    // Serialize the length for message field [safety_event]
    bufferOffset = _serializer.uint32(obj.safety_event.length, buffer, bufferOffset);
    obj.safety_event.forEach((val) => {
      bufferOffset = SafetyEvent.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [gpio_event]
    // Serialize the length for message field [gpio_event]
    bufferOffset = _serializer.uint32(obj.gpio_event.length, buffer, bufferOffset);
    obj.gpio_event.forEach((val) => {
      bufferOffset = GpioEvent.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [controller_event]
    // Serialize the length for message field [controller_event]
    bufferOffset = _serializer.uint32(obj.controller_event.length, buffer, bufferOffset);
    obj.controller_event.forEach((val) => {
      bufferOffset = ControllerEvent.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MapEvent_events
    let len;
    let data = new MapEvent_events(null);
    // Deserialize message field [safety_event]
    // Deserialize array length for message field [safety_event]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.safety_event = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.safety_event[i] = SafetyEvent.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [gpio_event]
    // Deserialize array length for message field [gpio_event]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gpio_event = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gpio_event[i] = GpioEvent.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [controller_event]
    // Deserialize array length for message field [controller_event]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controller_event = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controller_event[i] = ControllerEvent.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.safety_event.length;
    length += 12 * object.gpio_event.length;
    length += 12 * object.controller_event.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MapEvent_events';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '040caa44c991691249bac9565abbf70c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyEvent[] safety_event
    GpioEvent[] gpio_event
    ControllerEvent[] controller_event
    ================================================================================
    MSG: kortex_driver/SafetyEvent
    
    SafetyHandle safety_handle
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/GpioEvent
    
    uint32 input_type
    uint32 behavior
    uint32 input_identifier
    ================================================================================
    MSG: kortex_driver/ControllerEvent
    
    uint32 input_type
    uint32 behavior
    uint32 input_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MapEvent_events(null);
    if (msg.safety_event !== undefined) {
      resolved.safety_event = new Array(msg.safety_event.length);
      for (let i = 0; i < resolved.safety_event.length; ++i) {
        resolved.safety_event[i] = SafetyEvent.Resolve(msg.safety_event[i]);
      }
    }
    else {
      resolved.safety_event = []
    }

    if (msg.gpio_event !== undefined) {
      resolved.gpio_event = new Array(msg.gpio_event.length);
      for (let i = 0; i < resolved.gpio_event.length; ++i) {
        resolved.gpio_event[i] = GpioEvent.Resolve(msg.gpio_event[i]);
      }
    }
    else {
      resolved.gpio_event = []
    }

    if (msg.controller_event !== undefined) {
      resolved.controller_event = new Array(msg.controller_event.length);
      for (let i = 0; i < resolved.controller_event.length; ++i) {
        resolved.controller_event[i] = ControllerEvent.Resolve(msg.controller_event[i]);
      }
    }
    else {
      resolved.controller_event = []
    }

    return resolved;
    }
};

module.exports = MapEvent_events;
