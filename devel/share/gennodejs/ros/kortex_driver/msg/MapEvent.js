// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MapEvent_events = require('./MapEvent_events.js');

//-----------------------------------------------------------

class MapEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.oneof_events = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('oneof_events')) {
        this.oneof_events = initObj.oneof_events
      }
      else {
        this.oneof_events = new MapEvent_events();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MapEvent
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [oneof_events]
    bufferOffset = MapEvent_events.serialize(obj.oneof_events, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MapEvent
    let len;
    let data = new MapEvent(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [oneof_events]
    data.oneof_events = MapEvent_events.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += MapEvent_events.getMessageSize(object.oneof_events);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MapEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2332688b9b21902d710a4725da3d8c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string name
    MapEvent_events oneof_events
    ================================================================================
    MSG: kortex_driver/MapEvent_events
    
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
    const resolved = new MapEvent(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.oneof_events !== undefined) {
      resolved.oneof_events = MapEvent_events.Resolve(msg.oneof_events)
    }
    else {
      resolved.oneof_events = new MapEvent_events()
    }

    return resolved;
    }
};

module.exports = MapEvent;
