// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');

//-----------------------------------------------------------

class SensorFocusAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
      this.focus_action = null;
      this.oneof_action_parameters = null;
    }
    else {
      if (initObj.hasOwnProperty('sensor')) {
        this.sensor = initObj.sensor
      }
      else {
        this.sensor = 0;
      }
      if (initObj.hasOwnProperty('focus_action')) {
        this.focus_action = initObj.focus_action
      }
      else {
        this.focus_action = 0;
      }
      if (initObj.hasOwnProperty('oneof_action_parameters')) {
        this.oneof_action_parameters = initObj.oneof_action_parameters
      }
      else {
        this.oneof_action_parameters = new SensorFocusAction_action_parameters();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorFocusAction
    // Serialize message field [sensor]
    bufferOffset = _serializer.uint32(obj.sensor, buffer, bufferOffset);
    // Serialize message field [focus_action]
    bufferOffset = _serializer.uint32(obj.focus_action, buffer, bufferOffset);
    // Serialize message field [oneof_action_parameters]
    bufferOffset = SensorFocusAction_action_parameters.serialize(obj.oneof_action_parameters, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorFocusAction
    let len;
    let data = new SensorFocusAction(null);
    // Deserialize message field [sensor]
    data.sensor = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [focus_action]
    data.focus_action = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [oneof_action_parameters]
    data.oneof_action_parameters = SensorFocusAction_action_parameters.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += SensorFocusAction_action_parameters.getMessageSize(object.oneof_action_parameters);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SensorFocusAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cae3081e3d5f90fbdd9351620859003c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sensor
    uint32 focus_action
    SensorFocusAction_action_parameters oneof_action_parameters
    ================================================================================
    MSG: kortex_driver/SensorFocusAction_action_parameters
    
    FocusPoint[] focus_point
    ManualFocus[] manual_focus
    ================================================================================
    MSG: kortex_driver/FocusPoint
    
    uint32 x
    uint32 y
    ================================================================================
    MSG: kortex_driver/ManualFocus
    
    uint32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorFocusAction(null);
    if (msg.sensor !== undefined) {
      resolved.sensor = msg.sensor;
    }
    else {
      resolved.sensor = 0
    }

    if (msg.focus_action !== undefined) {
      resolved.focus_action = msg.focus_action;
    }
    else {
      resolved.focus_action = 0
    }

    if (msg.oneof_action_parameters !== undefined) {
      resolved.oneof_action_parameters = SensorFocusAction_action_parameters.Resolve(msg.oneof_action_parameters)
    }
    else {
      resolved.oneof_action_parameters = new SensorFocusAction_action_parameters()
    }

    return resolved;
    }
};

module.exports = SensorFocusAction;
