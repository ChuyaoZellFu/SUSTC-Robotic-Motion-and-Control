// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FocusPoint = require('./FocusPoint.js');
let ManualFocus = require('./ManualFocus.js');

//-----------------------------------------------------------

class SensorFocusAction_action_parameters {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.focus_point = null;
      this.manual_focus = null;
    }
    else {
      if (initObj.hasOwnProperty('focus_point')) {
        this.focus_point = initObj.focus_point
      }
      else {
        this.focus_point = [];
      }
      if (initObj.hasOwnProperty('manual_focus')) {
        this.manual_focus = initObj.manual_focus
      }
      else {
        this.manual_focus = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorFocusAction_action_parameters
    // Serialize message field [focus_point]
    // Serialize the length for message field [focus_point]
    bufferOffset = _serializer.uint32(obj.focus_point.length, buffer, bufferOffset);
    obj.focus_point.forEach((val) => {
      bufferOffset = FocusPoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [manual_focus]
    // Serialize the length for message field [manual_focus]
    bufferOffset = _serializer.uint32(obj.manual_focus.length, buffer, bufferOffset);
    obj.manual_focus.forEach((val) => {
      bufferOffset = ManualFocus.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorFocusAction_action_parameters
    let len;
    let data = new SensorFocusAction_action_parameters(null);
    // Deserialize message field [focus_point]
    // Deserialize array length for message field [focus_point]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.focus_point = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.focus_point[i] = FocusPoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [manual_focus]
    // Deserialize array length for message field [manual_focus]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.manual_focus = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.manual_focus[i] = ManualFocus.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.focus_point.length;
    length += 4 * object.manual_focus.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SensorFocusAction_action_parameters';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6e07eb3126a69325e87d9841916b8dc3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new SensorFocusAction_action_parameters(null);
    if (msg.focus_point !== undefined) {
      resolved.focus_point = new Array(msg.focus_point.length);
      for (let i = 0; i < resolved.focus_point.length; ++i) {
        resolved.focus_point[i] = FocusPoint.Resolve(msg.focus_point[i]);
      }
    }
    else {
      resolved.focus_point = []
    }

    if (msg.manual_focus !== undefined) {
      resolved.manual_focus = new Array(msg.manual_focus.length);
      for (let i = 0; i < resolved.manual_focus.length; ++i) {
        resolved.manual_focus[i] = ManualFocus.Resolve(msg.manual_focus[i]);
      }
    }
    else {
      resolved.manual_focus = []
    }

    return resolved;
    }
};

module.exports = SensorFocusAction_action_parameters;
