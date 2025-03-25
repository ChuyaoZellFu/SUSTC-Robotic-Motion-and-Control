// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerState = require('./ControllerState.js');
let ControllerElementState = require('./ControllerElementState.js');

//-----------------------------------------------------------

class ControllerNotification_state {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controller_state = null;
      this.controller_element = null;
    }
    else {
      if (initObj.hasOwnProperty('controller_state')) {
        this.controller_state = initObj.controller_state
      }
      else {
        this.controller_state = [];
      }
      if (initObj.hasOwnProperty('controller_element')) {
        this.controller_element = initObj.controller_element
      }
      else {
        this.controller_element = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerNotification_state
    // Serialize message field [controller_state]
    // Serialize the length for message field [controller_state]
    bufferOffset = _serializer.uint32(obj.controller_state.length, buffer, bufferOffset);
    obj.controller_state.forEach((val) => {
      bufferOffset = ControllerState.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [controller_element]
    // Serialize the length for message field [controller_element]
    bufferOffset = _serializer.uint32(obj.controller_element.length, buffer, bufferOffset);
    obj.controller_element.forEach((val) => {
      bufferOffset = ControllerElementState.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerNotification_state
    let len;
    let data = new ControllerNotification_state(null);
    // Deserialize message field [controller_state]
    // Deserialize array length for message field [controller_state]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controller_state = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controller_state[i] = ControllerState.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [controller_element]
    // Deserialize array length for message field [controller_element]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.controller_element = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.controller_element[i] = ControllerElementState.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.controller_state.length;
    object.controller_element.forEach((val) => {
      length += ControllerElementState.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerNotification_state';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f624e32135515547aa37c46203c9be9e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ControllerState[] controller_state
    ControllerElementState[] controller_element
    ================================================================================
    MSG: kortex_driver/ControllerState
    
    ControllerHandle handle
    uint32 event_type
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    ================================================================================
    MSG: kortex_driver/ControllerElementState
    
    ControllerElementHandle handle
    uint32 event_type
    float32 axis_value
    ================================================================================
    MSG: kortex_driver/ControllerElementHandle
    
    ControllerHandle controller_handle
    ControllerElementHandle_identifier oneof_identifier
    ================================================================================
    MSG: kortex_driver/ControllerElementHandle_identifier
    
    uint32[] button
    uint32[] axis
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControllerNotification_state(null);
    if (msg.controller_state !== undefined) {
      resolved.controller_state = new Array(msg.controller_state.length);
      for (let i = 0; i < resolved.controller_state.length; ++i) {
        resolved.controller_state[i] = ControllerState.Resolve(msg.controller_state[i]);
      }
    }
    else {
      resolved.controller_state = []
    }

    if (msg.controller_element !== undefined) {
      resolved.controller_element = new Array(msg.controller_element.length);
      for (let i = 0; i < resolved.controller_element.length; ++i) {
        resolved.controller_element[i] = ControllerElementState.Resolve(msg.controller_element[i]);
      }
    }
    else {
      resolved.controller_element = []
    }

    return resolved;
    }
};

module.exports = ControllerNotification_state;
