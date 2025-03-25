// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerElementHandle = require('./ControllerElementHandle.js');

//-----------------------------------------------------------

class ControllerElementState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.event_type = null;
      this.axis_value = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new ControllerElementHandle();
      }
      if (initObj.hasOwnProperty('event_type')) {
        this.event_type = initObj.event_type
      }
      else {
        this.event_type = 0;
      }
      if (initObj.hasOwnProperty('axis_value')) {
        this.axis_value = initObj.axis_value
      }
      else {
        this.axis_value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerElementState
    // Serialize message field [handle]
    bufferOffset = ControllerElementHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [event_type]
    bufferOffset = _serializer.uint32(obj.event_type, buffer, bufferOffset);
    // Serialize message field [axis_value]
    bufferOffset = _serializer.float32(obj.axis_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerElementState
    let len;
    let data = new ControllerElementState(null);
    // Deserialize message field [handle]
    data.handle = ControllerElementHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [event_type]
    data.event_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [axis_value]
    data.axis_value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControllerElementHandle.getMessageSize(object.handle);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerElementState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '522b5634710ce23afc015e933c822f1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ControllerElementHandle handle
    uint32 event_type
    float32 axis_value
    ================================================================================
    MSG: kortex_driver/ControllerElementHandle
    
    ControllerHandle controller_handle
    ControllerElementHandle_identifier oneof_identifier
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
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
    const resolved = new ControllerElementState(null);
    if (msg.handle !== undefined) {
      resolved.handle = ControllerElementHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new ControllerElementHandle()
    }

    if (msg.event_type !== undefined) {
      resolved.event_type = msg.event_type;
    }
    else {
      resolved.event_type = 0
    }

    if (msg.axis_value !== undefined) {
      resolved.axis_value = msg.axis_value;
    }
    else {
      resolved.axis_value = 0.0
    }

    return resolved;
    }
};

module.exports = ControllerElementState;
