// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ActionHandle = require('./ActionHandle.js');

//-----------------------------------------------------------

class ActionExecutionState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_event = null;
      this.handle = null;
    }
    else {
      if (initObj.hasOwnProperty('action_event')) {
        this.action_event = initObj.action_event
      }
      else {
        this.action_event = 0;
      }
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new ActionHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ActionExecutionState
    // Serialize message field [action_event]
    bufferOffset = _serializer.uint32(obj.action_event, buffer, bufferOffset);
    // Serialize message field [handle]
    bufferOffset = ActionHandle.serialize(obj.handle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ActionExecutionState
    let len;
    let data = new ActionExecutionState(null);
    // Deserialize message field [action_event]
    data.action_event = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [handle]
    data.handle = ActionHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ActionExecutionState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ffbf0e1fde1af8a50b9f2f937e17d3ac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 action_event
    ActionHandle handle
    ================================================================================
    MSG: kortex_driver/ActionHandle
    
    uint32 identifier
    uint32 action_type
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ActionExecutionState(null);
    if (msg.action_event !== undefined) {
      resolved.action_event = msg.action_event;
    }
    else {
      resolved.action_event = 0
    }

    if (msg.handle !== undefined) {
      resolved.handle = ActionHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new ActionHandle()
    }

    return resolved;
    }
};

module.exports = ActionExecutionState;
