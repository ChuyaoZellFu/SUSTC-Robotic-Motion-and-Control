// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyHandle = require('./SafetyHandle.js');

//-----------------------------------------------------------

class SafetyEnable {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.enable = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new SafetyHandle();
      }
      if (initObj.hasOwnProperty('enable')) {
        this.enable = initObj.enable
      }
      else {
        this.enable = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyEnable
    // Serialize message field [handle]
    bufferOffset = SafetyHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [enable]
    bufferOffset = _serializer.bool(obj.enable, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyEnable
    let len;
    let data = new SafetyEnable(null);
    // Deserialize message field [handle]
    data.handle = SafetyHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [enable]
    data.enable = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyEnable';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5f3f3341f0f54081acc9ca67a9de861a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyHandle handle
    bool enable
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SafetyEnable(null);
    if (msg.handle !== undefined) {
      resolved.handle = SafetyHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new SafetyHandle()
    }

    if (msg.enable !== undefined) {
      resolved.enable = msg.enable;
    }
    else {
      resolved.enable = false
    }

    return resolved;
    }
};

module.exports = SafetyEnable;
