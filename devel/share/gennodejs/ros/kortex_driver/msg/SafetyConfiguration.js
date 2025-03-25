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
let SafetyEnable = require('./SafetyEnable.js');

//-----------------------------------------------------------

class SafetyConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.error_threshold = null;
      this.warning_threshold = null;
      this.enable = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new SafetyHandle();
      }
      if (initObj.hasOwnProperty('error_threshold')) {
        this.error_threshold = initObj.error_threshold
      }
      else {
        this.error_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('warning_threshold')) {
        this.warning_threshold = initObj.warning_threshold
      }
      else {
        this.warning_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('enable')) {
        this.enable = initObj.enable
      }
      else {
        this.enable = new SafetyEnable();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyConfiguration
    // Serialize message field [handle]
    bufferOffset = SafetyHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [error_threshold]
    bufferOffset = _serializer.float32(obj.error_threshold, buffer, bufferOffset);
    // Serialize message field [warning_threshold]
    bufferOffset = _serializer.float32(obj.warning_threshold, buffer, bufferOffset);
    // Serialize message field [enable]
    bufferOffset = SafetyEnable.serialize(obj.enable, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyConfiguration
    let len;
    let data = new SafetyConfiguration(null);
    // Deserialize message field [handle]
    data.handle = SafetyHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [error_threshold]
    data.error_threshold = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [warning_threshold]
    data.warning_threshold = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [enable]
    data.enable = SafetyEnable.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '42160db695be444fee95ed355f4763b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyHandle handle
    float32 error_threshold
    float32 warning_threshold
    SafetyEnable enable
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    ================================================================================
    MSG: kortex_driver/SafetyEnable
    
    SafetyHandle handle
    bool enable
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SafetyConfiguration(null);
    if (msg.handle !== undefined) {
      resolved.handle = SafetyHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new SafetyHandle()
    }

    if (msg.error_threshold !== undefined) {
      resolved.error_threshold = msg.error_threshold;
    }
    else {
      resolved.error_threshold = 0.0
    }

    if (msg.warning_threshold !== undefined) {
      resolved.warning_threshold = msg.warning_threshold;
    }
    else {
      resolved.warning_threshold = 0.0
    }

    if (msg.enable !== undefined) {
      resolved.enable = SafetyEnable.Resolve(msg.enable)
    }
    else {
      resolved.enable = new SafetyEnable()
    }

    return resolved;
    }
};

module.exports = SafetyConfiguration;
