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

class SafetyInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.can_change_safety_state = null;
      this.has_warning_threshold = null;
      this.has_error_threshold = null;
      this.limit_type = null;
      this.default_warning_threshold = null;
      this.default_error_threshold = null;
      this.upper_hard_limit = null;
      this.lower_hard_limit = null;
      this.status = null;
      this.unit = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new SafetyHandle();
      }
      if (initObj.hasOwnProperty('can_change_safety_state')) {
        this.can_change_safety_state = initObj.can_change_safety_state
      }
      else {
        this.can_change_safety_state = false;
      }
      if (initObj.hasOwnProperty('has_warning_threshold')) {
        this.has_warning_threshold = initObj.has_warning_threshold
      }
      else {
        this.has_warning_threshold = false;
      }
      if (initObj.hasOwnProperty('has_error_threshold')) {
        this.has_error_threshold = initObj.has_error_threshold
      }
      else {
        this.has_error_threshold = false;
      }
      if (initObj.hasOwnProperty('limit_type')) {
        this.limit_type = initObj.limit_type
      }
      else {
        this.limit_type = 0;
      }
      if (initObj.hasOwnProperty('default_warning_threshold')) {
        this.default_warning_threshold = initObj.default_warning_threshold
      }
      else {
        this.default_warning_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('default_error_threshold')) {
        this.default_error_threshold = initObj.default_error_threshold
      }
      else {
        this.default_error_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('upper_hard_limit')) {
        this.upper_hard_limit = initObj.upper_hard_limit
      }
      else {
        this.upper_hard_limit = 0.0;
      }
      if (initObj.hasOwnProperty('lower_hard_limit')) {
        this.lower_hard_limit = initObj.lower_hard_limit
      }
      else {
        this.lower_hard_limit = 0.0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('unit')) {
        this.unit = initObj.unit
      }
      else {
        this.unit = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyInformation
    // Serialize message field [handle]
    bufferOffset = SafetyHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [can_change_safety_state]
    bufferOffset = _serializer.bool(obj.can_change_safety_state, buffer, bufferOffset);
    // Serialize message field [has_warning_threshold]
    bufferOffset = _serializer.bool(obj.has_warning_threshold, buffer, bufferOffset);
    // Serialize message field [has_error_threshold]
    bufferOffset = _serializer.bool(obj.has_error_threshold, buffer, bufferOffset);
    // Serialize message field [limit_type]
    bufferOffset = _serializer.uint32(obj.limit_type, buffer, bufferOffset);
    // Serialize message field [default_warning_threshold]
    bufferOffset = _serializer.float32(obj.default_warning_threshold, buffer, bufferOffset);
    // Serialize message field [default_error_threshold]
    bufferOffset = _serializer.float32(obj.default_error_threshold, buffer, bufferOffset);
    // Serialize message field [upper_hard_limit]
    bufferOffset = _serializer.float32(obj.upper_hard_limit, buffer, bufferOffset);
    // Serialize message field [lower_hard_limit]
    bufferOffset = _serializer.float32(obj.lower_hard_limit, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint32(obj.status, buffer, bufferOffset);
    // Serialize message field [unit]
    bufferOffset = _serializer.uint32(obj.unit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyInformation
    let len;
    let data = new SafetyInformation(null);
    // Deserialize message field [handle]
    data.handle = SafetyHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [can_change_safety_state]
    data.can_change_safety_state = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [has_warning_threshold]
    data.has_warning_threshold = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [has_error_threshold]
    data.has_error_threshold = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [limit_type]
    data.limit_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [default_warning_threshold]
    data.default_warning_threshold = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [default_error_threshold]
    data.default_error_threshold = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [upper_hard_limit]
    data.upper_hard_limit = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [lower_hard_limit]
    data.lower_hard_limit = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [unit]
    data.unit = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 35;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e8597ef9acfa23c653020b88d86d8b2f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyHandle handle
    bool can_change_safety_state
    bool has_warning_threshold
    bool has_error_threshold
    uint32 limit_type
    float32 default_warning_threshold
    float32 default_error_threshold
    float32 upper_hard_limit
    float32 lower_hard_limit
    uint32 status
    uint32 unit
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
    const resolved = new SafetyInformation(null);
    if (msg.handle !== undefined) {
      resolved.handle = SafetyHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new SafetyHandle()
    }

    if (msg.can_change_safety_state !== undefined) {
      resolved.can_change_safety_state = msg.can_change_safety_state;
    }
    else {
      resolved.can_change_safety_state = false
    }

    if (msg.has_warning_threshold !== undefined) {
      resolved.has_warning_threshold = msg.has_warning_threshold;
    }
    else {
      resolved.has_warning_threshold = false
    }

    if (msg.has_error_threshold !== undefined) {
      resolved.has_error_threshold = msg.has_error_threshold;
    }
    else {
      resolved.has_error_threshold = false
    }

    if (msg.limit_type !== undefined) {
      resolved.limit_type = msg.limit_type;
    }
    else {
      resolved.limit_type = 0
    }

    if (msg.default_warning_threshold !== undefined) {
      resolved.default_warning_threshold = msg.default_warning_threshold;
    }
    else {
      resolved.default_warning_threshold = 0.0
    }

    if (msg.default_error_threshold !== undefined) {
      resolved.default_error_threshold = msg.default_error_threshold;
    }
    else {
      resolved.default_error_threshold = 0.0
    }

    if (msg.upper_hard_limit !== undefined) {
      resolved.upper_hard_limit = msg.upper_hard_limit;
    }
    else {
      resolved.upper_hard_limit = 0.0
    }

    if (msg.lower_hard_limit !== undefined) {
      resolved.lower_hard_limit = msg.lower_hard_limit;
    }
    else {
      resolved.lower_hard_limit = 0.0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.unit !== undefined) {
      resolved.unit = msg.unit;
    }
    else {
      resolved.unit = 0
    }

    return resolved;
    }
};

module.exports = SafetyInformation;
