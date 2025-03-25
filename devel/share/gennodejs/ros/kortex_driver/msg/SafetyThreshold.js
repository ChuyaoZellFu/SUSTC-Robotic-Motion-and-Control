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

class SafetyThreshold {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new SafetyHandle();
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyThreshold
    // Serialize message field [handle]
    bufferOffset = SafetyHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float32(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyThreshold
    let len;
    let data = new SafetyThreshold(null);
    // Deserialize message field [handle]
    data.handle = SafetyHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SafetyThreshold';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6785d727fa30955d28c049a2ab6d6c82';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    SafetyHandle handle
    float32 value
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
    const resolved = new SafetyThreshold(null);
    if (msg.handle !== undefined) {
      resolved.handle = SafetyHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new SafetyHandle()
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    return resolved;
    }
};

module.exports = SafetyThreshold;
