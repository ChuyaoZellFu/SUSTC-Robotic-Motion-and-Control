// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Base_CapSenseConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.identifier = null;
      this.mode = null;
      this.threshold_a = null;
      this.threshold_b = null;
      this.sensitivity_a = null;
      this.sensitivity_b = null;
    }
    else {
      if (initObj.hasOwnProperty('identifier')) {
        this.identifier = initObj.identifier
      }
      else {
        this.identifier = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('threshold_a')) {
        this.threshold_a = initObj.threshold_a
      }
      else {
        this.threshold_a = 0.0;
      }
      if (initObj.hasOwnProperty('threshold_b')) {
        this.threshold_b = initObj.threshold_b
      }
      else {
        this.threshold_b = 0.0;
      }
      if (initObj.hasOwnProperty('sensitivity_a')) {
        this.sensitivity_a = initObj.sensitivity_a
      }
      else {
        this.sensitivity_a = 0.0;
      }
      if (initObj.hasOwnProperty('sensitivity_b')) {
        this.sensitivity_b = initObj.sensitivity_b
      }
      else {
        this.sensitivity_b = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Base_CapSenseConfig
    // Serialize message field [identifier]
    bufferOffset = _serializer.uint32(obj.identifier, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.uint32(obj.mode, buffer, bufferOffset);
    // Serialize message field [threshold_a]
    bufferOffset = _serializer.float32(obj.threshold_a, buffer, bufferOffset);
    // Serialize message field [threshold_b]
    bufferOffset = _serializer.float32(obj.threshold_b, buffer, bufferOffset);
    // Serialize message field [sensitivity_a]
    bufferOffset = _serializer.float32(obj.sensitivity_a, buffer, bufferOffset);
    // Serialize message field [sensitivity_b]
    bufferOffset = _serializer.float32(obj.sensitivity_b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Base_CapSenseConfig
    let len;
    let data = new Base_CapSenseConfig(null);
    // Deserialize message field [identifier]
    data.identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [threshold_a]
    data.threshold_a = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [threshold_b]
    data.threshold_b = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sensitivity_a]
    data.sensitivity_a = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sensitivity_b]
    data.sensitivity_b = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Base_CapSenseConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f62e507319ac72b37a316fe4248e2e5a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 identifier
    uint32 mode
    float32 threshold_a
    float32 threshold_b
    float32 sensitivity_a
    float32 sensitivity_b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Base_CapSenseConfig(null);
    if (msg.identifier !== undefined) {
      resolved.identifier = msg.identifier;
    }
    else {
      resolved.identifier = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.threshold_a !== undefined) {
      resolved.threshold_a = msg.threshold_a;
    }
    else {
      resolved.threshold_a = 0.0
    }

    if (msg.threshold_b !== undefined) {
      resolved.threshold_b = msg.threshold_b;
    }
    else {
      resolved.threshold_b = 0.0
    }

    if (msg.sensitivity_a !== undefined) {
      resolved.sensitivity_a = msg.sensitivity_a;
    }
    else {
      resolved.sensitivity_a = 0.0
    }

    if (msg.sensitivity_b !== undefined) {
      resolved.sensitivity_b = msg.sensitivity_b;
    }
    else {
      resolved.sensitivity_b = 0.0
    }

    return resolved;
    }
};

module.exports = Base_CapSenseConfig;
