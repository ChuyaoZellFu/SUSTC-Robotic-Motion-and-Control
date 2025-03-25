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

class DeviceConfig_CapSenseConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.threshold_a = null;
      this.threshold_b = null;
    }
    else {
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceConfig_CapSenseConfig
    // Serialize message field [mode]
    bufferOffset = _serializer.uint32(obj.mode, buffer, bufferOffset);
    // Serialize message field [threshold_a]
    bufferOffset = _serializer.float32(obj.threshold_a, buffer, bufferOffset);
    // Serialize message field [threshold_b]
    bufferOffset = _serializer.float32(obj.threshold_b, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConfig_CapSenseConfig
    let len;
    let data = new DeviceConfig_CapSenseConfig(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [threshold_a]
    data.threshold_a = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [threshold_b]
    data.threshold_b = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DeviceConfig_CapSenseConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd34e36d2242f94fe2b0925f57d7bdbd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 mode
    float32 threshold_a
    float32 threshold_b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceConfig_CapSenseConfig(null);
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

    return resolved;
    }
};

module.exports = DeviceConfig_CapSenseConfig;
