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

class UARTConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.port_id = null;
      this.enabled = null;
      this.speed = null;
      this.word_length = null;
      this.stop_bits = null;
      this.parity = null;
    }
    else {
      if (initObj.hasOwnProperty('port_id')) {
        this.port_id = initObj.port_id
      }
      else {
        this.port_id = 0;
      }
      if (initObj.hasOwnProperty('enabled')) {
        this.enabled = initObj.enabled
      }
      else {
        this.enabled = false;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0;
      }
      if (initObj.hasOwnProperty('word_length')) {
        this.word_length = initObj.word_length
      }
      else {
        this.word_length = 0;
      }
      if (initObj.hasOwnProperty('stop_bits')) {
        this.stop_bits = initObj.stop_bits
      }
      else {
        this.stop_bits = 0;
      }
      if (initObj.hasOwnProperty('parity')) {
        this.parity = initObj.parity
      }
      else {
        this.parity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UARTConfiguration
    // Serialize message field [port_id]
    bufferOffset = _serializer.uint32(obj.port_id, buffer, bufferOffset);
    // Serialize message field [enabled]
    bufferOffset = _serializer.bool(obj.enabled, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.uint32(obj.speed, buffer, bufferOffset);
    // Serialize message field [word_length]
    bufferOffset = _serializer.uint32(obj.word_length, buffer, bufferOffset);
    // Serialize message field [stop_bits]
    bufferOffset = _serializer.uint32(obj.stop_bits, buffer, bufferOffset);
    // Serialize message field [parity]
    bufferOffset = _serializer.uint32(obj.parity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UARTConfiguration
    let len;
    let data = new UARTConfiguration(null);
    // Deserialize message field [port_id]
    data.port_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [enabled]
    data.enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [word_length]
    data.word_length = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [stop_bits]
    data.stop_bits = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [parity]
    data.parity = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UARTConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da8e919001edf0734e1795af994fc4e9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 port_id
    bool enabled
    uint32 speed
    uint32 word_length
    uint32 stop_bits
    uint32 parity
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UARTConfiguration(null);
    if (msg.port_id !== undefined) {
      resolved.port_id = msg.port_id;
    }
    else {
      resolved.port_id = 0
    }

    if (msg.enabled !== undefined) {
      resolved.enabled = msg.enabled;
    }
    else {
      resolved.enabled = false
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0
    }

    if (msg.word_length !== undefined) {
      resolved.word_length = msg.word_length;
    }
    else {
      resolved.word_length = 0
    }

    if (msg.stop_bits !== undefined) {
      resolved.stop_bits = msg.stop_bits;
    }
    else {
      resolved.stop_bits = 0
    }

    if (msg.parity !== undefined) {
      resolved.parity = msg.parity;
    }
    else {
      resolved.parity = 0
    }

    return resolved;
    }
};

module.exports = UARTConfiguration;
