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

class IPv4Settings {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ipv4_address = null;
      this.ipv4_subnet_mask = null;
      this.ipv4_default_gateway = null;
    }
    else {
      if (initObj.hasOwnProperty('ipv4_address')) {
        this.ipv4_address = initObj.ipv4_address
      }
      else {
        this.ipv4_address = 0;
      }
      if (initObj.hasOwnProperty('ipv4_subnet_mask')) {
        this.ipv4_subnet_mask = initObj.ipv4_subnet_mask
      }
      else {
        this.ipv4_subnet_mask = 0;
      }
      if (initObj.hasOwnProperty('ipv4_default_gateway')) {
        this.ipv4_default_gateway = initObj.ipv4_default_gateway
      }
      else {
        this.ipv4_default_gateway = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IPv4Settings
    // Serialize message field [ipv4_address]
    bufferOffset = _serializer.uint32(obj.ipv4_address, buffer, bufferOffset);
    // Serialize message field [ipv4_subnet_mask]
    bufferOffset = _serializer.uint32(obj.ipv4_subnet_mask, buffer, bufferOffset);
    // Serialize message field [ipv4_default_gateway]
    bufferOffset = _serializer.uint32(obj.ipv4_default_gateway, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IPv4Settings
    let len;
    let data = new IPv4Settings(null);
    // Deserialize message field [ipv4_address]
    data.ipv4_address = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ipv4_subnet_mask]
    data.ipv4_subnet_mask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ipv4_default_gateway]
    data.ipv4_default_gateway = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/IPv4Settings';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '485e8d7e0822786c2180cac4b25f7434';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ipv4_address
    uint32 ipv4_subnet_mask
    uint32 ipv4_default_gateway
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IPv4Settings(null);
    if (msg.ipv4_address !== undefined) {
      resolved.ipv4_address = msg.ipv4_address;
    }
    else {
      resolved.ipv4_address = 0
    }

    if (msg.ipv4_subnet_mask !== undefined) {
      resolved.ipv4_subnet_mask = msg.ipv4_subnet_mask;
    }
    else {
      resolved.ipv4_subnet_mask = 0
    }

    if (msg.ipv4_default_gateway !== undefined) {
      resolved.ipv4_default_gateway = msg.ipv4_default_gateway;
    }
    else {
      resolved.ipv4_default_gateway = 0
    }

    return resolved;
    }
};

module.exports = IPv4Settings;
