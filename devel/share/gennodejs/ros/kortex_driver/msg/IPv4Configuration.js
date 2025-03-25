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

class IPv4Configuration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ip_address = null;
      this.subnet_mask = null;
      this.default_gateway = null;
      this.dhcp_enabled = null;
    }
    else {
      if (initObj.hasOwnProperty('ip_address')) {
        this.ip_address = initObj.ip_address
      }
      else {
        this.ip_address = 0;
      }
      if (initObj.hasOwnProperty('subnet_mask')) {
        this.subnet_mask = initObj.subnet_mask
      }
      else {
        this.subnet_mask = 0;
      }
      if (initObj.hasOwnProperty('default_gateway')) {
        this.default_gateway = initObj.default_gateway
      }
      else {
        this.default_gateway = 0;
      }
      if (initObj.hasOwnProperty('dhcp_enabled')) {
        this.dhcp_enabled = initObj.dhcp_enabled
      }
      else {
        this.dhcp_enabled = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IPv4Configuration
    // Serialize message field [ip_address]
    bufferOffset = _serializer.uint32(obj.ip_address, buffer, bufferOffset);
    // Serialize message field [subnet_mask]
    bufferOffset = _serializer.uint32(obj.subnet_mask, buffer, bufferOffset);
    // Serialize message field [default_gateway]
    bufferOffset = _serializer.uint32(obj.default_gateway, buffer, bufferOffset);
    // Serialize message field [dhcp_enabled]
    bufferOffset = _serializer.bool(obj.dhcp_enabled, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IPv4Configuration
    let len;
    let data = new IPv4Configuration(null);
    // Deserialize message field [ip_address]
    data.ip_address = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [subnet_mask]
    data.subnet_mask = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [default_gateway]
    data.default_gateway = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [dhcp_enabled]
    data.dhcp_enabled = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/IPv4Configuration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '938bacc92850e7d808a96d82d5b98e81';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ip_address
    uint32 subnet_mask
    uint32 default_gateway
    bool dhcp_enabled
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IPv4Configuration(null);
    if (msg.ip_address !== undefined) {
      resolved.ip_address = msg.ip_address;
    }
    else {
      resolved.ip_address = 0
    }

    if (msg.subnet_mask !== undefined) {
      resolved.subnet_mask = msg.subnet_mask;
    }
    else {
      resolved.subnet_mask = 0
    }

    if (msg.default_gateway !== undefined) {
      resolved.default_gateway = msg.default_gateway;
    }
    else {
      resolved.default_gateway = 0
    }

    if (msg.dhcp_enabled !== undefined) {
      resolved.dhcp_enabled = msg.dhcp_enabled;
    }
    else {
      resolved.dhcp_enabled = false
    }

    return resolved;
    }
};

module.exports = IPv4Configuration;
