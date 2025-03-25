// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NetworkHandle = require('./NetworkHandle.js');
let IPv4Configuration = require('./IPv4Configuration.js');

//-----------------------------------------------------------

class FullIPv4Configuration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.ipv4_configuration = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new NetworkHandle();
      }
      if (initObj.hasOwnProperty('ipv4_configuration')) {
        this.ipv4_configuration = initObj.ipv4_configuration
      }
      else {
        this.ipv4_configuration = new IPv4Configuration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FullIPv4Configuration
    // Serialize message field [handle]
    bufferOffset = NetworkHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [ipv4_configuration]
    bufferOffset = IPv4Configuration.serialize(obj.ipv4_configuration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FullIPv4Configuration
    let len;
    let data = new FullIPv4Configuration(null);
    // Deserialize message field [handle]
    data.handle = NetworkHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [ipv4_configuration]
    data.ipv4_configuration = IPv4Configuration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FullIPv4Configuration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0ba6974979f83d97757b00aa8a1bc7d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    NetworkHandle handle
    IPv4Configuration ipv4_configuration
    ================================================================================
    MSG: kortex_driver/NetworkHandle
    
    uint32 type
    ================================================================================
    MSG: kortex_driver/IPv4Configuration
    
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
    const resolved = new FullIPv4Configuration(null);
    if (msg.handle !== undefined) {
      resolved.handle = NetworkHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new NetworkHandle()
    }

    if (msg.ipv4_configuration !== undefined) {
      resolved.ipv4_configuration = IPv4Configuration.Resolve(msg.ipv4_configuration)
    }
    else {
      resolved.ipv4_configuration = new IPv4Configuration()
    }

    return resolved;
    }
};

module.exports = FullIPv4Configuration;
