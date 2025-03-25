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

class BridgePortConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_port = null;
      this.out_port = null;
    }
    else {
      if (initObj.hasOwnProperty('target_port')) {
        this.target_port = initObj.target_port
      }
      else {
        this.target_port = 0;
      }
      if (initObj.hasOwnProperty('out_port')) {
        this.out_port = initObj.out_port
      }
      else {
        this.out_port = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgePortConfig
    // Serialize message field [target_port]
    bufferOffset = _serializer.uint32(obj.target_port, buffer, bufferOffset);
    // Serialize message field [out_port]
    bufferOffset = _serializer.uint32(obj.out_port, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgePortConfig
    let len;
    let data = new BridgePortConfig(null);
    // Deserialize message field [target_port]
    data.target_port = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [out_port]
    data.out_port = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgePortConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c2591c5a923ec65c5c9c9b1eb43de273';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 target_port
    uint32 out_port
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BridgePortConfig(null);
    if (msg.target_port !== undefined) {
      resolved.target_port = msg.target_port;
    }
    else {
      resolved.target_port = 0
    }

    if (msg.out_port !== undefined) {
      resolved.out_port = msg.out_port;
    }
    else {
      resolved.out_port = 0
    }

    return resolved;
    }
};

module.exports = BridgePortConfig;
