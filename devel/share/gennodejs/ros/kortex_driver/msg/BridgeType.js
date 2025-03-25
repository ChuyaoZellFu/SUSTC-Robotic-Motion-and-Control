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

class BridgeType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgeType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgeType
    let len;
    let data = new BridgeType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgeType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b8e85d50bf3b32805a4cb6f83f1b2f4f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 BRIDGE_TYPE_UNSPECIFIED = 0
    
    uint32 BRIDGE_TYPE_UART = 1
    
    uint32 BRIDGE_TYPE_TELNET = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BridgeType(null);
    return resolved;
    }
};

// Constants for message
BridgeType.Constants = {
  BRIDGE_TYPE_UNSPECIFIED: 0,
  BRIDGE_TYPE_UART: 1,
  BRIDGE_TYPE_TELNET: 2,
}

module.exports = BridgeType;
