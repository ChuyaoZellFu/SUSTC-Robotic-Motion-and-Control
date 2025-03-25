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

class BridgeStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgeStatus
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgeStatus
    let len;
    let data = new BridgeStatus(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgeStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9c6586b899b39a3f8a12bc04700166a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 BRIDGE_STATUS_OK = 0
    
    uint32 BRIDGE_STATUS_OUTP_UNAVAILABLE = 1
    
    uint32 BRIDGE_STATUS_UNKNOWN_DEVID = 2
    
    uint32 BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE = 3
    
    uint32 BRIDGE_STATUS_NOT_FOUND = 4
    
    uint32 BRIDGE_STATUS_NOT_INITIALIZED = 5
    
    uint32 BRIDGE_STATUS_UNKNOWN = 6
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BridgeStatus(null);
    return resolved;
    }
};

// Constants for message
BridgeStatus.Constants = {
  BRIDGE_STATUS_OK: 0,
  BRIDGE_STATUS_OUTP_UNAVAILABLE: 1,
  BRIDGE_STATUS_UNKNOWN_DEVID: 2,
  BRIDGE_STATUS_UNKNOWN_BRIDGE_TYPE: 3,
  BRIDGE_STATUS_NOT_FOUND: 4,
  BRIDGE_STATUS_NOT_INITIALIZED: 5,
  BRIDGE_STATUS_UNKNOWN: 6,
}

module.exports = BridgeStatus;
