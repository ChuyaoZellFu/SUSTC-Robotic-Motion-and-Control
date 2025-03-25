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

class NetworkType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NetworkType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NetworkType
    let len;
    let data = new NetworkType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/NetworkType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'de77e3c574da819850bde657b3223236';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_NETWORK_TYPE = 0
    
    uint32 WIFI = 1
    
    uint32 WIRED_ETHERNET = 2
    
    uint32 WIRED_MICROUSB = 3
    
    uint32 WIRED_USB = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NetworkType(null);
    return resolved;
    }
};

// Constants for message
NetworkType.Constants = {
  UNSPECIFIED_NETWORK_TYPE: 0,
  WIFI: 1,
  WIRED_ETHERNET: 2,
  WIRED_MICROUSB: 3,
  WIRED_USB: 4,
}

module.exports = NetworkType;
