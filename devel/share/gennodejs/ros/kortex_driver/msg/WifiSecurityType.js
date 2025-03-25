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

class WifiSecurityType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WifiSecurityType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WifiSecurityType
    let len;
    let data = new WifiSecurityType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WifiSecurityType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '758c13b30796cf34636bc4ccee207084';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_AUTHENTICATION = 0
    
    uint32 WEP = 1
    
    uint32 WPA2_PERSONAL = 2
    
    uint32 WPA_PERSONAL = 4
    
    uint32 NO_AUTHENTICATION = 8
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WifiSecurityType(null);
    return resolved;
    }
};

// Constants for message
WifiSecurityType.Constants = {
  UNSPECIFIED_AUTHENTICATION: 0,
  WEP: 1,
  WPA2_PERSONAL: 2,
  WPA_PERSONAL: 4,
  NO_AUTHENTICATION: 8,
}

module.exports = WifiSecurityType;
