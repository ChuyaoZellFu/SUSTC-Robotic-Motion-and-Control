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

class WifiEncryptionType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WifiEncryptionType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WifiEncryptionType
    let len;
    let data = new WifiEncryptionType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/WifiEncryptionType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5312088e9fe010ba1511377fa25b980d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_ENCRYPTION = 0
    
    uint32 AES_ENCRYPTION = 1
    
    uint32 TKIP_ENCRYPTION = 2
    
    uint32 WEP_ENCRYPTION = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WifiEncryptionType(null);
    return resolved;
    }
};

// Constants for message
WifiEncryptionType.Constants = {
  UNSPECIFIED_ENCRYPTION: 0,
  AES_ENCRYPTION: 1,
  TKIP_ENCRYPTION: 2,
  WEP_ENCRYPTION: 4,
}

module.exports = WifiEncryptionType;
