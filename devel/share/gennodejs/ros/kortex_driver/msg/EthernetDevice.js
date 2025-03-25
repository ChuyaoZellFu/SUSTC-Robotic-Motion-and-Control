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

class EthernetDevice {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EthernetDevice
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EthernetDevice
    let len;
    let data = new EthernetDevice(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/EthernetDevice';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cbad10c4e89ab17bbf73ffd6e85d4b44';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ETHERNET_DEVICE_UNSPECIFIED = 0
    
    uint32 ETHERNET_DEVICE_EXPANSION = 1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EthernetDevice(null);
    return resolved;
    }
};

// Constants for message
EthernetDevice.Constants = {
  ETHERNET_DEVICE_UNSPECIFIED: 0,
  ETHERNET_DEVICE_EXPANSION: 1,
}

module.exports = EthernetDevice;
