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

class MACAddress {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mac_address = null;
    }
    else {
      if (initObj.hasOwnProperty('mac_address')) {
        this.mac_address = initObj.mac_address
      }
      else {
        this.mac_address = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MACAddress
    // Serialize message field [mac_address]
    bufferOffset = _arraySerializer.uint8(obj.mac_address, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MACAddress
    let len;
    let data = new MACAddress(null);
    // Deserialize message field [mac_address]
    data.mac_address = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.mac_address.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MACAddress';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5d1c55e7218bc267e43e7230dde48c16';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint8[] mac_address
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MACAddress(null);
    if (msg.mac_address !== undefined) {
      resolved.mac_address = msg.mac_address;
    }
    else {
      resolved.mac_address = []
    }

    return resolved;
    }
};

module.exports = MACAddress;
