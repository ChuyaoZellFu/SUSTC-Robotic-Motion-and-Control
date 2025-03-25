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

class DeviceType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.device_type = null;
    }
    else {
      if (initObj.hasOwnProperty('device_type')) {
        this.device_type = initObj.device_type
      }
      else {
        this.device_type = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceType
    // Serialize message field [device_type]
    bufferOffset = _serializer.uint32(obj.device_type, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceType
    let len;
    let data = new DeviceType(null);
    // Deserialize message field [device_type]
    data.device_type = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DeviceType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03c69ac8f4cbd0c35678776a27f8d84a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 device_type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceType(null);
    if (msg.device_type !== undefined) {
      resolved.device_type = msg.device_type;
    }
    else {
      resolved.device_type = 0
    }

    return resolved;
    }
};

module.exports = DeviceType;
