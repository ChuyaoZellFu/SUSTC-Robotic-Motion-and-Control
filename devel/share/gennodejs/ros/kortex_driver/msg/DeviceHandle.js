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

class DeviceHandle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.device_type = null;
      this.device_identifier = null;
      this.order = null;
    }
    else {
      if (initObj.hasOwnProperty('device_type')) {
        this.device_type = initObj.device_type
      }
      else {
        this.device_type = 0;
      }
      if (initObj.hasOwnProperty('device_identifier')) {
        this.device_identifier = initObj.device_identifier
      }
      else {
        this.device_identifier = 0;
      }
      if (initObj.hasOwnProperty('order')) {
        this.order = initObj.order
      }
      else {
        this.order = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceHandle
    // Serialize message field [device_type]
    bufferOffset = _serializer.uint32(obj.device_type, buffer, bufferOffset);
    // Serialize message field [device_identifier]
    bufferOffset = _serializer.uint32(obj.device_identifier, buffer, bufferOffset);
    // Serialize message field [order]
    bufferOffset = _serializer.uint32(obj.order, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceHandle
    let len;
    let data = new DeviceHandle(null);
    // Deserialize message field [device_type]
    data.device_type = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [device_identifier]
    data.device_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [order]
    data.order = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DeviceHandle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a89d226be38807777119fd9acd7acb3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 device_type
    uint32 device_identifier
    uint32 order
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceHandle(null);
    if (msg.device_type !== undefined) {
      resolved.device_type = msg.device_type;
    }
    else {
      resolved.device_type = 0
    }

    if (msg.device_identifier !== undefined) {
      resolved.device_identifier = msg.device_identifier;
    }
    else {
      resolved.device_identifier = 0
    }

    if (msg.order !== undefined) {
      resolved.order = msg.order;
    }
    else {
      resolved.order = 0
    }

    return resolved;
    }
};

module.exports = DeviceHandle;
