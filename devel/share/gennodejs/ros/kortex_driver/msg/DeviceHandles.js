// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DeviceHandle = require('./DeviceHandle.js');

//-----------------------------------------------------------

class DeviceHandles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.device_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('device_handle')) {
        this.device_handle = initObj.device_handle
      }
      else {
        this.device_handle = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceHandles
    // Serialize message field [device_handle]
    // Serialize the length for message field [device_handle]
    bufferOffset = _serializer.uint32(obj.device_handle.length, buffer, bufferOffset);
    obj.device_handle.forEach((val) => {
      bufferOffset = DeviceHandle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceHandles
    let len;
    let data = new DeviceHandles(null);
    // Deserialize message field [device_handle]
    // Deserialize array length for message field [device_handle]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.device_handle = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.device_handle[i] = DeviceHandle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.device_handle.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/DeviceHandles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad854635e0d722ce08228e35a86c0173';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    DeviceHandle[] device_handle
    ================================================================================
    MSG: kortex_driver/DeviceHandle
    
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
    const resolved = new DeviceHandles(null);
    if (msg.device_handle !== undefined) {
      resolved.device_handle = new Array(msg.device_handle.length);
      for (let i = 0; i < resolved.device_handle.length; ++i) {
        resolved.device_handle[i] = DeviceHandle.Resolve(msg.device_handle[i]);
      }
    }
    else {
      resolved.device_handle = []
    }

    return resolved;
    }
};

module.exports = DeviceHandles;
