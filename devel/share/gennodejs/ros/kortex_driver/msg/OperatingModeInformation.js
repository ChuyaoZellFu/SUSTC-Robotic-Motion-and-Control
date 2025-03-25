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

class OperatingModeInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.operating_mode = null;
      this.device_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('operating_mode')) {
        this.operating_mode = initObj.operating_mode
      }
      else {
        this.operating_mode = 0;
      }
      if (initObj.hasOwnProperty('device_handle')) {
        this.device_handle = initObj.device_handle
      }
      else {
        this.device_handle = new DeviceHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OperatingModeInformation
    // Serialize message field [operating_mode]
    bufferOffset = _serializer.uint32(obj.operating_mode, buffer, bufferOffset);
    // Serialize message field [device_handle]
    bufferOffset = DeviceHandle.serialize(obj.device_handle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OperatingModeInformation
    let len;
    let data = new OperatingModeInformation(null);
    // Deserialize message field [operating_mode]
    data.operating_mode = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [device_handle]
    data.device_handle = DeviceHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/OperatingModeInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c45e380eac0023b4784c990d000302a3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 operating_mode
    DeviceHandle device_handle
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
    const resolved = new OperatingModeInformation(null);
    if (msg.operating_mode !== undefined) {
      resolved.operating_mode = msg.operating_mode;
    }
    else {
      resolved.operating_mode = 0
    }

    if (msg.device_handle !== undefined) {
      resolved.device_handle = DeviceHandle.Resolve(msg.device_handle)
    }
    else {
      resolved.device_handle = new DeviceHandle()
    }

    return resolved;
    }
};

module.exports = OperatingModeInformation;
