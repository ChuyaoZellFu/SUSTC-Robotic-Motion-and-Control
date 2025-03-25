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

class BootloaderVersion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bootloader_version = null;
    }
    else {
      if (initObj.hasOwnProperty('bootloader_version')) {
        this.bootloader_version = initObj.bootloader_version
      }
      else {
        this.bootloader_version = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BootloaderVersion
    // Serialize message field [bootloader_version]
    bufferOffset = _serializer.uint32(obj.bootloader_version, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BootloaderVersion
    let len;
    let data = new BootloaderVersion(null);
    // Deserialize message field [bootloader_version]
    data.bootloader_version = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BootloaderVersion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ec0c056f9a04b0432415c72e0d38271e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 bootloader_version
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BootloaderVersion(null);
    if (msg.bootloader_version !== undefined) {
      resolved.bootloader_version = msg.bootloader_version;
    }
    else {
      resolved.bootloader_version = 0
    }

    return resolved;
    }
};

module.exports = BootloaderVersion;
