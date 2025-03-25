// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');

//-----------------------------------------------------------

class FirmwareBundleVersions {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.main_bundle_version = null;
      this.components_versions = null;
    }
    else {
      if (initObj.hasOwnProperty('main_bundle_version')) {
        this.main_bundle_version = initObj.main_bundle_version
      }
      else {
        this.main_bundle_version = '';
      }
      if (initObj.hasOwnProperty('components_versions')) {
        this.components_versions = initObj.components_versions
      }
      else {
        this.components_versions = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FirmwareBundleVersions
    // Serialize message field [main_bundle_version]
    bufferOffset = _serializer.string(obj.main_bundle_version, buffer, bufferOffset);
    // Serialize message field [components_versions]
    // Serialize the length for message field [components_versions]
    bufferOffset = _serializer.uint32(obj.components_versions.length, buffer, bufferOffset);
    obj.components_versions.forEach((val) => {
      bufferOffset = FirmwareComponentVersion.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FirmwareBundleVersions
    let len;
    let data = new FirmwareBundleVersions(null);
    // Deserialize message field [main_bundle_version]
    data.main_bundle_version = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [components_versions]
    // Deserialize array length for message field [components_versions]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.components_versions = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.components_versions[i] = FirmwareComponentVersion.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.main_bundle_version);
    object.components_versions.forEach((val) => {
      length += FirmwareComponentVersion.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FirmwareBundleVersions';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c4f9de789ae8af0bd7b668a4f8f7ed49';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    string main_bundle_version
    FirmwareComponentVersion[] components_versions
    ================================================================================
    MSG: kortex_driver/FirmwareComponentVersion
    
    string name
    string version
    uint32 device_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FirmwareBundleVersions(null);
    if (msg.main_bundle_version !== undefined) {
      resolved.main_bundle_version = msg.main_bundle_version;
    }
    else {
      resolved.main_bundle_version = ''
    }

    if (msg.components_versions !== undefined) {
      resolved.components_versions = new Array(msg.components_versions.length);
      for (let i = 0; i < resolved.components_versions.length; ++i) {
        resolved.components_versions[i] = FirmwareComponentVersion.Resolve(msg.components_versions[i]);
      }
    }
    else {
      resolved.components_versions = []
    }

    return resolved;
    }
};

module.exports = FirmwareBundleVersions;
