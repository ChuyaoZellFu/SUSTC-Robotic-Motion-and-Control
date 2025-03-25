// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

let FirmwareBundleVersions = require('../msg/FirmwareBundleVersions.js');

//-----------------------------------------------------------

class GetFirmwareBundleVersionsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetFirmwareBundleVersionsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetFirmwareBundleVersionsRequest
    let len;
    let data = new GetFirmwareBundleVersionsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetFirmwareBundleVersionsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa3403cd5897c9698bc0fdcb2a453fbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty input
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetFirmwareBundleVersionsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetFirmwareBundleVersionsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = new FirmwareBundleVersions();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetFirmwareBundleVersionsResponse
    // Serialize message field [output]
    bufferOffset = FirmwareBundleVersions.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetFirmwareBundleVersionsResponse
    let len;
    let data = new GetFirmwareBundleVersionsResponse(null);
    // Deserialize message field [output]
    data.output = FirmwareBundleVersions.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += FirmwareBundleVersions.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetFirmwareBundleVersionsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ae4986d5c9d75c597b5e6ac617efffa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    FirmwareBundleVersions output
    
    ================================================================================
    MSG: kortex_driver/FirmwareBundleVersions
    
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
    const resolved = new GetFirmwareBundleVersionsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = FirmwareBundleVersions.Resolve(msg.output)
    }
    else {
      resolved.output = new FirmwareBundleVersions()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetFirmwareBundleVersionsRequest,
  Response: GetFirmwareBundleVersionsResponse,
  md5sum() { return '8a198c1bd1965cc2d1e06f5a3548e004'; },
  datatype() { return 'kortex_driver/GetFirmwareBundleVersions'; }
};
