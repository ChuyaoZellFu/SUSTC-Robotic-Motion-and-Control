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

let FirmwareVersion = require('../msg/FirmwareVersion.js');

//-----------------------------------------------------------

class GetFirmwareVersionRequest {
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
    // Serializes a message object of type GetFirmwareVersionRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetFirmwareVersionRequest
    let len;
    let data = new GetFirmwareVersionRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetFirmwareVersionRequest';
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
    const resolved = new GetFirmwareVersionRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetFirmwareVersionResponse {
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
        this.output = new FirmwareVersion();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetFirmwareVersionResponse
    // Serialize message field [output]
    bufferOffset = FirmwareVersion.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetFirmwareVersionResponse
    let len;
    let data = new GetFirmwareVersionResponse(null);
    // Deserialize message field [output]
    data.output = FirmwareVersion.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetFirmwareVersionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '21015a11b53d0e99fd37623e0604662b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    FirmwareVersion output
    
    ================================================================================
    MSG: kortex_driver/FirmwareVersion
    
    uint32 firmware_version
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetFirmwareVersionResponse(null);
    if (msg.output !== undefined) {
      resolved.output = FirmwareVersion.Resolve(msg.output)
    }
    else {
      resolved.output = new FirmwareVersion()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetFirmwareVersionRequest,
  Response: GetFirmwareVersionResponse,
  md5sum() { return 'b9c18ab692345d79a149debba533958f'; },
  datatype() { return 'kortex_driver/GetFirmwareVersion'; }
};
