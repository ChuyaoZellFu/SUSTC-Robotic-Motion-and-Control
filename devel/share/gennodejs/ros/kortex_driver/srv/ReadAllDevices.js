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

let DeviceHandles = require('../msg/DeviceHandles.js');

//-----------------------------------------------------------

class ReadAllDevicesRequest {
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
    // Serializes a message object of type ReadAllDevicesRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadAllDevicesRequest
    let len;
    let data = new ReadAllDevicesRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadAllDevicesRequest';
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
    const resolved = new ReadAllDevicesRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class ReadAllDevicesResponse {
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
        this.output = new DeviceHandles();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadAllDevicesResponse
    // Serialize message field [output]
    bufferOffset = DeviceHandles.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadAllDevicesResponse
    let len;
    let data = new ReadAllDevicesResponse(null);
    // Deserialize message field [output]
    data.output = DeviceHandles.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += DeviceHandles.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadAllDevicesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a298fcf8896256a81f2b64c04ef7cfae';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    DeviceHandles output
    
    ================================================================================
    MSG: kortex_driver/DeviceHandles
    
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
    const resolved = new ReadAllDevicesResponse(null);
    if (msg.output !== undefined) {
      resolved.output = DeviceHandles.Resolve(msg.output)
    }
    else {
      resolved.output = new DeviceHandles()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadAllDevicesRequest,
  Response: ReadAllDevicesResponse,
  md5sum() { return '9f8a3530be8da22ec93eee7ab47ef9a0'; },
  datatype() { return 'kortex_driver/ReadAllDevices'; }
};
