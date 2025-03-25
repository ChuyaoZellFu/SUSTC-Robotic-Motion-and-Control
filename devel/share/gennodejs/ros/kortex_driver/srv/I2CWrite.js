// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CWriteParameter = require('../msg/I2CWriteParameter.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class I2CWriteRequest {
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
        this.input = new I2CWriteParameter();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CWriteRequest
    // Serialize message field [input]
    bufferOffset = I2CWriteParameter.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CWriteRequest
    let len;
    let data = new I2CWriteRequest(null);
    // Deserialize message field [input]
    data.input = I2CWriteParameter.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += I2CWriteParameter.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CWriteRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be1e22f2d4d44f0edb72cf8cbf2513e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CWriteParameter input
    
    ================================================================================
    MSG: kortex_driver/I2CWriteParameter
    
    uint32 device
    uint32 device_address
    uint32 timeout
    I2CData data
    ================================================================================
    MSG: kortex_driver/I2CData
    
    uint8[] data
    uint32 size
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new I2CWriteRequest(null);
    if (msg.input !== undefined) {
      resolved.input = I2CWriteParameter.Resolve(msg.input)
    }
    else {
      resolved.input = new I2CWriteParameter()
    }

    return resolved;
    }
};

class I2CWriteResponse {
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
        this.output = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CWriteResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CWriteResponse
    let len;
    let data = new I2CWriteResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CWriteResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6c43d221c810050f75091660f63b0cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty output
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new I2CWriteResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Empty.Resolve(msg.output)
    }
    else {
      resolved.output = new Empty()
    }

    return resolved;
    }
};

module.exports = {
  Request: I2CWriteRequest,
  Response: I2CWriteResponse,
  md5sum() { return '976795fac6276cbe1f1d899c54c1e043'; },
  datatype() { return 'kortex_driver/I2CWrite'; }
};
