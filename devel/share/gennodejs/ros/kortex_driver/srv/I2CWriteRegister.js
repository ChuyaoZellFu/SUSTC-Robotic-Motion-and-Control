// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CWriteRegisterParameter = require('../msg/I2CWriteRegisterParameter.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class I2CWriteRegisterRequest {
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
        this.input = new I2CWriteRegisterParameter();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CWriteRegisterRequest
    // Serialize message field [input]
    bufferOffset = I2CWriteRegisterParameter.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CWriteRegisterRequest
    let len;
    let data = new I2CWriteRegisterRequest(null);
    // Deserialize message field [input]
    data.input = I2CWriteRegisterParameter.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += I2CWriteRegisterParameter.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CWriteRegisterRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3269839fc92a235a39af85ff65c4ad63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CWriteRegisterParameter input
    
    ================================================================================
    MSG: kortex_driver/I2CWriteRegisterParameter
    
    uint32 device
    uint32 device_address
    uint32 register_address
    uint32 register_address_size
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
    const resolved = new I2CWriteRegisterRequest(null);
    if (msg.input !== undefined) {
      resolved.input = I2CWriteRegisterParameter.Resolve(msg.input)
    }
    else {
      resolved.input = new I2CWriteRegisterParameter()
    }

    return resolved;
    }
};

class I2CWriteRegisterResponse {
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
    // Serializes a message object of type I2CWriteRegisterResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CWriteRegisterResponse
    let len;
    let data = new I2CWriteRegisterResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CWriteRegisterResponse';
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
    const resolved = new I2CWriteRegisterResponse(null);
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
  Request: I2CWriteRegisterRequest,
  Response: I2CWriteRegisterResponse,
  md5sum() { return '51d6dc72e2a979ebcd0930e5191f7f87'; },
  datatype() { return 'kortex_driver/I2CWriteRegister'; }
};
