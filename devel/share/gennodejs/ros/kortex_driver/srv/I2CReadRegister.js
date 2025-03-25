// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CReadRegisterParameter = require('../msg/I2CReadRegisterParameter.js');

//-----------------------------------------------------------

let I2CData = require('../msg/I2CData.js');

//-----------------------------------------------------------

class I2CReadRegisterRequest {
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
        this.input = new I2CReadRegisterParameter();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CReadRegisterRequest
    // Serialize message field [input]
    bufferOffset = I2CReadRegisterParameter.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CReadRegisterRequest
    let len;
    let data = new I2CReadRegisterRequest(null);
    // Deserialize message field [input]
    data.input = I2CReadRegisterParameter.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CReadRegisterRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '61cda3da087be1f15ac91b9bec5f6a34';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CReadRegisterParameter input
    
    ================================================================================
    MSG: kortex_driver/I2CReadRegisterParameter
    
    uint32 device
    uint32 device_address
    uint32 register_address
    uint32 register_address_size
    uint32 size
    uint32 timeout
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new I2CReadRegisterRequest(null);
    if (msg.input !== undefined) {
      resolved.input = I2CReadRegisterParameter.Resolve(msg.input)
    }
    else {
      resolved.input = new I2CReadRegisterParameter()
    }

    return resolved;
    }
};

class I2CReadRegisterResponse {
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
        this.output = new I2CData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CReadRegisterResponse
    // Serialize message field [output]
    bufferOffset = I2CData.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CReadRegisterResponse
    let len;
    let data = new I2CReadRegisterResponse(null);
    // Deserialize message field [output]
    data.output = I2CData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += I2CData.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CReadRegisterResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d3e4addaef52f5412d20a5c63b66a92';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CData output
    
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
    const resolved = new I2CReadRegisterResponse(null);
    if (msg.output !== undefined) {
      resolved.output = I2CData.Resolve(msg.output)
    }
    else {
      resolved.output = new I2CData()
    }

    return resolved;
    }
};

module.exports = {
  Request: I2CReadRegisterRequest,
  Response: I2CReadRegisterResponse,
  md5sum() { return '4bdf02ffe8fd0ef6ff8749422c787f42'; },
  datatype() { return 'kortex_driver/I2CReadRegister'; }
};
