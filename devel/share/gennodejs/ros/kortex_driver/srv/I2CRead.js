// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CReadParameter = require('../msg/I2CReadParameter.js');

//-----------------------------------------------------------

let I2CData = require('../msg/I2CData.js');

//-----------------------------------------------------------

class I2CReadRequest {
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
        this.input = new I2CReadParameter();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type I2CReadRequest
    // Serialize message field [input]
    bufferOffset = I2CReadParameter.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CReadRequest
    let len;
    let data = new I2CReadRequest(null);
    // Deserialize message field [input]
    data.input = I2CReadParameter.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/I2CReadRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '27df07859b2585696c799d0a9746ced5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CReadParameter input
    
    ================================================================================
    MSG: kortex_driver/I2CReadParameter
    
    uint32 device
    uint32 device_address
    uint32 size
    uint32 timeout
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new I2CReadRequest(null);
    if (msg.input !== undefined) {
      resolved.input = I2CReadParameter.Resolve(msg.input)
    }
    else {
      resolved.input = new I2CReadParameter()
    }

    return resolved;
    }
};

class I2CReadResponse {
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
    // Serializes a message object of type I2CReadResponse
    // Serialize message field [output]
    bufferOffset = I2CData.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type I2CReadResponse
    let len;
    let data = new I2CReadResponse(null);
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
    return 'kortex_driver/I2CReadResponse';
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
    const resolved = new I2CReadResponse(null);
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
  Request: I2CReadRequest,
  Response: I2CReadResponse,
  md5sum() { return '76252f712912deacb218cb19eaf79993'; },
  datatype() { return 'kortex_driver/I2CRead'; }
};
