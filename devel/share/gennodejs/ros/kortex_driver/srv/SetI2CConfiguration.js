// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let I2CConfiguration = require('../msg/I2CConfiguration.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetI2CConfigurationRequest {
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
        this.input = new I2CConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetI2CConfigurationRequest
    // Serialize message field [input]
    bufferOffset = I2CConfiguration.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetI2CConfigurationRequest
    let len;
    let data = new SetI2CConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = I2CConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetI2CConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c16ffec51babd77f8fe120b196654d9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    I2CConfiguration input
    
    ================================================================================
    MSG: kortex_driver/I2CConfiguration
    
    uint32 device
    bool enabled
    uint32 mode
    uint32 addressing
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetI2CConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = I2CConfiguration.Resolve(msg.input)
    }
    else {
      resolved.input = new I2CConfiguration()
    }

    return resolved;
    }
};

class SetI2CConfigurationResponse {
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
    // Serializes a message object of type SetI2CConfigurationResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetI2CConfigurationResponse
    let len;
    let data = new SetI2CConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetI2CConfigurationResponse';
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
    const resolved = new SetI2CConfigurationResponse(null);
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
  Request: SetI2CConfigurationRequest,
  Response: SetI2CConfigurationResponse,
  md5sum() { return 'fcaaa1a0d5ba74e939b1b6f1a0988c2a'; },
  datatype() { return 'kortex_driver/SetI2CConfiguration'; }
};
