// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let OptionIdentifier = require('../msg/OptionIdentifier.js');

//-----------------------------------------------------------

let OptionValue = require('../msg/OptionValue.js');

//-----------------------------------------------------------

class GetOptionValueRequest {
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
        this.input = new OptionIdentifier();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOptionValueRequest
    // Serialize message field [input]
    bufferOffset = OptionIdentifier.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOptionValueRequest
    let len;
    let data = new GetOptionValueRequest(null);
    // Deserialize message field [input]
    data.input = OptionIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetOptionValueRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e66b1b0de1a710df3ff2cc0bc64b9ced';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    OptionIdentifier input
    
    ================================================================================
    MSG: kortex_driver/OptionIdentifier
    
    uint32 sensor
    uint32 option
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOptionValueRequest(null);
    if (msg.input !== undefined) {
      resolved.input = OptionIdentifier.Resolve(msg.input)
    }
    else {
      resolved.input = new OptionIdentifier()
    }

    return resolved;
    }
};

class GetOptionValueResponse {
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
        this.output = new OptionValue();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOptionValueResponse
    // Serialize message field [output]
    bufferOffset = OptionValue.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOptionValueResponse
    let len;
    let data = new GetOptionValueResponse(null);
    // Deserialize message field [output]
    data.output = OptionValue.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetOptionValueResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17f8b885036a7e161a39d06f11b7725b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    OptionValue output
    
    ================================================================================
    MSG: kortex_driver/OptionValue
    
    uint32 sensor
    uint32 option
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOptionValueResponse(null);
    if (msg.output !== undefined) {
      resolved.output = OptionValue.Resolve(msg.output)
    }
    else {
      resolved.output = new OptionValue()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetOptionValueRequest,
  Response: GetOptionValueResponse,
  md5sum() { return 'e414d686b57953b5748ab5bedaec5577'; },
  datatype() { return 'kortex_driver/GetOptionValue'; }
};
