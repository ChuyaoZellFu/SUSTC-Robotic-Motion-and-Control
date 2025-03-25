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

let OptionInformation = require('../msg/OptionInformation.js');

//-----------------------------------------------------------

class GetOptionInformationRequest {
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
    // Serializes a message object of type GetOptionInformationRequest
    // Serialize message field [input]
    bufferOffset = OptionIdentifier.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOptionInformationRequest
    let len;
    let data = new GetOptionInformationRequest(null);
    // Deserialize message field [input]
    data.input = OptionIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetOptionInformationRequest';
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
    const resolved = new GetOptionInformationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = OptionIdentifier.Resolve(msg.input)
    }
    else {
      resolved.input = new OptionIdentifier()
    }

    return resolved;
    }
};

class GetOptionInformationResponse {
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
        this.output = new OptionInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOptionInformationResponse
    // Serialize message field [output]
    bufferOffset = OptionInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOptionInformationResponse
    let len;
    let data = new GetOptionInformationResponse(null);
    // Deserialize message field [output]
    data.output = OptionInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 26;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetOptionInformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9eb358fd0dbe985fd71dfcb2e8ac705f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    OptionInformation output
    
    ================================================================================
    MSG: kortex_driver/OptionInformation
    
    uint32 sensor
    uint32 option
    bool supported
    bool read_only
    float32 minimum
    float32 maximum
    float32 step
    float32 default_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOptionInformationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = OptionInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new OptionInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetOptionInformationRequest,
  Response: GetOptionInformationResponse,
  md5sum() { return '0af2e58d9822a8c26f4c5a66d0d6015b'; },
  datatype() { return 'kortex_driver/GetOptionInformation'; }
};
