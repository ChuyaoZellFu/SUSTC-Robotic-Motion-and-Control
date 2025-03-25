// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyHandle = require('../msg/SafetyHandle.js');

//-----------------------------------------------------------

let SafetyEnable = require('../msg/SafetyEnable.js');

//-----------------------------------------------------------

class GetSafetyEnableRequest {
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
        this.input = new SafetyHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyEnableRequest
    // Serialize message field [input]
    bufferOffset = SafetyHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyEnableRequest
    let len;
    let data = new GetSafetyEnableRequest(null);
    // Deserialize message field [input]
    data.input = SafetyHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyEnableRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28fc5544fa28c41b8590a221a4f4db30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyHandle input
    
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSafetyEnableRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SafetyHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new SafetyHandle()
    }

    return resolved;
    }
};

class GetSafetyEnableResponse {
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
        this.output = new SafetyEnable();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyEnableResponse
    // Serialize message field [output]
    bufferOffset = SafetyEnable.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyEnableResponse
    let len;
    let data = new GetSafetyEnableResponse(null);
    // Deserialize message field [output]
    data.output = SafetyEnable.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyEnableResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '47b51a916802be953e870172b42b655e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyEnable output
    
    ================================================================================
    MSG: kortex_driver/SafetyEnable
    
    SafetyHandle handle
    bool enable
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSafetyEnableResponse(null);
    if (msg.output !== undefined) {
      resolved.output = SafetyEnable.Resolve(msg.output)
    }
    else {
      resolved.output = new SafetyEnable()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSafetyEnableRequest,
  Response: GetSafetyEnableResponse,
  md5sum() { return 'd579190d4e318f6ca36b725f2e3b7043'; },
  datatype() { return 'kortex_driver/GetSafetyEnable'; }
};
