// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CommunicationInterfaceConfiguration = require('../msg/CommunicationInterfaceConfiguration.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetCommunicationInterfaceEnableRequest {
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
        this.input = new CommunicationInterfaceConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCommunicationInterfaceEnableRequest
    // Serialize message field [input]
    bufferOffset = CommunicationInterfaceConfiguration.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCommunicationInterfaceEnableRequest
    let len;
    let data = new SetCommunicationInterfaceEnableRequest(null);
    // Deserialize message field [input]
    data.input = CommunicationInterfaceConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetCommunicationInterfaceEnableRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '98d776df460a80c9b3e32187fc3e4632';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CommunicationInterfaceConfiguration input
    
    ================================================================================
    MSG: kortex_driver/CommunicationInterfaceConfiguration
    
    uint32 type
    bool enable
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCommunicationInterfaceEnableRequest(null);
    if (msg.input !== undefined) {
      resolved.input = CommunicationInterfaceConfiguration.Resolve(msg.input)
    }
    else {
      resolved.input = new CommunicationInterfaceConfiguration()
    }

    return resolved;
    }
};

class SetCommunicationInterfaceEnableResponse {
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
    // Serializes a message object of type SetCommunicationInterfaceEnableResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCommunicationInterfaceEnableResponse
    let len;
    let data = new SetCommunicationInterfaceEnableResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetCommunicationInterfaceEnableResponse';
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
    const resolved = new SetCommunicationInterfaceEnableResponse(null);
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
  Request: SetCommunicationInterfaceEnableRequest,
  Response: SetCommunicationInterfaceEnableResponse,
  md5sum() { return 'ea61eebb1ea7afcd99f7fa2e0cb8d9db'; },
  datatype() { return 'kortex_driver/SetCommunicationInterfaceEnable'; }
};
