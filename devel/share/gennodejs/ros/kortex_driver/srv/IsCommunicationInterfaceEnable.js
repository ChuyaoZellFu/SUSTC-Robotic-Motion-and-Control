// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NetworkHandle = require('../msg/NetworkHandle.js');

//-----------------------------------------------------------

let CommunicationInterfaceConfiguration = require('../msg/CommunicationInterfaceConfiguration.js');

//-----------------------------------------------------------

class IsCommunicationInterfaceEnableRequest {
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
        this.input = new NetworkHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsCommunicationInterfaceEnableRequest
    // Serialize message field [input]
    bufferOffset = NetworkHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsCommunicationInterfaceEnableRequest
    let len;
    let data = new IsCommunicationInterfaceEnableRequest(null);
    // Deserialize message field [input]
    data.input = NetworkHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/IsCommunicationInterfaceEnableRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd83837d5994a891dad63c33420a4fcdf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    NetworkHandle input
    
    ================================================================================
    MSG: kortex_driver/NetworkHandle
    
    uint32 type
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IsCommunicationInterfaceEnableRequest(null);
    if (msg.input !== undefined) {
      resolved.input = NetworkHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new NetworkHandle()
    }

    return resolved;
    }
};

class IsCommunicationInterfaceEnableResponse {
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
        this.output = new CommunicationInterfaceConfiguration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IsCommunicationInterfaceEnableResponse
    // Serialize message field [output]
    bufferOffset = CommunicationInterfaceConfiguration.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IsCommunicationInterfaceEnableResponse
    let len;
    let data = new IsCommunicationInterfaceEnableResponse(null);
    // Deserialize message field [output]
    data.output = CommunicationInterfaceConfiguration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/IsCommunicationInterfaceEnableResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14aad7bb40261a0342e840837a530b52';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CommunicationInterfaceConfiguration output
    
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
    const resolved = new IsCommunicationInterfaceEnableResponse(null);
    if (msg.output !== undefined) {
      resolved.output = CommunicationInterfaceConfiguration.Resolve(msg.output)
    }
    else {
      resolved.output = new CommunicationInterfaceConfiguration()
    }

    return resolved;
    }
};

module.exports = {
  Request: IsCommunicationInterfaceEnableRequest,
  Response: IsCommunicationInterfaceEnableResponse,
  md5sum() { return '7b7c3f92182fedb31e77cfcc39090ac1'; },
  datatype() { return 'kortex_driver/IsCommunicationInterfaceEnable'; }
};
