// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let FullIPv4Configuration = require('../msg/FullIPv4Configuration.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetIPv4ConfigurationRequest {
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
        this.input = new FullIPv4Configuration();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetIPv4ConfigurationRequest
    // Serialize message field [input]
    bufferOffset = FullIPv4Configuration.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIPv4ConfigurationRequest
    let len;
    let data = new SetIPv4ConfigurationRequest(null);
    // Deserialize message field [input]
    data.input = FullIPv4Configuration.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetIPv4ConfigurationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eece4ea981c6bc4a91e7cd235e1fcc3f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    FullIPv4Configuration input
    
    ================================================================================
    MSG: kortex_driver/FullIPv4Configuration
    
    NetworkHandle handle
    IPv4Configuration ipv4_configuration
    ================================================================================
    MSG: kortex_driver/NetworkHandle
    
    uint32 type
    ================================================================================
    MSG: kortex_driver/IPv4Configuration
    
    uint32 ip_address
    uint32 subnet_mask
    uint32 default_gateway
    bool dhcp_enabled
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetIPv4ConfigurationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = FullIPv4Configuration.Resolve(msg.input)
    }
    else {
      resolved.input = new FullIPv4Configuration()
    }

    return resolved;
    }
};

class SetIPv4ConfigurationResponse {
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
    // Serializes a message object of type SetIPv4ConfigurationResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetIPv4ConfigurationResponse
    let len;
    let data = new SetIPv4ConfigurationResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetIPv4ConfigurationResponse';
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
    const resolved = new SetIPv4ConfigurationResponse(null);
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
  Request: SetIPv4ConfigurationRequest,
  Response: SetIPv4ConfigurationResponse,
  md5sum() { return 'efb45709f10aca05c41eab84d8334547'; },
  datatype() { return 'kortex_driver/SetIPv4Configuration'; }
};
