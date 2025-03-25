// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LinearTwist = require('../msg/LinearTwist.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetDesiredLinearTwistRequest {
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
        this.input = new LinearTwist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDesiredLinearTwistRequest
    // Serialize message field [input]
    bufferOffset = LinearTwist.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDesiredLinearTwistRequest
    let len;
    let data = new SetDesiredLinearTwistRequest(null);
    // Deserialize message field [input]
    data.input = LinearTwist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetDesiredLinearTwistRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a179e9e98e0cbd858e142aa4e286fd2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    LinearTwist input
    
    ================================================================================
    MSG: kortex_driver/LinearTwist
    
    float32 linear
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDesiredLinearTwistRequest(null);
    if (msg.input !== undefined) {
      resolved.input = LinearTwist.Resolve(msg.input)
    }
    else {
      resolved.input = new LinearTwist()
    }

    return resolved;
    }
};

class SetDesiredLinearTwistResponse {
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
    // Serializes a message object of type SetDesiredLinearTwistResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDesiredLinearTwistResponse
    let len;
    let data = new SetDesiredLinearTwistResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetDesiredLinearTwistResponse';
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
    const resolved = new SetDesiredLinearTwistResponse(null);
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
  Request: SetDesiredLinearTwistRequest,
  Response: SetDesiredLinearTwistResponse,
  md5sum() { return '2fd07256b791580cac358c271b7f0f2a'; },
  datatype() { return 'kortex_driver/SetDesiredLinearTwist'; }
};
