// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UserProfileHandle = require('../msg/UserProfileHandle.js');

//-----------------------------------------------------------

let UserProfile = require('../msg/UserProfile.js');

//-----------------------------------------------------------

class ReadUserProfileRequest {
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
        this.input = new UserProfileHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadUserProfileRequest
    // Serialize message field [input]
    bufferOffset = UserProfileHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadUserProfileRequest
    let len;
    let data = new ReadUserProfileRequest(null);
    // Deserialize message field [input]
    data.input = UserProfileHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadUserProfileRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bece15bd6f474817d8cf8261b2df5e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    UserProfileHandle input
    
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadUserProfileRequest(null);
    if (msg.input !== undefined) {
      resolved.input = UserProfileHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new UserProfileHandle()
    }

    return resolved;
    }
};

class ReadUserProfileResponse {
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
        this.output = new UserProfile();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadUserProfileResponse
    // Serialize message field [output]
    bufferOffset = UserProfile.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadUserProfileResponse
    let len;
    let data = new ReadUserProfileResponse(null);
    // Deserialize message field [output]
    data.output = UserProfile.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += UserProfile.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadUserProfileResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '352d54ed7065f2fa116f682f0b133d92';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    UserProfile output
    
    ================================================================================
    MSG: kortex_driver/UserProfile
    
    UserProfileHandle handle
    string username
    string firstname
    string lastname
    string application_data
    ================================================================================
    MSG: kortex_driver/UserProfileHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadUserProfileResponse(null);
    if (msg.output !== undefined) {
      resolved.output = UserProfile.Resolve(msg.output)
    }
    else {
      resolved.output = new UserProfile()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadUserProfileRequest,
  Response: ReadUserProfileResponse,
  md5sum() { return '88ed6f69b3ce05a4b3c054859c8deeff'; },
  datatype() { return 'kortex_driver/ReadUserProfile'; }
};
