// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UserProfileHandle = require('./UserProfileHandle.js');

//-----------------------------------------------------------

class UserProfile {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.username = null;
      this.firstname = null;
      this.lastname = null;
      this.application_data = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new UserProfileHandle();
      }
      if (initObj.hasOwnProperty('username')) {
        this.username = initObj.username
      }
      else {
        this.username = '';
      }
      if (initObj.hasOwnProperty('firstname')) {
        this.firstname = initObj.firstname
      }
      else {
        this.firstname = '';
      }
      if (initObj.hasOwnProperty('lastname')) {
        this.lastname = initObj.lastname
      }
      else {
        this.lastname = '';
      }
      if (initObj.hasOwnProperty('application_data')) {
        this.application_data = initObj.application_data
      }
      else {
        this.application_data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UserProfile
    // Serialize message field [handle]
    bufferOffset = UserProfileHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [username]
    bufferOffset = _serializer.string(obj.username, buffer, bufferOffset);
    // Serialize message field [firstname]
    bufferOffset = _serializer.string(obj.firstname, buffer, bufferOffset);
    // Serialize message field [lastname]
    bufferOffset = _serializer.string(obj.lastname, buffer, bufferOffset);
    // Serialize message field [application_data]
    bufferOffset = _serializer.string(obj.application_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UserProfile
    let len;
    let data = new UserProfile(null);
    // Deserialize message field [handle]
    data.handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [username]
    data.username = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [firstname]
    data.firstname = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [lastname]
    data.lastname = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [application_data]
    data.application_data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.username);
    length += _getByteLength(object.firstname);
    length += _getByteLength(object.lastname);
    length += _getByteLength(object.application_data);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UserProfile';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4599942b2b24021432dda0e643774952';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new UserProfile(null);
    if (msg.handle !== undefined) {
      resolved.handle = UserProfileHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new UserProfileHandle()
    }

    if (msg.username !== undefined) {
      resolved.username = msg.username;
    }
    else {
      resolved.username = ''
    }

    if (msg.firstname !== undefined) {
      resolved.firstname = msg.firstname;
    }
    else {
      resolved.firstname = ''
    }

    if (msg.lastname !== undefined) {
      resolved.lastname = msg.lastname;
    }
    else {
      resolved.lastname = ''
    }

    if (msg.application_data !== undefined) {
      resolved.application_data = msg.application_data;
    }
    else {
      resolved.application_data = ''
    }

    return resolved;
    }
};

module.exports = UserProfile;
