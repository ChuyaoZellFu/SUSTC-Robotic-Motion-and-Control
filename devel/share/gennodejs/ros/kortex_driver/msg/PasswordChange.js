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

class PasswordChange {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.old_password = null;
      this.new_password = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new UserProfileHandle();
      }
      if (initObj.hasOwnProperty('old_password')) {
        this.old_password = initObj.old_password
      }
      else {
        this.old_password = '';
      }
      if (initObj.hasOwnProperty('new_password')) {
        this.new_password = initObj.new_password
      }
      else {
        this.new_password = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PasswordChange
    // Serialize message field [handle]
    bufferOffset = UserProfileHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [old_password]
    bufferOffset = _serializer.string(obj.old_password, buffer, bufferOffset);
    // Serialize message field [new_password]
    bufferOffset = _serializer.string(obj.new_password, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PasswordChange
    let len;
    let data = new PasswordChange(null);
    // Deserialize message field [handle]
    data.handle = UserProfileHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [old_password]
    data.old_password = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [new_password]
    data.new_password = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.old_password);
    length += _getByteLength(object.new_password);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/PasswordChange';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75d13d1936094613c5e6bc629ee347b3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    UserProfileHandle handle
    string old_password
    string new_password
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
    const resolved = new PasswordChange(null);
    if (msg.handle !== undefined) {
      resolved.handle = UserProfileHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new UserProfileHandle()
    }

    if (msg.old_password !== undefined) {
      resolved.old_password = msg.old_password;
    }
    else {
      resolved.old_password = ''
    }

    if (msg.new_password !== undefined) {
      resolved.new_password = msg.new_password;
    }
    else {
      resolved.new_password = ''
    }

    return resolved;
    }
};

module.exports = PasswordChange;
