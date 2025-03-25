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

class UserList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_handles = null;
    }
    else {
      if (initObj.hasOwnProperty('user_handles')) {
        this.user_handles = initObj.user_handles
      }
      else {
        this.user_handles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UserList
    // Serialize message field [user_handles]
    // Serialize the length for message field [user_handles]
    bufferOffset = _serializer.uint32(obj.user_handles.length, buffer, bufferOffset);
    obj.user_handles.forEach((val) => {
      bufferOffset = UserProfileHandle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UserList
    let len;
    let data = new UserList(null);
    // Deserialize message field [user_handles]
    // Deserialize array length for message field [user_handles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.user_handles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.user_handles[i] = UserProfileHandle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.user_handles.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UserList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fd1c8a5c2edbaa168a73081302b5bac5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    UserProfileHandle[] user_handles
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
    const resolved = new UserList(null);
    if (msg.user_handles !== undefined) {
      resolved.user_handles = new Array(msg.user_handles.length);
      for (let i = 0; i < resolved.user_handles.length; ++i) {
        resolved.user_handles[i] = UserProfileHandle.Resolve(msg.user_handles[i]);
      }
    }
    else {
      resolved.user_handles = []
    }

    return resolved;
    }
};

module.exports = UserList;
