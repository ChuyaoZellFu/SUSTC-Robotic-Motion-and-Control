// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UserProfile = require('./UserProfile.js');

//-----------------------------------------------------------

class UserProfileList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_profiles = null;
    }
    else {
      if (initObj.hasOwnProperty('user_profiles')) {
        this.user_profiles = initObj.user_profiles
      }
      else {
        this.user_profiles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UserProfileList
    // Serialize message field [user_profiles]
    // Serialize the length for message field [user_profiles]
    bufferOffset = _serializer.uint32(obj.user_profiles.length, buffer, bufferOffset);
    obj.user_profiles.forEach((val) => {
      bufferOffset = UserProfile.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UserProfileList
    let len;
    let data = new UserProfileList(null);
    // Deserialize message field [user_profiles]
    // Deserialize array length for message field [user_profiles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.user_profiles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.user_profiles[i] = UserProfile.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.user_profiles.forEach((val) => {
      length += UserProfile.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UserProfileList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d71d5ab953fb8ddd6c9d9d4a1379bb4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    UserProfile[] user_profiles
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
    const resolved = new UserProfileList(null);
    if (msg.user_profiles !== undefined) {
      resolved.user_profiles = new Array(msg.user_profiles.length);
      for (let i = 0; i < resolved.user_profiles.length; ++i) {
        resolved.user_profiles[i] = UserProfile.Resolve(msg.user_profiles[i]);
      }
    }
    else {
      resolved.user_profiles = []
    }

    return resolved;
    }
};

module.exports = UserProfileList;
