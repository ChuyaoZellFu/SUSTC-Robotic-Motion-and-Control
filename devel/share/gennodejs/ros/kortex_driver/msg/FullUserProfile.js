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

class FullUserProfile {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_profile = null;
      this.password = null;
    }
    else {
      if (initObj.hasOwnProperty('user_profile')) {
        this.user_profile = initObj.user_profile
      }
      else {
        this.user_profile = new UserProfile();
      }
      if (initObj.hasOwnProperty('password')) {
        this.password = initObj.password
      }
      else {
        this.password = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FullUserProfile
    // Serialize message field [user_profile]
    bufferOffset = UserProfile.serialize(obj.user_profile, buffer, bufferOffset);
    // Serialize message field [password]
    bufferOffset = _serializer.string(obj.password, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FullUserProfile
    let len;
    let data = new FullUserProfile(null);
    // Deserialize message field [user_profile]
    data.user_profile = UserProfile.deserialize(buffer, bufferOffset);
    // Deserialize message field [password]
    data.password = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += UserProfile.getMessageSize(object.user_profile);
    length += _getByteLength(object.password);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/FullUserProfile';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5832f6aa4b0c784ce33a85ff505da582';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    UserProfile user_profile
    string password
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
    const resolved = new FullUserProfile(null);
    if (msg.user_profile !== undefined) {
      resolved.user_profile = UserProfile.Resolve(msg.user_profile)
    }
    else {
      resolved.user_profile = new UserProfile()
    }

    if (msg.password !== undefined) {
      resolved.password = msg.password;
    }
    else {
      resolved.password = ''
    }

    return resolved;
    }
};

module.exports = FullUserProfile;
